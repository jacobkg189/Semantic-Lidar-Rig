"""Receives the ARKit stream from the iPhone over TCP.

The phone listens and the Mac connects, because that is the direction usbmuxd
tunnels — `iproxy 5555 5555` maps a Mac-local port onto a port on the device, so
we dial 127.0.0.1 and land on the phone.

Transport lives behind `PhoneLink` so the Pi-on-the-mast build later swaps the
socket setup without touching anything that consumes poses.
"""

from __future__ import annotations

import socket
import threading
import time
from queue import Empty, Full, Queue
from typing import Iterator

from protocol import (
    HEADER,
    PROTOCOL_VERSION,
    CameraFrame,
    Hello,
    Pose,
    ProtocolError,
    decode,
)

DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 5555

# A single frame should never be this big; if we read a length larger than this
# the stream is desynchronised and bailing beats allocating a garbage buffer.
MAX_MESSAGE_BYTES = 16 * 1024 * 1024


def now_us() -> int:
    return time.time_ns() // 1000


class PhoneLinkError(Exception):
    pass


class PhoneLink:
    def __init__(self, host: str = DEFAULT_HOST, port: int = DEFAULT_PORT, timeout: float = 5.0):
        self.host, self.port = host, port
        self._timeout = timeout
        self._sock: socket.socket | None = None
        self._buf = bytearray()
        self.hello: Hello | None = None

    def __enter__(self) -> "PhoneLink":
        self.connect()
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def connect(self) -> None:
        try:
            self._sock = socket.create_connection((self.host, self.port), self._timeout)
        except ConnectionRefusedError as e:
            raise PhoneLinkError(
                f"nothing listening on {self.host}:{self.port}. Is iproxy running, "
                "and is the app in the foreground on an unlocked phone?"
            ) from e
        except OSError as e:
            raise PhoneLinkError(f"could not reach {self.host}:{self.port}: {e}") from e
        # Latency matters more than packing efficiency for 53-byte poses.
        self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._sock.settimeout(self._timeout)

    def close(self) -> None:
        if self._sock:
            self._sock.close()
            self._sock = None

    def _fill(self, n: int) -> None:
        """Read until the buffer holds at least n bytes."""
        while len(self._buf) < n:
            try:
                chunk = self._sock.recv(65536)
            except socket.timeout as e:
                raise PhoneLinkError("phone stream went quiet (app backgrounded?)") from e
            if not chunk:
                raise PhoneLinkError("phone closed the connection")
            self._buf += chunk

    def messages(self) -> Iterator[object]:
        """Yield decoded messages forever, stamping each with arrival time."""
        if not self._sock:
            raise PhoneLinkError("not connected")

        while True:
            self._fill(HEADER.size)
            length, type_id = HEADER.unpack_from(self._buf, 0)
            if length > MAX_MESSAGE_BYTES:
                raise PhoneLinkError(
                    f"implausible message length {length} — stream desynchronised"
                )
            self._fill(HEADER.size + length)
            payload = bytes(self._buf[HEADER.size : HEADER.size + length])
            del self._buf[: HEADER.size + length]

            arrival = now_us()
            try:
                msg = decode(type_id, payload)
            except (ProtocolError, Exception) as e:
                raise PhoneLinkError(f"could not decode message type 0x{type_id:02X}: {e}") from e

            if isinstance(msg, Hello):
                if msg.protocol_version != PROTOCOL_VERSION:
                    raise PhoneLinkError(
                        f"protocol mismatch: phone speaks v{msg.protocol_version}, "
                        f"Mac speaks v{PROTOCOL_VERSION}. Rebuild the iOS app."
                    )
                self.hello = msg
            elif isinstance(msg, (Pose, CameraFrame)):
                msg.t_arrival_us = arrival

            yield msg


class PhoneStream:
    """`PhoneLink` on its own thread.

    Arrival timestamps are what Phase 2 solves the clock offset from, so they
    are only meaningful if the socket is drained the instant data lands. A
    single loop alternating between sensors stamps each message *after* it
    finishes waiting on the other one, which bunches 60 Hz poses into bursts —
    the rate still averages out, but every individual timestamp is wrong.

    One thread per stream keeps each stamp honest.
    """

    def __init__(self, host: str = DEFAULT_HOST, port: int = DEFAULT_PORT, queue_size: int = 4096):
        self.link = PhoneLink(host, port)
        self._queue: Queue = Queue(maxsize=queue_size)
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._error: Exception | None = None
        self._dropped = 0

    @property
    def dropped(self) -> int:
        return self._dropped

    def connect(self) -> None:
        self.link.connect()

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="phone", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self.link.close()
        if self._thread:
            self._thread.join(timeout=2.0)

    def _run(self) -> None:
        try:
            for msg in self.link.messages():
                if self._stop.is_set():
                    break
                try:
                    self._queue.put_nowait(msg)
                except Full:
                    self._dropped += 1
        except Exception as e:
            if not self._stop.is_set():
                self._error = e

    def check(self) -> None:
        if self._error:
            raise self._error

    def messages(self) -> Iterator[object]:
        """Drain whatever has arrived. Non-blocking, so the caller can poll both
        streams without either one stalling the other."""
        self.check()
        while True:
            try:
                yield self._queue.get_nowait()
            except Empty:
                return
