"""Lidar revolutions, on a background thread, behind a swappable interface.

Two reasons this isn't just a call to `iter_measurements()`:

  * the C1 streams at ~5 kHz and blocks, so it needs its own thread to avoid
    starving the phone socket;
  * v0 reads the C1 over local USB, but the mast build will read it off a Pi
    over the network. Consumers depend on `LidarSource`, so that swap is one
    new subclass and no changes downstream.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from queue import Empty, Full, Queue
from typing import Iterator

from rplidar_c1 import Measurement, RPLidarC1, RPLidarError, find_ports


def now_us() -> int:
    return time.time_ns() // 1000


@dataclass
class Revolution:
    rev_id: int
    samples: list[Measurement]
    # Arrival time of the revolution's *last* sample. Phase 4 deskews using
    # per-sample interpolation; this is only for rate accounting.
    t_arrival_us: int = field(default_factory=now_us)

    @property
    def valid_count(self) -> int:
        return sum(1 for s in self.samples if s.is_valid)


class LidarSource:
    """Interface. Subclasses yield revolutions from wherever they come from."""

    def revolutions(self) -> Iterator[Revolution]:
        raise NotImplementedError

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    @property
    def dropped(self) -> int:
        return 0


class SerialLidarSource(LidarSource):
    """C1 on local USB — the v0 configuration."""

    def __init__(self, port: str | None = None, baud: int | None = None, queue_size: int = 64):
        if port is None:
            ports = find_ports()
            if not ports:
                raise RPLidarError("no lidar serial port found — is the C1 plugged in?")
            port = ports[0]
        self.port = port
        self._baud = baud
        self._queue: Queue[Revolution] = Queue(maxsize=queue_size)
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._dropped = 0
        self._error: Exception | None = None

    @property
    def dropped(self) -> int:
        """Revolutions discarded because the consumer fell behind. Non-zero here
        means the consumer is too slow, not that the lidar misbehaved."""
        return self._dropped

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="lidar", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=3.0)

    def _run(self) -> None:
        try:
            kwargs = {"baudrate": self._baud} if self._baud else {}
            with RPLidarC1(self.port, **kwargs) as lidar:
                lidar.stop()
                lidar.reset()  # the C1 will ACK a scan while wedged and send nothing
                rev_id = 0
                current: list[Measurement] = []
                for m in lidar.iter_measurements():
                    if self._stop.is_set():
                        break
                    if m.start_flag and current:
                        self._publish(Revolution(rev_id, current))
                        rev_id += 1
                        current = []
                    current.append(m)
        except Exception as e:  # surfaced to the consumer via check()
            # Wrap non-lidar errors (a yanked USB cable surfaces as a bare
            # SerialException) so consumers only ever catch one type.
            self._error = e if isinstance(e, RPLidarError) else RPLidarError(
                f"lidar reader stopped: {e}"
            )

    def _publish(self, rev: Revolution) -> None:
        try:
            self._queue.put_nowait(rev)
        except Full:
            # Drop the oldest rather than blocking the reader thread — a stalled
            # reader would back up the serial buffer and corrupt the stream.
            try:
                self._queue.get_nowait()
                self._queue.put_nowait(rev)
            except (Empty, Full):
                pass
            self._dropped += 1

    def check(self) -> None:
        """Re-raise anything the reader thread died of."""
        if self._error:
            raise self._error

    def revolutions(self) -> Iterator[Revolution]:
        while not self._stop.is_set():
            self.check()
            try:
                yield self._queue.get(timeout=0.5)
            except Empty:
                continue

    def drain(self) -> Iterator[Revolution]:
        """Yield whatever has queued up, then return. Non-blocking, so a caller
        polling several sensors is never stalled by a quiet one."""
        while True:
            try:
                yield self._queue.get_nowait()
            except Empty:
                return
