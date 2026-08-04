"""Minimal RPLIDAR C1 driver, speaking the Slamtec serial protocol directly.

Deliberately dependency-light (pyserial only) so it can move into the bridge
service later without dragging a third-party lidar library along. The common
Python RPLIDAR packages target the A1/A2 and get the C1's baud rate and express
modes wrong, which is more trouble than the ~200 lines it saves.

Protocol reference: request frames are 0xA5 <cmd>, responses open with a 7-byte
descriptor (0xA5 0x5A, 30-bit length + 2-bit mode, 1-byte data type).
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Iterator

import serial
from serial.tools import list_ports

# The C1 runs at 460800, unlike the A1's 115200. Wrong baud looks exactly like
# a dead device, so this is the first thing to check when nothing responds.
DEFAULT_BAUD = 460800

SYNC0, SYNC1 = 0xA5, 0x5A

CMD_STOP = 0x25
CMD_RESET = 0x40
CMD_SCAN = 0x20
CMD_GET_INFO = 0x50
CMD_GET_HEALTH = 0x52

DTYPE_INFO = 0x04
DTYPE_HEALTH = 0x06
DTYPE_SCAN = 0x81

HEALTH_STATUS = {0: "Good", 1: "Warning", 2: "Error"}

# Slamtec doesn't publish a full model-id table; annotate what we know and pass
# anything else through unlabelled rather than guessing at a wrong name.
KNOWN_MODELS = {0x18: "A1", 0x28: "A2", 0x41: "C1"}


class RPLidarError(Exception):
    pass


@dataclass
class DeviceInfo:
    model_id: int
    firmware: str
    hardware: int
    serial_number: str

    @property
    def model_name(self) -> str:
        return KNOWN_MODELS.get(self.model_id, f"unrecognised (0x{self.model_id:02X})")


@dataclass
class Health:
    status: int
    error_code: int

    @property
    def label(self) -> str:
        return HEALTH_STATUS.get(self.status, f"unknown ({self.status})")

    @property
    def is_ok(self) -> bool:
        return self.status == 0


@dataclass
class Measurement:
    angle_deg: float
    distance_mm: float
    quality: int
    start_flag: bool

    @property
    def is_valid(self) -> bool:
        # A zero distance means the beam found nothing in range, not a bad read.
        return self.distance_mm > 0


def find_ports() -> list[str]:
    """Serial ports that plausibly belong to a lidar adapter, best guess first."""
    skip = ("Bluetooth", "debug-console")
    hits = ("usbserial", "usbmodem", "SLAB_USBtoUART", "wchusbserial", "ttyUSB", "ttyACM")
    found = []
    for port in list_ports.comports():
        if any(s in port.device for s in skip):
            continue
        if any(h in port.device for h in hits):
            found.append(port.device)
    # On macOS prefer /dev/cu.* over /dev/tty.* — the tty variant blocks on DCD.
    found.sort(key=lambda d: (".tty." in d, d))
    return found


class RPLidarC1:
    def __init__(self, port: str, baudrate: int = DEFAULT_BAUD, timeout: float = 1.0):
        self.port = port
        self._serial = serial.Serial(port, baudrate, timeout=timeout)
        # Slamtec adapters gate the motor on DTR, inverted: low means spin.
        self._serial.dtr = False
        self._buf = bytearray()
        self._scanning = False
        time.sleep(0.05)
        self._serial.reset_input_buffer()

    def __enter__(self) -> "RPLidarC1":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # --- framing -----------------------------------------------------------

    def _send(self, cmd: int) -> None:
        self._serial.write(bytes([SYNC0, cmd]))
        self._serial.flush()

    def _read_exact(self, n: int, timeout: float = 2.0) -> bytes:
        deadline = time.monotonic() + timeout
        out = bytearray()
        while len(out) < n:
            chunk = self._serial.read(n - len(out))
            if chunk:
                out += chunk
            elif time.monotonic() > deadline:
                raise RPLidarError(f"timed out reading {n} bytes (got {len(out)})")
        return bytes(out)

    def _read_descriptor(self) -> tuple[int, int, int]:
        # Resync on the sync pair rather than assuming the stream is aligned;
        # a previous aborted scan can leave junk in the buffer.
        deadline = time.monotonic() + 2.0
        window = bytearray()
        while True:
            window += self._read_exact(1, timeout=2.0)
            if len(window) > 2:
                window = window[-2:]
            if bytes(window) == bytes([SYNC0, SYNC1]):
                break
            if time.monotonic() > deadline:
                raise RPLidarError("no response descriptor (check baud rate and cabling)")

        rest = self._read_exact(5)
        length = int.from_bytes(rest[0:4], "little") & 0x3FFFFFFF
        mode = rest[3] >> 6
        dtype = rest[4]
        return length, mode, dtype

    # --- commands ----------------------------------------------------------

    def get_info(self) -> DeviceInfo:
        self._send(CMD_GET_INFO)
        length, _, dtype = self._read_descriptor()
        if dtype != DTYPE_INFO or length != 20:
            raise RPLidarError(f"unexpected info response (type=0x{dtype:02X} len={length})")
        d = self._read_exact(20)
        return DeviceInfo(
            model_id=d[0],
            firmware=f"{d[2]}.{d[1]}",
            hardware=d[3],
            serial_number=d[4:20][::-1].hex().upper(),
        )

    def get_health(self) -> Health:
        self._send(CMD_GET_HEALTH)
        length, _, dtype = self._read_descriptor()
        if dtype != DTYPE_HEALTH or length != 3:
            raise RPLidarError(f"unexpected health response (type=0x{dtype:02X} len={length})")
        d = self._read_exact(3)
        return Health(status=d[0], error_code=int.from_bytes(d[1:3], "little"))

    def reset(self) -> None:
        """Reboot the device. Worth doing before every scan: the C1 will happily
        ACK a SCAN command while sitting in a wedged state from a previous run,
        returning a valid descriptor and then no samples at all."""
        self._send(CMD_RESET)
        time.sleep(0.5)
        # Drain the ASCII boot banner, then wait for the line to go quiet.
        quiet_since = time.monotonic()
        while time.monotonic() - quiet_since < 0.3:
            if self._serial.read(256):
                quiet_since = time.monotonic()
        self._serial.reset_input_buffer()
        self._buf.clear()

    def stop(self) -> None:
        self._send(CMD_STOP)
        self._scanning = False
        time.sleep(0.05)
        self._serial.reset_input_buffer()
        self._buf.clear()

    def close(self) -> None:
        # Teardown is best-effort. If the adapter has already fallen off the USB
        # bus these writes raise, and letting that propagate would replace the
        # real failure with a confusing one from the cleanup path.
        try:
            if self._scanning:
                self.stop()
            self._serial.dtr = True  # park the motor
        except Exception:
            pass
        finally:
            try:
                self._serial.close()
            except Exception:
                pass

    # --- scanning ----------------------------------------------------------

    def iter_measurements(self, spin_up: float = 8.0, stall: float = 2.0) -> Iterator[Measurement]:
        """Yield individual returns forever. Caller decides when to stop.

        The descriptor comes back immediately but the motor needs a couple of
        seconds to reach speed, so the first sample gets a longer grace period
        than the steady-state stall timeout.
        """
        self._send(CMD_SCAN)
        _, _, dtype = self._read_descriptor()
        if dtype != DTYPE_SCAN:
            raise RPLidarError(f"unexpected scan response type 0x{dtype:02X}")
        self._scanning = True

        streaming = False
        deadline = time.monotonic() + spin_up

        while True:
            # Read in chunks — byte-at-a-time can't keep up with 5 kHz of samples.
            want = max(self._serial.in_waiting, 1)
            chunk = self._serial.read(want)
            if not chunk:
                if time.monotonic() > deadline:
                    raise RPLidarError(
                        "scan stream stalled (motor stopped or cable dropped)" if streaming
                        else f"no samples within {spin_up:.0f}s of starting the scan "
                             "(motor not spinning — check 5V supply and that the belt is clear)"
                    )
                continue
            streaming = True
            deadline = time.monotonic() + stall
            self._buf += chunk

            while len(self._buf) >= 5:
                packet = self._buf[:5]
                start = packet[0] & 0x01
                inv_start = (packet[0] >> 1) & 0x01
                check = packet[1] & 0x01
                if start == inv_start or check != 1:
                    del self._buf[0]  # misaligned — slide one byte and retry
                    continue
                del self._buf[:5]
                yield Measurement(
                    angle_deg=((packet[1] >> 1) | (packet[2] << 7)) / 64.0,
                    distance_mm=(packet[3] | (packet[4] << 8)) / 4.0,
                    quality=packet[0] >> 2,
                    start_flag=bool(start),
                )

    def iter_scans(self) -> Iterator[list[Measurement]]:
        """Group returns into full revolutions using the device's start flag."""
        current: list[Measurement] = []
        for m in self.iter_measurements():
            if m.start_flag and current:
                yield current
                current = []
            current.append(m)
