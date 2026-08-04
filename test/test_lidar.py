#!/usr/bin/env python3
"""Bench test for the RPLIDAR C1 — confirms the device is alive, healthy, and
producing sane geometry before any of it gets wired into the mapping pipeline.

    python3 test_lidar.py              # autodetect, scan 5s, draw the room
    python3 test_lidar.py --list       # just show candidate serial ports
    python3 test_lidar.py --port /dev/cu.usbserial-2110 --seconds 10
"""

from __future__ import annotations

import argparse
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "shared"))

from rplidar_c1 import (  # noqa: E402
    DEFAULT_BAUD,
    Measurement,
    RPLidarC1,
    RPLidarError,
    find_ports,
)


def draw_topdown(points: list[Measurement], width: int = 73, height: int = 33) -> str:
    """Top-down ASCII view. Sensor at centre, 0 deg up, angles clockwise."""
    import math

    valid = [p for p in points if p.is_valid]
    if not valid:
        return "  (no valid returns to plot)"

    # Clip the scale to the 95th percentile so one far outlier doesn't squash
    # the whole room into the middle four characters.
    dists = sorted(p.distance_mm for p in valid)
    scale_mm = dists[int(len(dists) * 0.95)] or dists[-1]

    grid = [[" "] * width for _ in range(height)]
    cx, cy = width // 2, height // 2

    for p in valid:
        r = min(p.distance_mm / scale_mm, 1.0)
        theta = math.radians(p.angle_deg)
        # Characters are roughly twice as tall as wide; scale x to compensate.
        x = int(round(cx + r * math.sin(theta) * (width // 2)))
        y = int(round(cy - r * math.cos(theta) * (height // 2)))
        if 0 <= x < width and 0 <= y < height:
            grid[y][x] = "*"

    grid[cy][cx] = "+"
    body = "\n".join("  " + "".join(row) for row in grid)
    return f"{body}\n\n  + = sensor, * = return, edge of plot ~ {scale_mm / 1000:.1f} m"


def summarise(points: list[Measurement], revolutions: int, elapsed: float) -> bool:
    """Print scan statistics. Returns True if the data looks healthy."""
    valid = [p for p in points if p.is_valid]
    dists = [p.distance_mm for p in valid]
    covered = len({int(p.angle_deg) % 360 for p in valid})

    print(f"  Samples          {len(points)}  ({len(points) / elapsed:.0f} Hz)")
    print(f"  Valid returns    {len(valid)}  ({100 * len(valid) / max(len(points), 1):.1f}%)")
    print(f"  Revolutions      {revolutions}  ({revolutions / elapsed:.1f} Hz)")
    print(f"  Angular coverage {covered}/360 degree bins")

    if dists:
        print(f"  Distance         min {min(dists) / 1000:.2f} m   "
              f"median {statistics.median(dists) / 1000:.2f} m   "
              f"max {max(dists) / 1000:.2f} m")
        print(f"  Mean quality     {statistics.mean(p.quality for p in valid):.1f}")

    # Thresholds are deliberately loose — this is a smoke test, not calibration.
    problems = []
    if not valid:
        problems.append("no valid returns at all")
    if revolutions / elapsed < 2:
        problems.append(f"scan rate {revolutions / elapsed:.1f} Hz is very low (motor obstructed?)")
    if covered < 180:
        problems.append(f"only {covered}/360 degrees covered (occluded, or too close to a wall?)")

    for p in problems:
        print(f"  !! {p}")
    return not problems


def main() -> int:
    ap = argparse.ArgumentParser(description="RPLIDAR C1 smoke test")
    ap.add_argument("--port", help="serial port (autodetected if omitted)")
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    ap.add_argument("--seconds", type=float, default=5.0, help="scan duration")
    ap.add_argument("--list", action="store_true", help="list candidate ports and exit")
    ap.add_argument("--no-plot", action="store_true")
    args = ap.parse_args()

    ports = find_ports()

    if args.list:
        print("Candidate serial ports:")
        for p in ports or []:
            print(f"  {p}")
        if not ports:
            print("  (none found — is the adapter plugged in?)")
        return 0

    port = args.port or (ports[0] if ports else None)
    if not port:
        print("No serial port found. Plug in the C1, or pass --port explicitly.")
        print("Run with --list to see what's available.")
        return 1

    print(f"Port   {port} @ {args.baud} baud\n")

    try:
        with RPLidarC1(port, args.baud) as lidar:
            lidar.stop()   # clear any scan left running by a previous crash
            lidar.reset()  # ...and any wedged state that survives a plain stop

            info = lidar.get_info()
            print("Device")
            print(f"  Model            {info.model_name}")
            print(f"  Firmware         {info.firmware}")
            print(f"  Hardware         {info.hardware}")
            print(f"  Serial           {info.serial_number}")

            health = lidar.get_health()
            print(f"  Health           {health.label}", end="")
            print(f" (error 0x{health.error_code:04X})" if health.error_code else "")

            if not health.is_ok:
                print("\nDevice reports a fault. Try a reset and power-cycle before trusting data.")
                return 1

            print(f"\nScanning for {args.seconds:.0f}s — give it a clear 360 if you can...")

            points: list[Measurement] = []
            revolutions = 0
            # Time from the first sample, not from the scan command — the motor
            # spends a couple of seconds spinning up and counting that dead time
            # drags the reported rates well below spec.
            started = None
            for m in lidar.iter_measurements():
                if started is None:
                    started = time.monotonic()
                if m.start_flag:
                    revolutions += 1
                points.append(m)
                if time.monotonic() - started >= args.seconds:
                    break
            elapsed = time.monotonic() - started
            lidar.stop()

    except RPLidarError as e:
        print(f"\nLidar error: {e}")
        return 1
    except PermissionError:
        print(f"\nPermission denied opening {port}. Another process may hold it open.")
        return 1
    except KeyboardInterrupt:
        print("\nInterrupted.")
        return 130

    print("\nResults")
    ok = summarise(points, revolutions, elapsed)

    if not args.no_plot:
        # Plot the last revolution only — a 5s pile-up of scans is unreadable.
        last = points[-int(len(points) / max(revolutions, 1)):] if revolutions else points
        print("\nLast revolution\n")
        print(draw_topdown(last))

    print("\nPASS — device is working." if ok else "\nCHECK — device responded but the data looks off.")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
