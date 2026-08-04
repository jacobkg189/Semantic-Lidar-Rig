#!/usr/bin/env python3
"""Phase 0 gate: both streams arriving at stable rates.

    python3 mac/phase0_check.py                 # both streams
    python3 mac/phase0_check.py --lidar-only    # no phone needed
    python3 mac/phase0_check.py --phone-only    # no lidar needed

Passing means transport works end to end. It says nothing about whether the two
streams agree with each other — that's Phases 2 and 3.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

# Entry point owns the path setup so the modules themselves stay plain imports.
_ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(_ROOT / "shared"), str(_ROOT / "phase0_transport")]

from lidar_source import SerialLidarSource  # noqa: E402
from phone_link import DEFAULT_HOST, DEFAULT_PORT, PhoneLinkError, PhoneStream  # noqa: E402
from protocol import CameraFrame, Hello, Pose, UnknownMessage  # noqa: E402
from rplidar_c1 import RPLidarError  # noqa: E402

# Nominal rates. The C1 is 10 Hz mechanical; ARKit runs the session at 60 Hz.
EXPECT_LIDAR_HZ = 10.0
EXPECT_POSE_HZ = 60.0
TOLERANCE = 0.6  # fraction of nominal we'll accept before flagging

# A rate meaningfully *above* nominal is a bug, not good news: the hardware
# can't exceed its own clock, so it means messages queued somewhere and got
# stamped in a burst. That corrupts the arrival timestamps Phase 2 depends on,
# which is exactly how a single-threaded drain loop failed here.
BURST_RATIO = 1.25


class Counter:
    """Rate tracker that ignores the spin-up window, since counting dead time
    at the start understates every rate (this bit us on the bench test)."""

    def __init__(self) -> None:
        self.count = 0
        self.first_us: int | None = None
        self.last_us: int = 0

    def tick(self, t_us: int, n: int = 1) -> None:
        if self.first_us is None:
            self.first_us = t_us
        self.last_us = t_us
        self.count += n

    @property
    def elapsed(self) -> float:
        if self.first_us is None or self.last_us <= self.first_us:
            return 0.0
        return (self.last_us - self.first_us) / 1e6

    @property
    def hz(self) -> float:
        return self.count / self.elapsed if self.elapsed > 0 else 0.0


def main() -> int:
    ap = argparse.ArgumentParser(description="Phase 0 transport check")
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--host", default=DEFAULT_HOST)
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--lidar-port", help="serial port override")
    ap.add_argument("--lidar-only", action="store_true")
    ap.add_argument("--phone-only", action="store_true")
    args = ap.parse_args()

    want_lidar = not args.phone_only
    want_phone = not args.lidar_only

    lidar = None
    link = None
    revs, poses, frames = Counter(), Counter(), Counter()
    samples = 0
    other_msgs = 0

    try:
        if want_lidar:
            print("Lidar   opening serial...")
            lidar = SerialLidarSource(port=args.lidar_port)
            lidar.start()
            print(f"Lidar   {lidar.port}")

        if want_phone:
            print(f"Phone   connecting to {args.host}:{args.port}...")
            link = PhoneStream(args.host, args.port)
            link.connect()
            link.start()
            print("Phone   connected, waiting for HELLO")

        print(f"\nSampling for {args.seconds:.0f}s...\n")

        deadline = time.monotonic() + args.seconds

        # Each stream is drained by its own thread and stamped on arrival there.
        # This loop only tallies, so it can poll both without either stalling
        # the other — and without distorting the timestamps it's counting.
        while time.monotonic() < deadline:
            if lidar is not None:
                lidar.check()
                for rev in lidar.drain():
                    revs.tick(rev.t_arrival_us)
                    samples += len(rev.samples)

            if link is not None:
                for msg in link.messages():
                    if isinstance(msg, Pose):
                        poses.tick(msg.t_arrival_us)
                    elif isinstance(msg, CameraFrame):
                        frames.tick(msg.t_arrival_us)
                    elif isinstance(msg, Hello):
                        print(f"Phone   {msg.device_name}, iOS {msg.os_version}")
                        caps = msg.capability_names()
                        print(f"Phone   capabilities: {', '.join(caps) if caps else 'none'}\n")
                    elif isinstance(msg, UnknownMessage):
                        other_msgs += 1

            time.sleep(0.002)  # both queues empty; don't spin the CPU

    except (RPLidarError, PhoneLinkError) as e:
        print(f"\n{type(e).__name__}: {e}")
        return 1
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        if lidar:
            lidar.stop()
        if link:
            link.stop()

    # --- report ---
    print("Results")
    ok = True

    if want_lidar:
        sample_hz = samples / revs.elapsed if revs.elapsed > 0 else 0.0
        print(f"  Lidar revolutions  {revs.count}  ({revs.hz:.1f} Hz, nominal {EXPECT_LIDAR_HZ:.0f})")
        print(f"  Lidar samples      {samples}  ({sample_hz:.0f} Hz)")
        if lidar and lidar.dropped:
            print(f"  !! dropped {lidar.dropped} revolutions — consumer too slow")
            ok = False
        if revs.hz < EXPECT_LIDAR_HZ * TOLERANCE:
            print(f"  !! lidar rate below {EXPECT_LIDAR_HZ * TOLERANCE:.0f} Hz")
            ok = False
        if revs.hz > EXPECT_LIDAR_HZ * BURST_RATIO:
            print(f"  !! lidar rate above nominal — arrivals are bursting, timestamps unreliable")
            ok = False

    if want_phone:
        print(f"  ARKit poses        {poses.count}  ({poses.hz:.1f} Hz, nominal {EXPECT_POSE_HZ:.0f})")
        if frames.count:
            print(f"  Camera frames      {frames.count}  ({frames.hz:.1f} Hz)")
        if other_msgs:
            print(f"  Unknown messages   {other_msgs}  (ignored, forward-compatible)")
        if poses.hz < EXPECT_POSE_HZ * TOLERANCE:
            print(f"  !! pose rate below {EXPECT_POSE_HZ * TOLERANCE:.0f} Hz")
            ok = False
        if poses.hz > EXPECT_POSE_HZ * BURST_RATIO:
            print("  !! pose rate above nominal — arrivals are bursting, timestamps unreliable")
            ok = False
        if link and link.dropped:
            print(f"  !! dropped {link.dropped} phone messages — consumer too slow")
            ok = False
        if not poses.count:
            print("  !! no poses received at all")
            ok = False

    print("\nPHASE 0 PASS" if ok else "\nPHASE 0 FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
