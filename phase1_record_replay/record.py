#!/usr/bin/env python3
"""Record a session to disk.

    python3 phase1_record_replay/record.py --seconds 30 --notes "kitchen loop"

Same threaded readers as Phase 0 — one per stream, each stamping arrival at the
moment data lands. That property is the whole point of the recording: Phase 2
solves the clock offset from these timestamps, so anything that bunches them
makes the session useless for sync.
"""

from __future__ import annotations

import argparse
import sys
import time
from datetime import datetime
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [
    str(_ROOT / "shared"),
    str(_ROOT / "phase0_transport"),
    str(_ROOT / "phase1_record_replay"),
]

from lidar_source import SerialLidarSource  # noqa: E402
from phone_link import DEFAULT_HOST, DEFAULT_PORT, PhoneLinkError, PhoneStream  # noqa: E402
from protocol import CameraFrame, Hello, Pose, _POSE  # noqa: E402
from rplidar_c1 import RPLidarError  # noqa: E402
from session import SessionWriter  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description="Record a capture session")
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--out", default="recordings", help="root directory for sessions")
    ap.add_argument("--notes", default="", help="what this recording is of")
    ap.add_argument("--host", default=DEFAULT_HOST)
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--lidar-port")
    ap.add_argument("--no-frames", action="store_true", help="discard camera frames")
    args = ap.parse_args()

    stamp = datetime.now().strftime("%Y-%m-%dT%H-%M-%S")
    out_dir = Path(args.out) / stamp

    lidar = link = None
    try:
        print("Lidar   opening serial...")
        lidar = SerialLidarSource(port=args.lidar_port)
        lidar.start()
        print(f"Lidar   {lidar.port}")

        print(f"Phone   connecting to {args.host}:{args.port}...")
        link = PhoneStream(args.host, args.port)
        link.connect()
        link.start()
        print("Phone   connected")

        with SessionWriter(out_dir, notes=args.notes) as w:
            w.meta["lidar_port"] = lidar.port
            w.meta["started_wall_clock"] = datetime.now().isoformat()
            print(f"\nRecording {args.seconds:.0f}s to {out_dir}/ ...")

            deadline = time.monotonic() + args.seconds
            last_print = 0.0

            while time.monotonic() < deadline:
                lidar.check()
                for rev in lidar.drain():
                    w.write_lidar(rev.t_arrival_us, rev.rev_id, rev.samples)

                for msg in link.messages():
                    if isinstance(msg, Pose):
                        w.write_pose(
                            msg.t_arrival_us,
                            _POSE.pack(
                                msg.t_device_us, *msg.position, *msg.quaternion,
                                *msg.intrinsics, msg.tracking_state,
                            ),
                        )
                    elif isinstance(msg, CameraFrame) and not args.no_frames:
                        w.write_frame(
                            msg.t_arrival_us, msg.t_device_us,
                            msg.width, msg.height, msg.jpeg,
                        )
                    elif isinstance(msg, Hello):
                        w.set_device(msg.device_name, msg.os_version, msg.capability_names())
                        print(f"Phone   {msg.device_name}, iOS {msg.os_version}")

                now = time.monotonic()
                # Carriage-return progress only makes sense on a terminal; piped
                # into a file it turns the whole run into one unreadable line.
                if sys.stdout.isatty() and now - last_print >= 1.0:
                    remaining = deadline - now
                    print(
                        f"  {remaining:4.0f}s left   "
                        f"poses {w.counts['poses']:6d}  "
                        f"revs {w.counts['lidar']:5d}  "
                        f"frames {w.counts['frames']:4d}",
                        end="\r", flush=True,
                    )
                    last_print = now

                time.sleep(0.002)

            counts = dict(w.counts)

    except (RPLidarError, PhoneLinkError) as e:
        print(f"\n{type(e).__name__}: {e}")
        return 1
    except KeyboardInterrupt:
        print("\nInterrupted — manifest still written, session is readable.")
        return 130
    finally:
        if lidar:
            lidar.stop()
        if link:
            link.stop()

    size_mb = sum(f.stat().st_size for f in out_dir.glob("*.bin")) / 1e6
    print(f"\n\nWrote {out_dir}/  ({size_mb:.1f} MB)")
    print(f"  poses  {counts['poses']}")
    print(f"  revs   {counts['lidar']}  ({counts['lidar_samples']} samples)")
    print(f"  frames {counts['frames']}")
    print(f"\nVerify:  python3 phase1_record_replay/check.py {out_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
