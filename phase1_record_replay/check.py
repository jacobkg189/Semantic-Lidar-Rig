#!/usr/bin/env python3
"""Phase 1 gate: a recording replays deterministically and losslessly.

    python3 phase1_record_replay/check.py [session_dir]

With no argument it checks the most recent session. Needs no hardware — that's
the point of the phase.

Four things get checked, because "replay is deterministic" alone is nearly
vacuous (re-reading a file twice usually does match). What matters is that the
recording is *faithful* to what arrived, and still usable as a clock source.
"""

from __future__ import annotations

import argparse
import hashlib
import statistics
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(_ROOT / "shared"), str(_ROOT / "phase1_record_replay")]

from protocol import MsgType, decode  # noqa: E402
from session import (  # noqa: E402
    LIDAR_SAMPLE,
    Record,
    SessionReader,
    Stream,
    decode_lidar,
    latest_session,
)

EXPECT_POSE_HZ = 60.0
EXPECT_LIDAR_HZ = 10.0


def digest(records) -> str:
    h = hashlib.sha256()
    for r in records:
        h.update(r.stream.value.to_bytes(1, "little"))
        h.update(r.t_arrival_us.to_bytes(8, "little"))
        h.update(r.payload)
    return h.hexdigest()


def check_determinism(reader: SessionReader) -> tuple[bool, str]:
    a = digest(reader.records())
    b = digest(reader.records())
    return a == b, a


def check_counts(reader: SessionReader) -> bool:
    want = reader.counts
    got = {
        "poses": sum(1 for _ in reader.poses()),
        "lidar": sum(1 for _ in reader.lidar()),
        "frames": sum(1 for _ in reader.frames()),
    }
    ok = True
    for key in ("poses", "lidar", "frames"):
        if want.get(key) != got[key]:
            print(f"  !! {key}: manifest says {want.get(key)}, file holds {got[key]}")
            ok = False
    return ok


def check_lossless(reader: SessionReader) -> bool:
    """Every pose payload must decode, and every lidar sample must survive the
    float round trip exactly — the C1's native units are fixed-point, so storing
    them as q6/q2 should be lossless, not merely close."""
    ok = True

    n = 0
    for rec in reader.poses():
        msg = decode(MsgType.POSE, rec.payload)
        if msg is None or not hasattr(msg, "position"):
            print("  !! a pose payload failed to decode")
            ok = False
            break
        n += 1
    if n == 0:
        print("  !! no poses in session")
        ok = False

    bad = 0
    checked = 0
    for rec in reader.lidar():
        _rev, _t, samples = decode_lidar(rec.payload)
        for angle, dist, _q in samples:
            # q6 and q2 quantisation: these must land exactly on the grid.
            if abs(angle * 64 - round(angle * 64)) > 1e-6:
                bad += 1
            if abs(dist * 4 - round(dist * 4)) > 1e-6:
                bad += 1
            checked += 1
    if bad:
        print(f"  !! {bad} lidar values are off the fixed-point grid (lossy round trip)")
        ok = False
    print(f"  Round trip         {n} poses decoded, {checked} lidar samples exact")
    return ok


def check_timing(reader: SessionReader) -> bool:
    """Arrival timestamps must still look like a live stream. If recording
    bunched them, Phase 2 has nothing to solve from — and this is the only
    place that failure is visible before it becomes a maths problem."""
    ok = True

    for label, records, nominal in (
        ("poses", reader.poses(), EXPECT_POSE_HZ),
        ("lidar", reader.lidar(), EXPECT_LIDAR_HZ),
    ):
        stamps = [r.t_arrival_us for r in records]
        if len(stamps) < 10:
            print(f"  !! too few {label} records to assess timing")
            ok = False
            continue

        if stamps != sorted(stamps):
            print(f"  !! {label} arrival timestamps are not monotonic")
            ok = False

        gaps = [(b - a) / 1e6 for a, b in zip(stamps, stamps[1:])]
        median = statistics.median(gaps)
        hz = 1.0 / median if median > 0 else 0.0
        worst = max(gaps)

        print(f"  {label:<18} {hz:5.1f} Hz median, worst gap {worst * 1000:.0f} ms")

        if not (nominal * 0.6 <= hz <= nominal * 1.4):
            print(f"  !! {label} median rate {hz:.1f} Hz is far from nominal {nominal:.0f}")
            ok = False

        # A burst shows up as a cluster of near-zero gaps: messages that queued
        # somewhere and were stamped together rather than on arrival.
        near_zero = sum(1 for g in gaps if g < median * 0.1)
        if near_zero > len(gaps) * 0.05:
            print(f"  !! {near_zero}/{len(gaps)} {label} gaps are near-zero — arrivals bunched")
            ok = False
    return ok


def main() -> int:
    ap = argparse.ArgumentParser(description="Phase 1 record/replay gate")
    ap.add_argument("session", nargs="?", help="session dir (default: most recent)")
    ap.add_argument("--root", default="recordings")
    args = ap.parse_args()

    try:
        path = Path(args.session) if args.session else latest_session(Path(args.root))
    except FileNotFoundError as e:
        print(f"{e}\nRecord one first:  python3 phase1_record_replay/record.py --seconds 20")
        return 1

    try:
        reader = SessionReader(path)
    except (FileNotFoundError, ValueError) as e:
        print(f"Cannot read session: {e}")
        return 1

    print(f"Session {path}")
    dev = reader.meta.get("device", {})
    if dev:
        print(f"Device  {dev.get('name')}, iOS {dev.get('os')}")
    if reader.meta.get("notes"):
        print(f"Notes   {reader.meta['notes']}")
    if reader.meta.get("git_commit"):
        print(f"Commit  {reader.meta['git_commit']}")
    print()

    print("Checks")
    deterministic, sha = check_determinism(reader)
    print(f"  Deterministic      {'yes' if deterministic else 'NO'}  sha256 {sha[:16]}")
    if not deterministic:
        print("  !! two replays of the same file disagree")

    counts_ok = check_counts(reader)
    if counts_ok:
        c = reader.counts
        print(f"  Counts match       {c.get('poses')} poses, "
              f"{c.get('lidar')} revs, {c.get('frames')} frames")

    lossless_ok = check_lossless(reader)
    timing_ok = check_timing(reader)

    ok = deterministic and counts_ok and lossless_ok and timing_ok
    print("\nPHASE 1 PASS" if ok else "\nPHASE 1 FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
