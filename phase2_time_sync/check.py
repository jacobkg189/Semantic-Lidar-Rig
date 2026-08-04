#!/usr/bin/env python3
"""Phase 2 gate: the clock offset is stable across independent recordings.

    python3 phase2_time_sync/check.py                  # all sessions found
    python3 phase2_time_sync/check.py rec/a rec/b rec/c

Stability across separate captures is the whole test. A single recording will
always produce *a* number; only agreement between recordings shows that the
number means something.

Needs recordings with deliberate rotation — see the phase README. Sessions
without enough motion are reported as inconclusive rather than being folded into
the result, because a confident-looking answer fitted to noise is worse than no
answer.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

_ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [
    str(_ROOT / "shared"),
    str(_ROOT / "phase1_record_replay"),
    str(_ROOT / "phase2_time_sync"),
]

from protocol import MsgType, decode  # noqa: E402
from session import SessionReader, decode_lidar  # noqa: E402
from sync import align_lidar, fit_clock  # noqa: E402

# "A few milliseconds", made concrete.
LAG_SPREAD_LIMIT_US = 10_000.0
LATENCY_SPREAD_LIMIT_US = 2_000.0

# Skew is judged by the timing error it *causes*, not by its ppm spread.
#
# Segments are bounded by the ~43 s resync period, so no recording can ever give
# a longer baseline for the slope — and slope precision is roughly
# (latency noise)/(segment length) ≈ 1 ms / 43 s ≈ 23 ppm. Demanding tighter ppm
# agreement than that asks for more than the data contains.
#
# What matters downstream is how far a timestamp can be wrong. A skew spread of
# S ppm over a segment of D seconds is S·D microseconds of error, and that has
# to stay well inside the 16.7 ms pose interval.
SEGMENT_SECONDS = 43.0
TIMING_ERROR_BUDGET_US = 3_000.0


def load(path: Path):
    r = SessionReader(path)
    dev_t, arr_t, quats = [], [], []
    for rec in r.poses():
        p = decode(MsgType.POSE, rec.payload)
        dev_t.append(p.t_device_us)
        arr_t.append(rec.t_arrival_us)
        quats.append(p.quaternion)

    rev_t, rev_s = [], []
    for rec in r.lidar():
        _rev, t, samples = decode_lidar(rec.payload)
        rev_t.append(t)
        rev_s.append(samples)

    return (
        r,
        np.asarray(dev_t, np.float64),
        np.asarray(arr_t, np.float64),
        np.asarray(quats, np.float64),
        np.asarray(rev_t, np.float64),
        rev_s,
    )


def main() -> int:
    ap = argparse.ArgumentParser(description="Phase 2 time-sync gate")
    ap.add_argument("sessions", nargs="*")
    ap.add_argument("--root", default="recordings")
    args = ap.parse_args()

    paths = [Path(p) for p in args.sessions] or sorted(
        p for p in Path(args.root).glob("*") if (p / "manifest.json").exists()
    )
    if not paths:
        print(f"No sessions under {args.root}/.")
        return 1

    skews, latencies, lags, usable = [], [], [], []

    for path in paths:
        try:
            r, dev_t, arr_t, quats, rev_t, rev_s = load(path)
        except Exception as e:
            print(f"{path.name}: unreadable ({e})\n")
            continue

        if len(dev_t) < 10:
            print(f"{path.name}: too few poses\n")
            continue

        clock = fit_clock(dev_t, arr_t)
        pose_mac_t = clock.to_mac(dev_t)
        align = align_lidar(pose_mac_t, quats, rev_t, rev_s)

        note = r.meta.get("notes", "")
        print(f"{path.name}{'  — ' + note if note else ''}")
        print(f"  clock offset     {clock.offset_us / 1000:12.3f} ms")
        if clock.skew_reliable and clock.skew_large:
            print(f"  clock skew       {clock.skew_ppm:12.1f} ppm  "
                  "(beyond crystal drift — separate capture clock, modelled)")
        elif clock.skew_reliable:
            print(f"  clock skew       {clock.skew_ppm:12.1f} ppm "
                  f"(± {clock.skew_uncertainty_ppm:.0f})")
        else:
            print(f"  clock skew            unresolved — {clock.duration_s:.0f}s recording, "
                  f"±{clock.skew_uncertainty_ppm:.0f} ppm uncertainty")
        if clock.jump_count:
            print(f"  clock resyncs    {clock.jump_count:12d}  "
                  f"(median step {clock.jump_median_us / 1000:.1f} ms)")
        print(f"  transport latency  p50 {clock.latency_p50_us / 1000:.2f} ms, "
              f"p95 {clock.latency_p95_us / 1000:.2f} ms")
        print(f"  rotation          {align.motion_rms_dps:12.1f} deg/s rms yaw "
              f"({align.n_revolutions} revs, tilt {align.tilt_rms_dps:.0f} deg/s "
              f"= {100 * align.tilt_ratio:.0f}%)")

        if align.confident:
            print(f"  lidar lag        {align.lag_us / 1000:12.1f} ms  "
                  f"(corr {align.correlation:.2f}, yaw sign {align.sign:+d})")
            lags.append(align.lag_us)
            usable.append(path.name)
        else:
            print(f"  lidar lag        inconclusive — {align.why_not}")

        if clock.skew_reliable:
            skews.append(clock.skew_ppm)
        latencies.append(clock.latency_p50_us)
        print()

    # --- verdict ---
    print("Across recordings")
    ok = True

    # The raw offset is deliberately *not* compared across sessions. It's the
    # intercept at each session's own start time, and with a non-zero skew two
    # sessions hours apart legitimately have very different intercepts. What has
    # to be reproducible is the model's parameters, not that one number.
    if len(skews) >= 2:
        spread = float(np.max(skews) - np.min(skews))
        implied_us = spread * SEGMENT_SECONDS  # ppm × s = µs
        print(f"  clock skew spread    {spread:8.2f} ppm  over {len(skews)} sessions "
              f"(mean {np.mean(skews):.1f})")
        print(f"  implied timing error {implied_us / 1000:8.2f} ms  over a "
              f"{SEGMENT_SECONDS:.0f} s segment")
        if implied_us > TIMING_ERROR_BUDGET_US:
            print(f"  !! exceeds the {TIMING_ERROR_BUDGET_US / 1000:.0f} ms budget")
            ok = False
    else:
        print("  !! need at least 2 sessions with resolvable skew")
        ok = False

    if len(latencies) >= 2:
        spread = float(np.max(latencies) - np.min(latencies))
        print(f"  latency floor spread {spread / 1000:8.2f} ms  over {len(latencies)} sessions")
        if spread > LATENCY_SPREAD_LIMIT_US:
            print(f"  !! exceeds {LATENCY_SPREAD_LIMIT_US / 1000:.0f} ms — transport is inconsistent")
            ok = False

    if len(lags) >= 3:
        spread = float(np.max(lags) - np.min(lags))
        print(f"  lidar lag spread     {spread / 1000:8.1f} ms  "
              f"over {len(lags)} usable sessions  (mean {np.mean(lags) / 1000:.1f} ms)")
        if spread > LAG_SPREAD_LIMIT_US:
            print(f"  !! exceeds {LAG_SPREAD_LIMIT_US / 1000:.0f} ms")
            ok = False
    else:
        print(f"  !! only {len(lags)} usable rotation recordings, need 3")
        print("     capture with deliberate yaw sweeps — see phase2_time_sync/README.md")
        ok = False

    print("\nPHASE 2 PASS" if ok else "\nPHASE 2 FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
