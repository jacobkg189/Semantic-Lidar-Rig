#!/usr/bin/env python3
"""Phase 6: measure the map, and compare against tape-measured ground truth.

    python3 phase6_validation/validate.py recordings/<session>
    python3 phase6_validation/validate.py recordings/<session> --truth truth.json

`truth.json` is a flat mapping of measurement name to metres, e.g.

    {"wall span A": 3.42, "wall span B": 4.18, "floor to ceiling": 2.44}

Without it the tool just reports what the map thinks, which is the list to go
and measure.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

_ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [
    str(_ROOT / "shared"),
    str(_ROOT / "phase1_record_replay"),
    str(_ROOT / "phase2_time_sync"),
    str(_ROOT / "phase3_extrinsics"),
    str(_ROOT / "phase4_fusion"),
    str(_ROOT / "phase5_semantics"),
    str(_ROOT / "phase6_validation"),
]

from depth import depth_to_world, load_depth_frames  # noqa: E402
from extrinsics import Extrinsic, to_world  # noqa: E402
from fuse import corrected_trajectory, load  # noqa: E402
from fusion import build_edges, build_keyframes, optimise  # noqa: E402
from measure import compare, measure_room  # noqa: E402
from protocol import MsgType, decode  # noqa: E402
from semantics import label_points, load_mesh  # noqa: E402
from session import SessionReader  # noqa: E402
from sync import fit_clock  # noqa: E402


def build_clouds(session: Path):
    cal = json.loads((_ROOT / "calibration/timing.json").read_text())
    exj = json.loads((_ROOT / "calibration/extrinsic.json").read_text())
    ext = Extrinsic(np.array(exj["rvec_rad"]), np.array(exj["tvec_m"]),
                    exj["residual_dt_us"] / 1e6)

    r, traj, revs = load(session, cal)
    reader = SessionReader(session)
    dev, arr = [], []
    for rec in reader.poses():
        p = decode(MsgType.POSE, rec.payload)
        dev.append(p.t_device_us)
        arr.append(rec.t_arrival_us)
    clock = fit_clock(np.asarray(dev, float), np.asarray(arr, float))

    kfs = build_keyframes(revs, traj, ext, cal["lidar_lag_us"], to_world)
    edges, n_loop, _ = build_edges(kfs)
    traj2 = corrected_trajectory(traj, kfs, optimise(kfs, edges))

    c1 = to_world(revs, traj2, ext, cal["lidar_lag_us"])
    frames = load_depth_frames(reader)
    dp = depth_to_world(frames, traj2, clock.to_mac) if frames else np.empty((0, 3))

    mesh = None
    try:
        mesh = load_mesh(reader)
    except Exception:
        pass
    lc1 = label_points(c1, mesh) if mesh is not None else None
    ldp = label_points(dp, mesh) if (mesh is not None and len(dp)) else None
    return r, c1, lc1, dp, ldp, len(kfs), n_loop


def main() -> int:
    ap = argparse.ArgumentParser(description="Phase 6 validation")
    ap.add_argument("session")
    ap.add_argument("--truth", help="JSON of tape-measured values, in metres")
    args = ap.parse_args()

    r, c1, lc1, dp, ldp, nkf, nloop = build_clouds(Path(args.session))
    print(f"Session   {Path(args.session).name}  {r.meta.get('notes')!r}")
    print(f"          pose graph {nkf} keyframes, {nloop} loop closures")
    print(f"          C1 {len(c1)} pts, depth {len(dp)} pts\n")

    sources = [("iPhone depth", dp, ldp)] if len(dp) else []
    sources.append(("RPLidar C1", c1, lc1))

    all_pred = {}
    for name, pts, lab in sources:
        if len(pts) < 1000:
            continue
        print(f"{name}")
        preds = measure_room(pts, lab)
        for m in preds:
            unit = "" if "yaw" in m.name else " m"
            print(f"  {m.name:<32s} {m.value_m:8.3f}{unit}   {m.detail}")
        all_pred[name] = preds
        print()

    # Cross-check: the two sensors should agree with each other even before any
    # tape is involved. Disagreement here means the problem is internal.
    if len(all_pred) == 2:
        (n1, p1), (n2, p2) = all_pred.items()
        d1 = {m.name: m.value_m for m in p1 if "yaw" not in m.name}
        d2 = {m.name: m.value_m for m in p2 if "yaw" not in m.name}
        shared = [k for k in d1 if k in d2]
        if shared:
            print(f"Sensor agreement ({n1} vs {n2})")
            for k in shared:
                print(f"  {k:<32s} {d1[k]:7.3f} vs {d2[k]:7.3f}   "
                      f"diff {abs(d1[k]-d2[k])*100:5.1f} cm")
            print()

    if not args.truth:
        print("No ground truth supplied. Measure these physically, then:")
        print("  python3 phase6_validation/validate.py <session> --truth truth.json")
        return 0

    truth = json.loads(Path(args.truth).read_text())
    print("Against tape measure")
    ok = True
    for name, preds in all_pred.items():
        rows = compare(preds, truth)
        if not rows:
            continue
        print(f"  {name}")
        errs = []
        for nm, got, exp, err, pct in rows:
            flag = "" if abs(err) < 0.03 else "  !!"
            print(f"    {nm:<30s} map {got:6.3f}  tape {exp:6.3f}  "
                  f"err {err*100:+6.1f} cm ({pct:+5.2f}%){flag}")
            errs.append(abs(err))
        if errs:
            print(f"    {'':<30s} mean |err| {np.mean(errs)*100:.1f} cm, "
                  f"max {np.max(errs)*100:.1f} cm")
            if np.max(errs) > 0.03:
                ok = False
        print()

    print("PHASE 6 PASS" if ok else "PHASE 6 FAIL — errors exceed 3 cm")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
