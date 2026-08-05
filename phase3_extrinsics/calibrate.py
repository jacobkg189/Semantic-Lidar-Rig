#!/usr/bin/env python3
"""Solve the lidar↔camera extrinsic by minimising point-cloud smear.

    python3 phase3_extrinsics/calibrate.py recordings/<session>
    python3 phase3_extrinsics/calibrate.py --all          # every usable session

Writes calibration/extrinsic.json.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
from scipy.optimize import minimize

_ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [
    str(_ROOT / "shared"),
    str(_ROOT / "phase1_record_replay"),
    str(_ROOT / "phase2_time_sync"),
    str(_ROOT / "phase3_extrinsics"),
]

from extrinsics import (  # noqa: E402
    Extrinsic,
    mask_operator,
    Trajectory,
    build_revolutions,
    occupied_voxels,
    topdown_occupancy,
    wall_thickness,
    to_world,
)
from protocol import MsgType, decode  # noqa: E402
from session import SessionReader, decode_lidar  # noqa: E402
from sync import fit_clock  # noqa: E402

# Initial guess from the physical mounting rather than identity. ARKit's camera
# is +X right, +Y up, -Z forward; the C1's scan plane is its XY with +Z the spin
# axis pointing up. So lidar X -> camera -Z and lidar Z -> camera +Y, which is
# this rotation vector. Starting from identity puts the scan plane vertical —
# 90 degrees out — and the optimiser has no gradient to climb back from.
INIT_RVEC = np.array([-1.2092, 1.2092, 1.2092])
INIT_TVEC = np.array([0.0, 0.10, 0.0])   # lidar sits ~10 cm above the camera

# Sanity bounds. Beyond these the answer is not a plausible physical mounting,
# and the optimiser is exploiting the cost rather than fitting the rig.
MAX_TRANS_M = 0.40
MAX_DT_S = 0.10


def load_session(path: Path, yaw_sign: int):
    r = SessionReader(path)
    dev, arr, pos, quat = [], [], [], []
    for rec in r.poses():
        p = decode(MsgType.POSE, rec.payload)
        dev.append(p.t_device_us)
        arr.append(rec.t_arrival_us)
        pos.append(p.position)
        quat.append(p.quaternion)
    if len(dev) < 100:
        return None

    dev = np.asarray(dev, float)
    clock = fit_clock(dev, np.asarray(arr, float))
    traj = Trajectory(clock.to_mac(dev), np.asarray(pos, float), np.asarray(quat, float))

    rev_t, rev_s = [], []
    for rec in r.lidar():
        _i, t, s = decode_lidar(rec.payload)
        rev_t.append(t)
        rev_s.append(s)
    revs = build_revolutions(np.asarray(rev_t, float), rev_s, yaw_sign=yaw_sign)
    # The operator turns with the rig, so their returns smear regardless of
    # calibration and would dominate any compactness cost.
    revs, _blocked = mask_operator(revs)
    return (r, traj, revs) if len(revs) >= 100 else None


def make_cost(sessions, lag_us: float, stride: int):
    """Total occupied top-down cells across all sessions, for one parameter vector."""
    subsets = [(traj, revs[::stride]) for _, traj, revs in sessions]

    def cost(v: np.ndarray) -> float:
        ext = Extrinsic.from_vector(v)
        if np.linalg.norm(ext.tvec) > MAX_TRANS_M or abs(ext.dt_s) > MAX_DT_S:
            return 1e12  # outside plausible mounting geometry
        total = 0
        for traj, revs in subsets:
            pts = to_world(revs, traj, ext, lag_us)
            if len(pts) < 1000:
                return 1e12
            total += topdown_occupancy(pts)
        return float(total)

    return cost


def report(label: str, sessions, ext: Extrinsic, lag_us: float) -> dict:
    print(f"  {label}")
    print(f"    {ext.describe()}")
    rms_all, vox_all, n_all = [], 0, 0
    for name, traj, revs in sessions:
        pts = to_world(revs, traj, ext, lag_us)
        rms, cnt = wall_thickness(pts)
        rms_all.append(rms)
        vox_all += occupied_voxels(pts)
        n_all += len(pts)
        print(f"    {Path(name).name:24s} plane {rms * 100:5.2f} cm over "
              f"{100 * cnt / max(len(pts), 1):3.0f}% of {len(pts)} pts")
    mean_rms = float(np.nanmean(rms_all))
    print(f"    {'combined':24s} voxels {vox_all}, mean plane {mean_rms * 100:.2f} cm")
    return {"mean_plane_rms_m": mean_rms, "voxels": vox_all, "points": n_all}


def main() -> int:
    ap = argparse.ArgumentParser(description="Phase 3 extrinsic calibration")
    ap.add_argument("sessions", nargs="*")
    ap.add_argument("--all", action="store_true", help="use every usable session")
    ap.add_argument("--root", default="recordings")
    ap.add_argument("--stride", type=int, default=3,
                    help="use every Nth revolution while optimising (speed)")
    ap.add_argument("--out", default="calibration/extrinsic.json")
    args = ap.parse_args()

    cal = json.loads(Path("calibration/timing.json").read_text())
    lag_us = float(cal["lidar_lag_us"])
    yaw_sign = int(cal["lidar_yaw_sign"])

    paths = [Path(p) for p in args.sessions]
    if args.all or not paths:
        paths = sorted(p for p in Path(args.root).glob("*") if (p / "manifest.json").exists())

    sessions = []
    for p in paths:
        try:
            loaded = load_session(p, yaw_sign)
        except Exception as e:
            print(f"  skip {p.name}: {e}")
            continue
        if loaded:
            r, traj, revs = loaded
            sessions.append((str(p), traj, revs))

    if not sessions:
        print("No usable sessions.")
        return 1

    print(f"Sessions  {len(sessions)}")
    print(f"Timing    lag {lag_us / 1000:+.1f} ms (provisional), yaw sign {yaw_sign:+d}\n")

    x0 = Extrinsic(INIT_RVEC.copy(), INIT_TVEC.copy(), 0.0).to_vector()
    print("Before")
    before = report("mounting estimate", sessions, Extrinsic.from_vector(x0), lag_us)

    print("\nOptimising (top-down wall sharpness)...")
    cost = make_cost(sessions, lag_us, args.stride)
    t0 = time.monotonic()
    # Gradient-free: the voxel count is piecewise constant, so anything
    # derivative-based sees zero gradient almost everywhere.
    res = minimize(cost, x0, method="Nelder-Mead",
                   options={"maxiter": 3000, "xatol": 1e-4, "fatol": 1.0, "adaptive": True})
    print(f"  {res.nit} iterations, {res.nfev} evaluations, {time.monotonic() - t0:.0f}s")

    ext = Extrinsic.from_vector(res.x)
    print("\nAfter")
    after = report("optimised", sessions, ext, lag_us)

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps({
        "rvec_rad": ext.rvec.tolist(),
        "tvec_m": ext.tvec.tolist(),
        "residual_dt_us": ext.dt_s * 1e6,
        "total_lag_us": lag_us + ext.dt_s * 1e6,
        "yaw_sign": yaw_sign,
        "sessions": [Path(s).name for s, _, _ in sessions],
        "before": before,
        "after": after,
    }, indent=2) + "\n")
    print(f"\nWrote {out}")

    improved = after["mean_plane_rms_m"] < before["mean_plane_rms_m"]
    print("Improved." if improved else "!! No improvement — check the initial guess.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
