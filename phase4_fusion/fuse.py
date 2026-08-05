#!/usr/bin/env python3
"""Phase 4: fuse a recording into a drift-corrected map.

    python3 phase4_fusion/fuse.py recordings/<session> [--render out.png]
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
]

from extrinsics import (  # noqa: E402
    Extrinsic,
    Trajectory,
    build_revolutions,
    mask_operator,
    to_world,
    topdown_occupancy,
    wall_thickness,
)
from fusion import build_edges, build_keyframes, optimise, se2, se2_inv  # noqa: E402
from protocol import MsgType, decode  # noqa: E402
from session import SessionReader, decode_lidar  # noqa: E402
from sync import fit_clock  # noqa: E402


def load(path: Path, cal):
    r = SessionReader(path)
    dev, arr, pos, quat = [], [], [], []
    for rec in r.poses():
        p = decode(MsgType.POSE, rec.payload)
        dev.append(p.t_device_us)
        arr.append(rec.t_arrival_us)
        pos.append(p.position)
        quat.append(p.quaternion)
    dev = np.asarray(dev, float)
    traj = Trajectory(fit_clock(dev, np.asarray(arr, float)).to_mac(dev),
                      np.asarray(pos, float), np.asarray(quat, float))

    rev_t, rev_s = [], []
    for rec in r.lidar():
        _i, t, s = decode_lidar(rec.payload)
        rev_t.append(t)
        rev_s.append(s)
    revs, _ = mask_operator(build_revolutions(np.asarray(rev_t, float), rev_s,
                                              yaw_sign=cal["lidar_yaw_sign"]))
    return r, traj, revs


def corrected_trajectory(traj: Trajectory, kfs, solved: np.ndarray) -> Trajectory:
    """Warp the full pose stream by the per-keyframe SE(2) correction.

    The graph only solves keyframes, but every lidar sample needs a pose. The
    correction is interpolated between keyframes rather than the poses being
    replaced, so ARKit's high-rate local motion — which is good — is preserved
    and only the slow drift is removed.
    """
    kt = np.array([k.t_us for k in kfs])
    corr = []
    for k, sol in zip(kfs, solved):
        C = se2(sol[0], sol[1], sol[2]) @ se2_inv(se2(k.xz[0], k.xz[1], k.yaw))
        corr.append((C[0, 2], C[1, 2], np.arctan2(C[1, 0], C[0, 0])))
    corr = np.asarray(corr)

    t = traj.t
    i = np.clip(np.searchsorted(kt, t) - 1, 0, len(kt) - 2)
    u = np.clip((t - kt[i]) / np.maximum(kt[i + 1] - kt[i], 1e-9), 0, 1)
    cx = corr[i, 0] * (1 - u) + corr[i + 1, 0] * u
    cz = corr[i, 1] * (1 - u) + corr[i + 1, 1] * u
    # Angles interpolate through the shortest arc.
    d = (corr[i + 1, 2] - corr[i, 2] + np.pi) % (2 * np.pi) - np.pi
    ct = corr[i, 2] + u * d

    p = traj.p.copy()
    c, s = np.cos(ct), np.sin(ct)
    p[:, 0] = c * traj.p[:, 0] - s * traj.p[:, 2] + cx
    p[:, 2] = s * traj.p[:, 0] + c * traj.p[:, 2] + cz

    # Compose the yaw correction onto each orientation, about world up.
    h = ct / 2.0
    qy = np.stack([np.zeros_like(h), np.sin(h), np.zeros_like(h), np.cos(h)], -1)
    x1, y1, z1, w1 = qy[:, 0], qy[:, 1], qy[:, 2], qy[:, 3]
    x2, y2, z2, w2 = traj.q[:, 0], traj.q[:, 1], traj.q[:, 2], traj.q[:, 3]
    q = np.stack([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ], -1)
    return Trajectory(traj.t, p, q)


def measure(revs, traj, ext, lag):
    pts = to_world(revs, traj, ext, lag)
    y = pts[:, 1] - np.median(pts[:, 1])
    band = pts[(y > -1.2) & (y < 0.6)]
    w, n = wall_thickness(band)
    return pts, band, w, topdown_occupancy(pts)


def main() -> int:
    ap = argparse.ArgumentParser(description="Phase 4 pose-graph fusion")
    ap.add_argument("session")
    ap.add_argument("--render", default=None)
    args = ap.parse_args()

    cal = json.loads((_ROOT / "calibration/timing.json").read_text())
    exj = json.loads((_ROOT / "calibration/extrinsic.json").read_text())
    ext = Extrinsic(np.array(exj["rvec_rad"]), np.array(exj["tvec_m"]),
                    exj["residual_dt_us"] / 1e6)
    lag = cal["lidar_lag_us"]

    r, traj, revs = load(Path(args.session), cal)
    print(f"Session   {Path(args.session).name}  {r.meta.get('notes')!r}")
    print(f"          {len(revs)} revolutions")

    _, _, w0, v0 = measure(revs, traj, ext, lag)
    print(f"\nBefore    walls {w0 * 100:.2f} cm   cells {v0}")

    kfs = build_keyframes(revs, traj, ext, lag, to_world)
    print(f"\nKeyframes {len(kfs)}")
    edges, n_loop, n_rej = build_edges(kfs)
    print(f"Edges     {len(edges)}  ({n_loop} loop closures, {n_rej} rejected as implausible)")
    if n_loop == 0:
        print("  !! no loop closures found — the path may not revisit anywhere")

    solved = optimise(kfs, edges)
    shift = np.linalg.norm(solved[:, :2] - np.array([k.xz for k in kfs]), axis=1)
    print(f"Correction  median {np.median(shift) * 100:.1f} cm, max {shift.max() * 100:.1f} cm")

    traj2 = corrected_trajectory(traj, kfs, solved)
    _, band1, w1, v1 = measure(revs, traj2, ext, lag)
    print(f"\nAfter     walls {w1 * 100:.2f} cm   cells {v1}")
    print(f"          {100 * (1 - w1 / w0):+.0f}% wall thickness, "
          f"{100 * (1 - v1 / v0):+.0f}% cells")

    if args.render:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        _, band0, _, _ = measure(revs, traj, ext, lag)
        fig, ax = plt.subplots(1, 2, figsize=(15, 7))
        for a, b, lab, w in ((ax[0], band0, "ARKit only", w0), (ax[1], band1, "pose-graph fused", w1)):
            H, xe, ye = np.histogram2d(b[:, 0], b[:, 2], bins=550)
            a.imshow(np.log1p(H.T), origin="lower", extent=[xe[0], xe[-1], ye[0], ye[-1]],
                     cmap="inferno", aspect="equal")
            a.set_title(f"{lab} — walls {w * 100:.1f} cm")
            a.set_xlabel("x (m)")
        ax[0].set_ylabel("z (m)")
        plt.tight_layout()
        plt.savefig(args.render, dpi=120)
        print(f"\nWrote {args.render}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
