"""Pose-graph fusion: remove ARKit's drift using lidar scan matching.

Phase 3 established that the residual smear is drift, not calibration — the
extrinsic cross-validates to 1.3 cm across recordings, yet wall thickness grows
with window length (4.6 cm at 5 s, 7.4 cm at 40 s) and differs between walks on
identical parameters. That is exactly what a pose graph with loop closure fixes.

**Why the graph is 2D.** ARKit is gravity-aligned via the IMU, so roll and pitch
are absolutely referenced and do not drift — only yaw and position do. Solving
all 6 DoF would spend effort on three that are already correct and are far
better constrained by the IMU than by a planar lidar. So roll, pitch and height
are taken from ARKit and the graph corrects (x, z, yaw).

That also matches the sensor: the C1 measures a horizontal slice, and has almost
nothing to say about the vertical.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.spatial import cKDTree

# Keyframe spacing. Close enough that consecutive scans overlap heavily, far
# enough apart that the graph stays small and each edge carries real geometry.
KEYFRAME_M = 0.25
KEYFRAME_S = 1.0

# Slice of the cloud used for 2D matching, relative to the sensor height. The rig
# tilts deliberately, so returns that hit floor or ceiling project into the
# horizontal plane as garbage and have to be excluded.
HEIGHT_BAND = (-0.6, 0.6)

# Loop closure search: pairs closer than this in space but further apart in time.
LOOP_RADIUS_M = 1.5
LOOP_MIN_GAP_S = 6.0

# A loop closure may only *refine* the prior, never overrule it. ARKit drifts
# slowly; over a 90 s indoor walk the accumulated error is centimetres, not half
# a metre. Any match claiming a bigger correction than this has found a wrong
# alignment, not a real revisit.
#
# This matters especially in a near-square room, where four similar walls give
# the scan roughly 4-fold rotational symmetry: ICP can snap to a 90 deg-rotated
# pose that scores *better* than the truth. Without this gate 2133 of 2232 edges
# were accepted as loop closures and the map got 4x worse.
MAX_LOOP_TRANS_M = 0.30
MAX_LOOP_YAW_DEG = 12.0

# Loop edges get down-weighted relative to sequential ones: consecutive scans
# overlap almost completely, revisits only partially.
LOOP_WEIGHT_SCALE = 0.3

# Sequential (ARKit) edges are trusted heavily; loop closures only nudge.
SEQ_WEIGHT = 50.0

# Huber threshold on edge residuals during optimisation, so a surviving bad edge
# is bounded rather than dominating a least-squares fit.
HUBER_M = 0.05


@dataclass
class Keyframe:
    idx: int
    t_us: float
    xz: np.ndarray        # ARKit position, horizontal (x, z)
    yaw: float            # ARKit yaw about world up
    pts: np.ndarray       # Nx2 scan in the keyframe's own frame
    height: float = 0.0


@dataclass
class Edge:
    i: int
    j: int
    dx: float
    dz: float
    dyaw: float
    weight: float = 1.0
    loop: bool = False


def yaw_from_matrix(R: np.ndarray) -> np.ndarray:
    """Heading about world up (+Y), from camera-to-world rotations.

    Uses the camera's forward axis projected onto the horizontal plane. Reading
    a Y-Euler angle directly would gimbal-lock as the rig pitches, which it does
    deliberately during capture.
    """
    fwd = -R[..., :, 2]          # ARKit camera looks along -Z
    return np.arctan2(fwd[..., 0], fwd[..., 2])


def se2(dx: float, dz: float, dyaw: float) -> np.ndarray:
    c, s = np.cos(dyaw), np.sin(dyaw)
    return np.array([[c, -s, dx], [s, c, dz], [0, 0, 1]])


def se2_inv(T: np.ndarray) -> np.ndarray:
    R = T[:2, :2]
    out = np.eye(3)
    out[:2, :2] = R.T
    out[:2, 2] = -R.T @ T[:2, 2]
    return out


def build_keyframes(revs, traj, ext, lag_us: float, to_world_fn) -> list[Keyframe]:
    """Group revolutions into keyframes, each carrying a local 2D scan."""
    from extrinsics import quat_to_matrix  # local import: shared module

    kfs: list[Keyframe] = []
    last_xz, last_t = None, None
    group: list = []

    for rev in revs:
        pos, rot = traj.at(np.array([rev.t_end_us]))
        xz = np.array([pos[0, 0], pos[0, 2]])
        moved = last_xz is None or np.linalg.norm(xz - last_xz) >= KEYFRAME_M
        elapsed = last_t is None or (rev.t_end_us - last_t) >= KEYFRAME_S * 1e6
        group.append(rev)

        if moved or elapsed:
            pts = to_world_fn(group, traj, ext, lag_us)
            if len(pts) >= 100:
                y = pts[:, 1] - pos[0, 1]
                band = (y > HEIGHT_BAND[0]) & (y < HEIGHT_BAND[1])
                p = pts[band][:, [0, 2]]
                if len(p) >= 60:
                    yaw = float(yaw_from_matrix(rot[0]))
                    c, s = np.cos(-yaw), np.sin(-yaw)
                    local = (p - xz) @ np.array([[c, -s], [s, c]]).T
                    kfs.append(Keyframe(len(kfs), rev.t_end_us, xz, yaw, local,
                                        height=float(pos[0, 1])))
            group = []
            last_xz, last_t = xz, rev.t_end_us

    return kfs


def icp2d(src: np.ndarray, dst_tree: cKDTree, init: np.ndarray,
          iters: int = 30, max_corr: float = 0.5) -> tuple[np.ndarray, float]:
    """Point-to-point ICP in SE(2). Returns (transform, mean inlier residual).

    The ARKit relative pose supplies the initial guess, so this only has to
    remove the drift rather than solve from scratch.
    """
    T = init.copy()
    resid = np.inf
    for _ in range(iters):
        p = (T[:2, :2] @ src.T).T + T[:2, 2]
        d, idx = dst_tree.query(p, k=1, distance_upper_bound=max_corr)
        good = np.isfinite(d)
        if good.sum() < 30:
            return T, np.inf
        a = p[good]
        b = dst_tree.data[idx[good]]

        # Umeyama / Kabsch in 2D.
        ca, cb = a.mean(0), b.mean(0)
        H = (a - ca).T @ (b - cb)
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[1] *= -1
            R = Vt.T @ U.T
        step = np.eye(3)
        step[:2, :2] = R
        step[:2, 2] = cb - R @ ca
        T = step @ T
        new_resid = float(np.mean(d[good]))
        if abs(resid - new_resid) < 1e-5:
            resid = new_resid
            break
        resid = new_resid
    return T, resid


def relative_arkit(a: Keyframe, b: Keyframe) -> np.ndarray:
    return se2_inv(se2(a.xz[0], a.xz[1], a.yaw)) @ se2(b.xz[0], b.xz[1], b.yaw)


def build_edges(kfs: list[Keyframe], loop_radius: float = LOOP_RADIUS_M,
                loop_gap_s: float = LOOP_MIN_GAP_S,
                accept_resid: float = 0.05,
                max_per_keyframe: int = 4) -> tuple[list[Edge], int, int]:
    """Sequential constraints plus loop closures, both from scan matching."""
    trees = [cKDTree(k.pts) for k in kfs]
    edges: list[Edge] = []

    # Sequential constraints come from ARKit, NOT from scan matching. Over a
    # 0.25 m / 1 s keyframe gap, VIO is accurate to millimetres, whereas 2D ICP
    # on a sparse planar slice disagrees with it by a median 2.3 cm / 1.0 deg and
    # 11.8 cm / 4.5 deg at p90. Substituting ICP here replaced good odometry with
    # noise and chained 100 such errors together, making the map 3x worse.
    #
    # Lidar's job is loop closure — the one thing VIO cannot do — not odometry.
    for i in range(len(kfs) - 1):
        T = relative_arkit(kfs[i], kfs[i + 1])
        edges.append(Edge(i, i + 1, T[0, 2], T[1, 2],
                          float(np.arctan2(T[1, 0], T[0, 0])),
                          weight=SEQ_WEIGHT))

    # Loop closures: revisited places, matched against a far-earlier keyframe.
    xz = np.array([k.xz for k in kfs])
    t = np.array([k.t_us for k in kfs])
    tree_xy = cKDTree(xz)
    pairs = tree_xy.query_pairs(loop_radius, output_type="ndarray")
    n_loop, n_rejected = 0, 0
    per_kf: dict[int, int] = {}
    # Best-scoring candidates first, so the per-keyframe cap keeps the strongest.
    cands = []
    for i, j in pairs:
        if abs(t[j] - t[i]) < loop_gap_s * 1e6:
            continue
        prior = relative_arkit(kfs[i], kfs[j])
        T, r = icp2d(kfs[j].pts, trees[i], prior)
        if not np.isfinite(r) or r >= accept_resid:
            continue
        # How far did ICP move from the ARKit prior?
        d = se2_inv(prior) @ T
        dtrans = float(np.hypot(d[0, 2], d[1, 2]))
        dyaw = abs(float(np.arctan2(d[1, 0], d[0, 0])))
        if dtrans > MAX_LOOP_TRANS_M or dyaw > np.radians(MAX_LOOP_YAW_DEG):
            n_rejected += 1
            continue
        cands.append((r, int(i), int(j), T))

    for r, i, j, T in sorted(cands):
        if per_kf.get(i, 0) >= max_per_keyframe or per_kf.get(j, 0) >= max_per_keyframe:
            continue
        per_kf[i] = per_kf.get(i, 0) + 1
        per_kf[j] = per_kf.get(j, 0) + 1
        edges.append(Edge(i, j, T[0, 2], T[1, 2],
                          float(np.arctan2(T[1, 0], T[0, 0])),
                          weight=LOOP_WEIGHT_SCALE / max(r, 0.01), loop=True))
        n_loop += 1
    return edges, n_loop, n_rejected


def optimise(kfs: list[Keyframe], edges: list[Edge], iters: int = 40):
    """Gauss-Newton on SE(2) poses. Returns corrected (x, z, yaw) per keyframe."""
    n = len(kfs)
    x = np.zeros(3 * n)
    for k in kfs:
        x[3 * k.idx:3 * k.idx + 3] = [k.xz[0], k.xz[1], k.yaw]

    def wrap(a):
        return (a + np.pi) % (2 * np.pi) - np.pi

    for _ in range(iters):
        H = np.zeros((3 * n, 3 * n))
        b = np.zeros(3 * n)

        for e in edges:
            xi, yi, ti = x[3 * e.i:3 * e.i + 3]
            xj, yj, tj = x[3 * e.j:3 * e.j + 3]
            c, s = np.cos(ti), np.sin(ti)
            # Predicted relative pose, minus the measured one.
            dx = c * (xj - xi) + s * (yj - yi) - e.dx
            dy = -s * (xj - xi) + c * (yj - yi) - e.dz
            dt = wrap(tj - ti - e.dyaw)
            r = np.array([dx, dy, dt])

            # Huber: bound the influence of any edge that is still an outlier.
            mag = float(np.hypot(dx, dy))
            robust = 1.0 if mag <= HUBER_M else HUBER_M / mag

            Ai = np.array([[-c, -s, -s * (xj - xi) + c * (yj - yi)],
                           [s, -c, -c * (xj - xi) - s * (yj - yi)],
                           [0, 0, -1.0]])
            Aj = np.array([[c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]])
            w = e.weight * robust
            I, J = slice(3 * e.i, 3 * e.i + 3), slice(3 * e.j, 3 * e.j + 3)
            H[I, I] += w * Ai.T @ Ai
            H[J, J] += w * Aj.T @ Aj
            H[I, J] += w * Ai.T @ Aj
            H[J, I] += w * Aj.T @ Ai
            b[I] += w * Ai.T @ r
            b[J] += w * Aj.T @ r

        # Gauge freedom: the whole graph can translate and rotate freely, so
        # anchor the first pose or H is singular.
        H[:3, :3] += np.eye(3) * 1e6
        H += np.eye(3 * n) * 1e-6

        try:
            dx_vec = np.linalg.solve(H, -b)
        except np.linalg.LinAlgError:
            break
        x += dx_vec
        if np.max(np.abs(dx_vec)) < 1e-6:
            break

    return x.reshape(n, 3)
