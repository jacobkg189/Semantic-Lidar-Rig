"""Lidar↔camera extrinsic calibration by self-consistency.

If the rigid transform between the C1 and the ARKit camera is right, then a wall
scanned from many viewpoints accumulates into a *thin* surface in the world
frame. If it's wrong, the same wall smears into a slab. That thickness is a
direct measure of calibration quality and needs no second sensor to trust — the
alternative, matching against ARKit's reconstruction mesh, would require sending
scene geometry from the phone, which the app doesn't do yet.

Seven parameters are solved together:

    rx, ry, rz    lidar orientation in the camera frame (rotation vector, rad)
    tx, ty, tz    lidar position in the camera frame (m)
    dt            residual time offset on top of the Phase 2 estimate (s)

`dt` is here deliberately. Phase 2's isolated lag estimate hit its noise floor at
+/-6 ms; solving it jointly gives it every lidar return as a constraint instead
of one yaw estimate per revolution.

Observability note: the C1 is planar. Held level it only ever sees vertical
walls, whose normals are all horizontal — which leaves translation along the
spin axis unobservable. The calibration capture must include deliberate tilt so
the scan plane cuts the floor and ceiling. That is the exact opposite of what
Phase 2 wanted, where tilt was noise.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

# Voxel edge for the compactness cost. Small enough to resolve a well-calibrated
# wall, large enough that sensor noise alone doesn't dominate the count.
VOXEL_M = 0.02

# Points nearer than this are the rig itself or the operator's hands.
MIN_RANGE_M = 0.20
MAX_RANGE_M = 12.0


@dataclass
class Extrinsic:
    """Lidar pose in the ARKit camera frame, plus a residual time offset."""

    rvec: np.ndarray          # rotation vector (rad), lidar -> camera
    tvec: np.ndarray          # translation (m), lidar origin in camera frame
    dt_s: float = 0.0         # residual time offset on top of Phase 2's lag

    @staticmethod
    def identity() -> "Extrinsic":
        return Extrinsic(np.zeros(3), np.zeros(3), 0.0)

    @staticmethod
    def from_vector(v: np.ndarray) -> "Extrinsic":
        return Extrinsic(np.asarray(v[0:3], float), np.asarray(v[3:6], float), float(v[6]))

    def to_vector(self) -> np.ndarray:
        return np.concatenate([self.rvec, self.tvec, [self.dt_s]])

    def matrix(self) -> np.ndarray:
        m = np.eye(4)
        m[:3, :3] = rodrigues(self.rvec)
        m[:3, 3] = self.tvec
        return m

    def describe(self) -> str:
        deg = np.degrees(self.rvec)
        return (f"rot [{deg[0]:+7.2f} {deg[1]:+7.2f} {deg[2]:+7.2f}] deg   "
                f"trans [{self.tvec[0]*100:+6.1f} {self.tvec[1]*100:+6.1f} "
                f"{self.tvec[2]*100:+6.1f}] cm   dt {self.dt_s*1000:+6.1f} ms")


def rodrigues(r: np.ndarray) -> np.ndarray:
    """Rotation vector -> matrix."""
    theta = float(np.linalg.norm(r))
    if theta < 1e-12:
        return np.eye(3)
    k = r / theta
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * (K @ K)


def quat_to_matrix(q: np.ndarray) -> np.ndarray:
    """(x, y, z, w) -> 3x3, vectorised over leading axis."""
    q = q / np.linalg.norm(q, axis=-1, keepdims=True)
    x, y, z, w = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    return np.stack([
        np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], -1),
        np.stack([2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], -1),
        np.stack([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], -1),
    ], -2)


def slerp(q0: np.ndarray, q1: np.ndarray, u: np.ndarray) -> np.ndarray:
    """Shortest-arc quaternion interpolation, vectorised."""
    q0 = q0 / np.linalg.norm(q0, axis=-1, keepdims=True)
    q1 = q1 / np.linalg.norm(q1, axis=-1, keepdims=True)
    dot = np.sum(q0 * q1, axis=-1, keepdims=True)
    # Flip one end where the dot product is negative, else interpolation takes
    # the long way round and the rig appears to spin backwards mid-revolution.
    q1 = np.where(dot < 0, -q1, q1)
    dot = np.abs(dot).clip(-1.0, 1.0)

    theta = np.arccos(dot)
    small = theta < 1e-6
    sin_theta = np.sin(theta)
    w0 = np.where(small, 1.0 - u, np.sin((1.0 - u) * theta) / np.where(small, 1.0, sin_theta))
    w1 = np.where(small, u, np.sin(u * theta) / np.where(small, 1.0, sin_theta))
    out = w0 * q0 + w1 * q1
    return out / np.linalg.norm(out, axis=-1, keepdims=True)


class Trajectory:
    """Pose lookup at arbitrary times, from the ARKit stream."""

    def __init__(self, t_us: np.ndarray, positions: np.ndarray, quats: np.ndarray):
        order = np.argsort(t_us)
        self.t = np.asarray(t_us, float)[order]
        self.p = np.asarray(positions, float)[order]
        self.q = np.asarray(quats, float)[order]

    @property
    def span_us(self) -> tuple[float, float]:
        return float(self.t[0]), float(self.t[-1])

    def at(self, t_query_us: np.ndarray):
        """Interpolated (position, rotation matrix) at each query time."""
        t = np.asarray(t_query_us, float)
        i = np.clip(np.searchsorted(self.t, t) - 1, 0, len(self.t) - 2)
        t0, t1 = self.t[i], self.t[i + 1]
        u = np.clip((t - t0) / np.maximum(t1 - t0, 1e-9), 0.0, 1.0)[:, None]
        pos = self.p[i] * (1 - u) + self.p[i + 1] * u
        rot = quat_to_matrix(slerp(self.q[i], self.q[i + 1], u))
        return pos, rot


@dataclass
class Revolution:
    """One C1 sweep, with a per-sample timestamp."""

    t_end_us: float
    angles_deg: np.ndarray
    ranges_m: np.ndarray
    t_sample_us: np.ndarray


def build_revolutions(rev_t_us: np.ndarray, rev_samples: list,
                      yaw_sign: int = -1) -> list[Revolution]:
    """Attach a timestamp to every individual return.

    The C1 stamps arrive once per revolution, but the sweep takes ~100 ms and the
    rig moves during it. Samples are emitted in order at a near-constant rate, so
    a sample's time is its position through the revolution — this is the
    deskewing that Phase 4 depends on, and it matters here too because a smeared
    revolution looks exactly like a bad extrinsic.

    `yaw_sign` handles the C1's angle convention relative to ARKit. It is a
    reflection, not a rotation, so it cannot be absorbed into the extrinsic and
    has to be applied when the point is constructed.
    """
    out: list[Revolution] = []
    for i in range(1, len(rev_t_us)):
        samples = rev_samples[i]
        if not samples:
            continue
        arr = np.asarray(samples, float)
        ang, rng, _q = arr[:, 0], arr[:, 1] / 1000.0, arr[:, 2]

        keep = (rng >= MIN_RANGE_M) & (rng <= MAX_RANGE_M)
        if keep.sum() < 32:
            continue
        ang, rng = ang[keep], rng[keep]

        period = rev_t_us[i] - rev_t_us[i - 1]
        if not (20_000 < period < 500_000):  # implausible; dropped revolution
            continue
        frac = np.linspace(0.0, 1.0, len(samples))[keep]
        out.append(Revolution(
            t_end_us=float(rev_t_us[i]),
            angles_deg=yaw_sign * ang,
            ranges_m=rng,
            t_sample_us=rev_t_us[i] - period * (1.0 - frac),
        ))
    return out


def mask_operator(revs: list[Revolution], nbins: int = 72, near_m: float = 1.0,
                  persist: float = 0.45) -> tuple[list[Revolution], np.ndarray]:
    """Drop the angular sector occupied by the person holding the rig.

    The operator stands behind the rig and turns with it, so they sit at a fixed
    bearing in the *lidar* frame while being at close range. In the world frame
    those returns smear everywhere, and they are not few: measured at 41% and 29%
    of all returns in two recordings. That is enough to dominate any compactness
    cost and make the extrinsic nearly unobservable.

    Detection is per-recording because the sector moves with how the rig is held
    (0-105 deg in one capture, 20-70 deg in another), so a hardcoded mask would
    be wrong as often as right.
    """
    close = np.zeros(nbins)
    total = np.zeros(nbins)
    for rev in revs:
        b = (np.floor((rev.angles_deg % 360) / 360.0 * nbins)).astype(int) % nbins
        np.add.at(total, b, 1)
        np.add.at(close, b[rev.ranges_m < near_m], 1)

    frac = np.divide(close, total, out=np.zeros(nbins), where=total > 0)
    blocked = frac >= persist

    out = []
    for rev in revs:
        b = (np.floor((rev.angles_deg % 360) / 360.0 * nbins)).astype(int) % nbins
        keep = ~blocked[b]
        if keep.sum() < 32:
            continue
        out.append(Revolution(
            t_end_us=rev.t_end_us,
            angles_deg=rev.angles_deg[keep],
            ranges_m=rev.ranges_m[keep],
            t_sample_us=rev.t_sample_us[keep],
        ))
    return out, blocked


def to_world(revs: list[Revolution], traj: Trajectory, ext: Extrinsic,
             lag_us: float) -> np.ndarray:
    """Project every lidar return into the ARKit world frame."""
    R_cl = rodrigues(ext.rvec)
    t_cl = ext.tvec
    shift = lag_us + ext.dt_s * 1e6

    chunks = []
    lo, hi = traj.span_us
    for rev in revs:
        t = rev.t_sample_us - shift
        inside = (t >= lo) & (t <= hi)
        if inside.sum() < 32:
            continue
        t = t[inside]
        a = np.radians(rev.angles_deg[inside])
        r = rev.ranges_m[inside]

        # Planar scanner: every return lies in the lidar's z = 0 plane.
        p_lidar = np.stack([r * np.cos(a), r * np.sin(a), np.zeros_like(r)], -1)
        p_cam = p_lidar @ R_cl.T + t_cl

        pos, rot = traj.at(t)
        chunks.append(np.einsum("nij,nj->ni", rot, p_cam) + pos)

    return np.concatenate(chunks) if chunks else np.empty((0, 3))


def occupied_voxels(points: np.ndarray, voxel_m: float = VOXEL_M) -> int:
    """Compactness cost: a correct extrinsic packs a wall into fewer voxels.

    Deliberately not a plane fit — plane extraction depends on the calibration
    being roughly right already, which is the thing being solved for.
    """
    if len(points) == 0:
        return 0
    keys = np.floor(points / voxel_m).astype(np.int64)
    return int(len(np.unique(keys, axis=0)))


def topdown_occupancy(points: np.ndarray, cell_m: float = 0.02,
                      height_band: tuple[float, float] = (-1.2, 0.6)) -> int:
    """Occupied cells in a top-down projection — the calibration cost.

    Chosen over 3D voxel count after that turned out to be misaligned with the
    goal: it improved while wall thickness got worse, because it also rewards
    compacting floor, ceiling and furniture. Projecting to 2D scores exactly the
    wall sharpness that is visible in a top-down render, and is blind to the
    vertical spread that deliberate tilting introduces on purpose.

    The height band is relative to the trajectory's mean height and drops
    floor/ceiling returns, which carry no wall information.
    """
    if len(points) == 0:
        return 0
    y = points[:, 1] - np.median(points[:, 1])
    keep = (y > height_band[0]) & (y < height_band[1])
    p = points[keep][:, [0, 2]]
    if len(p) == 0:
        return 0
    return int(len(np.unique(np.floor(p / cell_m).astype(np.int64), axis=0)))


def wall_thickness(points: np.ndarray, iters: int = 200, inlier_m: float = 0.25,
                   min_frac: float = 0.03, seed: int = 0) -> tuple[float, int]:
    """RANSAC the dominant *vertical* plane; return (rms residual, inlier count).

    Restricted to walls on purpose. A planar lidar held roughly level puts every
    return into a thin horizontal slab, so an unconstrained plane fit locks onto
    that slab — it captured 77-90% of all points and reported how much the rig
    tilted, which is almost completely insensitive to the extrinsic. Walls are
    the surfaces whose thickness actually reflects calibration quality.

    ARKit's world frame is gravity-aligned, so "vertical plane" means the normal
    is near-horizontal: |n · up| small.
    """
    if len(points) < 200:
        return float("nan"), 0
    up = np.array([0.0, 1.0, 0.0])
    rng = np.random.default_rng(seed)
    best_n, best = 0, (float("nan"), 0)

    for _ in range(iters):
        idx = rng.choice(len(points), 3, replace=False)
        a, b, c = points[idx]
        n = np.cross(b - a, c - a)
        norm = np.linalg.norm(n)
        if norm < 1e-9:
            continue
        n = n / norm
        if abs(n @ up) > 0.35:  # too close to horizontal — that's the scan slab
            continue

        inliers = np.abs((points - a) @ n) < inlier_m
        count = int(inliers.sum())
        if count <= best_n or count < min_frac * len(points):
            continue

        # Refit on the inlier set; three random points fix the orientation only
        # crudely, and the residual is the number we care about.
        pts = points[inliers]
        centroid = pts.mean(0)
        _, _, vt = np.linalg.svd(pts - centroid, full_matrices=False)
        nn = vt[2]
        if abs(nn @ up) > 0.35:
            continue
        # Robust spread, not the raw rms. A generous selection band is needed
        # to find the wall at all, but rms then reports the *band*: residuals
        # filling +/-T uniformly give T/sqrt(3) whatever the true thickness.
        # MAD is dominated by the core of the distribution, so a few off-wall
        # points in the band barely move it.
        resid = (pts - centroid) @ nn
        sigma = 1.4826 * float(np.median(np.abs(resid - np.median(resid))))

        best_n = count
        best = (sigma, count)

    return best
