"""3D occupancy grid with explicit free / occupied / unknown.

This is the step that turns a map into a *navigation* map, and the distinction
matters more than it sounds: a point cloud records where surfaces are, and says
nothing about where it is safe to fly. The volume a drone cares about most is the
volume with nothing in it, and a cloud has no representation of that at all.

Free space is *carved*, not inferred. Every sensor return defines a ray from the
sensor to the hit point; everything along that ray was observed to be empty, and
the endpoint is a surface. Anything no ray ever passed through stays **unknown**,
which a planner must treat as solid.

Log-odds accumulation, as in OctoMap: repeated observations of the same voxel
reinforce each other, so a single spurious return cannot punch a hole through a
wall and a single missed return cannot fill a doorway.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

# 5 cm suits a drone: fine enough to resolve a doorway, coarse enough that a
# whole house stays tractable. A 4x5x3 m room is ~200k voxels at this size.
VOXEL_M = 0.05

# Log-odds increments. Occupied evidence is weighted more heavily than free —
# missing an obstacle is dangerous, over-reporting one is merely inconvenient.
L_OCC = 0.85
L_FREE = -0.4
L_MIN, L_MAX = -4.0, 6.0

# Decision thresholds on the accumulated log-odds.
OCC_THRESH = 0.9
FREE_THRESH = -0.9

# Rays are sampled rather than traversed exactly (a true DDA per ray is far too
# slow in Python for millions of returns). Step size below the voxel edge so no
# voxel along a ray is skipped.
STEP_M = 0.035
MAX_STEPS = 220


@dataclass
class OccupancyGrid:
    origin: np.ndarray      # world position of voxel (0,0,0) corner
    shape: tuple
    voxel_m: float
    logodds: np.ndarray     # float32, shape

    @property
    def occupied(self) -> np.ndarray:
        return self.logodds > OCC_THRESH

    @property
    def free(self) -> np.ndarray:
        return self.logodds < FREE_THRESH

    @property
    def unknown(self) -> np.ndarray:
        return (self.logodds >= FREE_THRESH) & (self.logodds <= OCC_THRESH)

    def counts(self) -> dict:
        n = int(np.prod(self.shape))
        o, f = int(self.occupied.sum()), int(self.free.sum())
        return {
            "voxels": n,
            "occupied": o,
            "free": f,
            "unknown": n - o - f,
            "occupied_m3": o * self.voxel_m ** 3,
            "free_m3": f * self.voxel_m ** 3,
            "unknown_m3": (n - o - f) * self.voxel_m ** 3,
        }

    def world_to_index(self, pts: np.ndarray) -> np.ndarray:
        return np.floor((pts - self.origin) / self.voxel_m).astype(np.int32)


def build(sensor_origins: np.ndarray, hits: np.ndarray,
          voxel_m: float = VOXEL_M, pad_m: float = 0.3,
          chunk: int = 40_000) -> OccupancyGrid:
    """Carve free space along every sensor->hit ray, mark endpoints occupied.

    `sensor_origins` is per-hit: each return has its own sensor pose, because the
    rig is moving and a revolution spans ~100 ms of motion.
    """
    lo = np.minimum(hits.min(0), sensor_origins.min(0)) - pad_m
    hi = np.maximum(hits.max(0), sensor_origins.max(0)) + pad_m
    shape = tuple(np.ceil((hi - lo) / voxel_m).astype(int) + 1)
    grid = np.zeros(shape, np.float32)

    steps = np.arange(MAX_STEPS, dtype=np.float32) * STEP_M

    for s in range(0, len(hits), chunk):
        o = sensor_origins[s:s + chunk]
        h = hits[s:s + chunk]
        d = h - o
        length = np.linalg.norm(d, axis=1, keepdims=True)
        length[length < 1e-6] = 1e-6
        unit = d / length

        # Sample along each ray, stopping one step short of the surface so the
        # endpoint itself is not marked free by its own ray.
        valid = steps[None, :] < (length - STEP_M)
        pts = o[:, None, :] + unit[:, None, :] * steps[None, :, None]

        idx = np.floor((pts - lo) / voxel_m).astype(np.int32)
        v = valid.ravel()
        idx = idx.reshape(-1, 3)[v]
        inside = np.all((idx >= 0) & (idx < np.array(shape)), axis=1)
        idx = idx[inside]
        np.add.at(grid, (idx[:, 0], idx[:, 1], idx[:, 2]), L_FREE)

        # Occupied endpoints last, so they win over free evidence from rays that
        # grazed the same voxel.
        hidx = np.floor((h - lo) / voxel_m).astype(np.int32)
        ok = np.all((hidx >= 0) & (hidx < np.array(shape)), axis=1)
        hidx = hidx[ok]
        np.add.at(grid, (hidx[:, 0], hidx[:, 1], hidx[:, 2]), L_OCC)

        np.clip(grid, L_MIN, L_MAX, out=grid)

    return OccupancyGrid(lo, shape, voxel_m, grid)


def inflate(grid: OccupancyGrid, radius_m: float) -> np.ndarray:
    """Obstacles grown by the drone's radius, so a planner can treat it as a
    point. Unknown is inflated too — it must be assumed solid."""
    from scipy import ndimage
    blocked = grid.occupied | grid.unknown
    r = max(1, int(round(radius_m / grid.voxel_m)))
    return ndimage.binary_dilation(blocked, ndimage.generate_binary_structure(3, 1),
                                   iterations=r)


def reachable_free(grid: OccupancyGrid, seed_world: np.ndarray,
                   clearance_m: float = 0.0) -> np.ndarray:
    """Free voxels actually connected to a starting point.

    Free space that no path reaches is not navigable, and counting it flatters
    the map. Seeded from the trajectory, which is known-flyable by construction.
    """
    from scipy import ndimage
    free = grid.free
    if clearance_m > 0:
        free = free & ~inflate(grid, clearance_m)
    lab, _ = ndimage.label(free, ndimage.generate_binary_structure(3, 1))
    seeds = grid.world_to_index(np.atleast_2d(seed_world))
    ids = set()
    for s in seeds:
        if np.all((s >= 0) & (s < np.array(grid.shape))) and lab[tuple(s)]:
            ids.add(int(lab[tuple(s)]))
    return np.isin(lab, list(ids)) if ids else np.zeros_like(free)
