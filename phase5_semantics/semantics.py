"""Attach ARKit's semantic labels to the fused point clouds.

ARKit classifies every face of its reconstruction mesh as wall / floor /
ceiling / table / seat / window / door, on device, with no model of ours. That
is the free semantic tier: it covers most of what room measurement needs before
any CoreML work is justified.

The mesh and the point clouds are separate things, though — the mesh comes from
the phone's own reconstruction, the clouds from the C1 and from unprojected
depth. Labelling means transferring class from the nearest mesh face to each
point, which only works because both are placed by the same pose-graph-corrected
trajectory.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.spatial import cKDTree

LABEL_NAMES = {
    0: "none", 1: "wall", 2: "floor", 3: "ceiling",
    4: "table", 5: "seat", 6: "window", 7: "door",
}

# Distinct hues; "none" is deliberately dim so unlabelled geometry recedes.
LABEL_COLORS = {
    0: "#3a3a3a", 1: "#4c9ae8", 2: "#8a6a3a", 3: "#9b7fd4",
    4: "#e8b04c", 5: "#e8664c", 6: "#4ce8c8", 7: "#7de84c",
}

# A point further than this from any mesh face gets no label. ARKit's mesh only
# covers what the camera saw, while the C1 sweeps 360 degrees, so a large part
# of the lidar cloud legitimately has no mesh nearby — better to leave it
# unknown than to attach the class of a face a metre away.
MAX_ASSIGN_M = 0.20


@dataclass
class SemanticMesh:
    vertices: np.ndarray      # Nx3, world frame
    faces: np.ndarray         # Mx3 indices
    labels: np.ndarray        # M, uint8
    centroids: np.ndarray     # Mx3
    areas: np.ndarray         # M

    @property
    def tree(self) -> cKDTree:
        if not hasattr(self, "_tree"):
            self._tree = cKDTree(self.centroids)
        return self._tree


def load_mesh(reader) -> SemanticMesh:
    """Merge the newest chunk per anchor into one world-frame mesh.

    Chunks are replacements, not increments: ARKit revises anchors continuously,
    so keeping every chunk would pile up stale overlapping geometry.
    """
    from protocol import MsgType, decode  # shared module

    latest = {}
    for rec in reader.mesh():
        m = decode(MsgType.MESH_CHUNK, rec.payload)
        latest[m.anchor_id] = m

    V, F, L = [], [], []
    base = 0
    for m in latest.values():
        v, f, c, T = m.as_arrays()
        if len(v) == 0 or len(f) == 0:
            continue
        # anchor-local -> world
        vw = v @ T[:3, :3].T + T[:3, 3]
        V.append(vw)
        F.append(f.astype(np.int64) + base)
        L.append(c[:len(f)])
        base += len(v)

    if not V:
        raise ValueError("no mesh anchors in this session")

    vertices = np.concatenate(V)
    faces = np.concatenate(F)
    labels = np.concatenate(L)

    tri = vertices[faces]
    centroids = tri.mean(axis=1)
    areas = 0.5 * np.linalg.norm(
        np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]), axis=1)
    return SemanticMesh(vertices, faces, labels, centroids, areas)


def label_points(points: np.ndarray, mesh: SemanticMesh,
                 max_dist: float = MAX_ASSIGN_M) -> np.ndarray:
    """Nearest-face class per point; 255 where nothing is close enough.

    Nearest *centroid* rather than true point-to-triangle distance: ARKit's
    faces are a few centimetres across, so the difference is well inside the
    labelling tolerance and it is far cheaper.
    """
    if len(points) == 0:
        return np.empty(0, np.uint8)
    d, idx = mesh.tree.query(points, k=1, distance_upper_bound=max_dist)
    out = np.full(len(points), 255, np.uint8)
    hit = np.isfinite(d)
    out[hit] = mesh.labels[idx[hit]]
    return out


def summarise(labels: np.ndarray) -> list[tuple[str, int, float]]:
    total = len(labels)
    rows = []
    for k, n in zip(*np.unique(labels, return_counts=True)):
        name = "unlabelled" if k == 255 else LABEL_NAMES.get(int(k), str(k))
        rows.append((name, int(n), 100.0 * n / total))
    return sorted(rows, key=lambda r: -r[1])


def surface_area_by_class(mesh: SemanticMesh) -> dict[str, float]:
    """Square metres of mesh per class — the basis for room measurement."""
    out: dict[str, float] = {}
    for k in np.unique(mesh.labels):
        out[LABEL_NAMES.get(int(k), str(k))] = float(mesh.areas[mesh.labels == k].sum())
    return out
