"""Extract measurable quantities from the map, for comparison against a tape.

Everything up to here has been *self*-consistency: the map agrees with itself,
and two sensors agree with each other. Neither shows it agrees with the actual
house. This is the first external check, and it is what turns "walls are 3.4 cm
thick" into an error budget you can design safety margins around.

The output is deliberately a short list of things that are easy to measure
physically — wall-to-wall spans, ceiling height, door width — rather than
statistics that have no tape-measure equivalent.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

# Slab used to find walls: high enough to clear furniture, low enough to avoid
# ceiling slope and coving.
WALL_BAND = (-0.45, 0.25)

# A wall shows up as a sharp peak in the point histogram along its normal.
HIST_BIN_M = 0.01
PEAK_MIN_FRAC = 0.04


@dataclass
class Measurement:
    name: str
    value_m: float
    detail: str = ""


def manhattan_angle(xz: np.ndarray, step_deg: float = 0.25) -> float:
    """Dominant wall direction, in radians, by rotation search.

    ARKit's X and Z axes are fixed by wherever the phone pointed at session
    start, so the room sits at an arbitrary yaw — measured at -10.7 deg on one
    walk and +5.3 deg on another.

    Searching beats any closed-form trick here. An earlier version differenced
    points sorted by bearing and took the circular mean of 4*theta; that assumes
    a clean convex boundary, and furniture wrecks it badly enough that the two
    sensors disagreed by 56 degrees on the same room.

    When the walls are axis-aligned, their points pile into a few histogram bins
    per axis. Sum-of-squares of the bin counts measures exactly that
    concentration, and peaks at the correct rotation.
    """
    if len(xz) > 200_000:                      # score is stable well before this
        xz = xz[np.linspace(0, len(xz) - 1, 200_000).astype(int)]
    best = (-1.0, 0.0)
    for deg in np.arange(0.0, 90.0, step_deg):
        a = np.radians(deg)
        c, s = np.cos(-a), np.sin(-a)
        r = xz @ np.array([[c, -s], [s, c]]).T
        score = 0.0
        for axis in (0, 1):
            v = r[:, axis]
            h, _ = np.histogram(v, bins=np.arange(v.min(), v.max() + 0.02, 0.02))
            score += float(np.sum((h / max(h.sum(), 1)) ** 2))
        if score > best[0]:
            best = (score, a)
    return float(best[1])


def rotate(xz: np.ndarray, ang: float) -> np.ndarray:
    c, s = np.cos(-ang), np.sin(-ang)
    return xz @ np.array([[c, -s], [s, c]]).T


def wall_positions(v: np.ndarray, min_frac: float = PEAK_MIN_FRAC) -> np.ndarray:
    """Positions of wall planes along one axis, from histogram peaks."""
    lo, hi = v.min(), v.max()
    bins = np.arange(lo, hi + HIST_BIN_M, HIST_BIN_M)
    h, edges = np.histogram(v, bins=bins)
    thresh = min_frac * h.max()

    peaks = []
    i = 0
    while i < len(h):
        if h[i] >= thresh:
            j = i
            while j < len(h) and h[j] >= thresh:
                j += 1
            seg = h[i:j]
            # Intensity-weighted centre of the run, so a slightly thick wall
            # still yields one position rather than two.
            centre = np.average(edges[i:j] + HIST_BIN_M / 2, weights=seg)
            peaks.append((seg.sum(), centre))
            i = j
        else:
            i += 1
    peaks.sort(key=lambda p: -p[0])
    return np.array(sorted(c for _, c in peaks))


def measure_room(points: np.ndarray, labels: np.ndarray | None = None) -> list[Measurement]:
    out: list[Measurement] = []
    y = points[:, 1]
    floor_ref = np.percentile(y, 1)

    band = points[(y - np.median(y) > WALL_BAND[0]) & (y - np.median(y) < WALL_BAND[1])]
    xz = band[:, [0, 2]]
    ang = manhattan_angle(xz)
    out.append(Measurement("room yaw vs ARKit axes", np.degrees(ang), "deg — arbitrary, not an error"))

    r = rotate(xz, ang)
    for axis, name in ((0, "A"), (1, "B")):
        pk = wall_positions(r[:, axis])
        if len(pk) >= 2:
            span = float(pk[-1] - pk[0])
            out.append(Measurement(f"wall span {name} (outer to outer)", span,
                                   f"{len(pk)} wall planes found on this axis"))

    # Ceiling height: floor to ceiling, using labels where available since the
    # C1 barely sees the floor and a percentile alone is unreliable.
    if labels is not None:
        fl = points[labels == 2]
        ce = points[labels == 3]
        if len(fl) > 500 and len(ce) > 500:
            f = float(np.percentile(fl[:, 1], 50))
            c = float(np.percentile(ce[:, 1], 50))
            out.append(Measurement("floor to ceiling", c - f,
                                   f"{len(fl)} floor pts, {len(ce)} ceiling pts"))

        # Doors: cluster the door-labelled points and report each width.
        dr = points[labels == 7]
        if len(dr) > 500:
            from scipy import ndimage
            V = 0.06
            key = np.floor(dr / V).astype(np.int64)
            mn = key.min(0)
            idx = key - mn
            grid = np.zeros(idx.max(0) + 3, bool)
            grid[idx[:, 0], idx[:, 1], idx[:, 2]] = True
            lab, n = ndimage.label(grid, np.ones((3, 3, 3)))
            comp = lab[idx[:, 0], idx[:, 1], idx[:, 2]]
            for c in range(1, n + 1):
                m = comp == c
                if m.sum() < 400:
                    continue
                q = dr[m]
                qr = rotate(q[:, [0, 2]], ang)
                width = float(max(np.ptp(qr[:, 0]), np.ptp(qr[:, 1])))
                height = float(np.ptp(q[:, 1]))
                if width < 0.4 or width > 2.5:
                    continue
                out.append(Measurement("door width", width,
                                       f"height {height:.2f} m, {int(m.sum())} pts"))
    else:
        out.append(Measurement("floor to ceiling", float(np.percentile(y, 99) - floor_ref),
                               "percentile estimate — no labels supplied"))
    return out


def compare(predicted: list[Measurement], truth: dict[str, float]) -> list[tuple]:
    """Pair measurements against tape values. Returns (name, map, tape, err, pct)."""
    rows = []
    for m in predicted:
        if m.name in truth:
            t = truth[m.name]
            err = m.value_m - t
            rows.append((m.name, m.value_m, t, err, 100.0 * err / t if t else float("nan")))
    return rows
