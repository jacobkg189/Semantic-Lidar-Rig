# Phase 7 — Occupancy grid

**Goal:** Turn the fused map into something a path planner can consume.

**Why it exists:** a point cloud records where surfaces *are* and says nothing
about where it is safe to fly. The volume a drone cares about most is the volume
with nothing in it, and a cloud has no representation of that at all.

## First result (`recordings/2026-08-12T23-04-06`, 5 cm voxels)

```
occupied     3.8%    3.44 m3
free        20.5%   18.39 m3
unknown     75.7%   67.84 m3     <- must be treated as solid

flyable volume by drone radius:   10 cm  13.31 m3
                                  20 cm   9.45 m3
                                  30 cm   6.49 m3
```

## Three findings that shape the drone side

**Height dominates.** At 0.4 m the slice is almost entirely occupied — floor and
furniture, essentially nothing navigable. At 1.9 m it is nearly all flyable. A
drone should cruise high and descend only at waypoints.

**Unknown pockets sit inside the room, not just behind walls.** Those are
volumes no ray ever passed through, in the middle of the flight envelope. A
planner must refuse to route through them, so they either get scanned or they
permanently block paths.

**Clearance is the dominant constraint.** A 30 cm drone loses 65% of the free
space. In a domestic room that is not a minor correction.

## Method

Free space is *carved*, not inferred: every return defines a ray from sensor to
hit point, everything along it was observed empty, and the endpoint is a
surface. Anything no ray traversed stays unknown.

Log-odds accumulation as in OctoMap, so repeated observations reinforce: one
spurious return cannot punch a hole through a wall, and one missed return cannot
fill a doorway. Occupied evidence is weighted above free evidence (0.85 vs 0.40)
because missing an obstacle is dangerous while over-reporting one is merely
inconvenient.

Rays are sampled at 3.5 cm rather than traversed by exact DDA — a true DDA per
ray is far too slow in Python for a million returns, and a sub-voxel step skips
nothing.

`reachable_free()` reports only free space actually *connected* to the
trajectory. Disconnected free space is not navigable, and counting it flatters
the map.

## Capture metric

The unknown fraction is a direct, checkable measure of scan completeness.
ARKit only carves free space where a depth ray actually travelled, so pointing
at walls from across the room leaves the space between unobserved. Sweep the
camera through the volume, rebuild, and watch the number drop.
