# Phase 3 — Extrinsic calibration

**Goal:** Solve the 6-DoF rigid transform between the C1 scan plane and the
ARKit camera frame, plus a residual time offset carried over from Phase 2.

**Gate:** Walls accumulate to ≤2 cm rms across independent recordings.

## Status — ⬜ blocked on a suitable capture

Tooling is built and validated. The existing recordings cannot solve it, for a
reason worth understanding rather than working around.

## The method

If the extrinsic is right, a wall scanned from many viewpoints accumulates into
a *thin* surface in the world frame. If it's wrong, the same wall smears into a
slab. That needs no second sensor to trust — the alternative, matching against
ARKit's reconstruction mesh, would require sending scene geometry from the
phone, which the app does not do yet.

Seven parameters solved jointly: three rotation, three translation, and `dt`.
`dt` is included deliberately — Phase 2's isolated lag estimate hit its noise
floor at ±6 ms, and solving it here gives it every lidar return as a constraint
instead of one yaw estimate per revolution.

## Why the current recordings can't solve it

They were captured for Phase 2, which wanted rotation in place. Measured:

```
rotation 1   translation extent [0.18  0.10  0.29] m
lag A2       translation extent [0.40  0.11  0.24] m
```

Self-consistency works because *the same surface seen from different places must
coincide*. With no translation there are no different places: every revolution
observes the room from one point, so a wrong extrinsic yields a map that is
consistently wrong and perfectly self-consistent.

Measured sensitivity confirms it — perturbing the extrinsic by **20 degrees**
changes wall thickness by **0.01 cm**:

```
rot + 0 deg   wall 3.42 cm      trans +  0 cm  wall 3.42 cm
rot +20 deg   wall 3.41 cm      trans + 20 cm  wall 3.44 cm
```

Rotation about the spin axis is not merely weak but *fundamentally*
unobservable: it rotates the entire map, which self-consistency cannot see. Only
an external reference or translation can pin it down.

## The operator is a third of every scan

Flagged by the user, and much larger than expected. The person holding the rig
stands behind it and turns with it, so they sit at a fixed bearing in the lidar
frame at close range:

```
rotation 1   sector   0-105 deg, median range 0.34-0.70 m   41% of all returns under 0.8 m
lag A2       sector  20- 70 deg, median range 0.54-0.79 m   29% of all returns under 0.8 m
```

In the world frame those returns smear everywhere regardless of calibration.
`mask_operator()` detects the sector per-recording — it moves with how the rig is
held, so a hardcoded mask would be wrong as often as right.

It removed **33 of 72 bins** from one recording. That is a 46% blind arc, which
matters well beyond calibration: it is geometry the map will never contain.
Holding the rig further from the body, or on a short boom, would buy that back.

## A metric bug worth remembering

The first version fitted the dominant plane without constraint and reported
77-90% of points as inliers. No wall in a room holds 77% of returns — it was
locking onto the **horizontal scan slab itself**, since a level planar lidar puts
every return into one. It was measuring how much the rig tilted, not how thick
the walls were, and was almost perfectly insensitive to the extrinsic.

`wall_thickness()` now rejects planes whose normal is near-vertical.

## What a Phase 3 capture needs

The opposite of Phase 2's:

- **Translation, several metres of it.** Walk a full circuit of the room and
  return to the start. This is the requirement that matters most.
- **Viewpoint diversity.** Approach walls and back away; see the same surfaces
  from genuinely different positions and distances.
- **Deliberate tilt**, roughly ±20 deg of pitch. The C1 held level only ever sees
  vertical walls, whose normals are all horizontal — which leaves translation
  along the spin axis unobservable. Tilting cuts the floor and ceiling and
  supplies the missing normals. Phase 2 treated tilt as noise; here it is
  essential.
- **Keep the rig away from your body** as much as the mount allows.
- 90 s, `--no-frames`, three separate recordings so the result can be
  cross-validated rather than merely fitted.
