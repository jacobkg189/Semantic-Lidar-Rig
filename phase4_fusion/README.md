# Phase 4 — Fusion

**Goal:** Remove ARKit drift with a pose graph, and add the phone's dense depth.

**Gate:** Loop closes; walls consistent across independent recordings.

## Status — ✅ pose graph passed · ⬜ depth built, awaiting a recording

```
walk 1   7.80 -> 3.33 cm   (+57%)   corrections median 1.8 cm, max 4.4 cm
walk 2   4.11 -> 3.44 cm   (+16%)   corrections median 4.0 cm, max 6.2 cm
```

The number that matters is not the improvement but the **agreement**: two
independent walks previously sat at 7.8 and 4.1 cm and now both land at ~3.4 cm.
Loop closure removed the per-recording drift variance. Corrections of a few
centimetres are the right magnitude for 90 s of indoor VIO.

## Architecture

**ARKit provides odometry. Lidar provides loop closure.** That split is the
whole design, and getting it backwards was the main bug (below).

The graph is **2D — (x, z, yaw)**. ARKit is gravity-aligned through the IMU, so
roll and pitch are absolutely referenced and do not drift; only yaw and position
do. Solving 6 DoF would spend effort on three that the IMU already constrains
far better than a planar lidar can, and the C1 has almost nothing to say about
the vertical anyway.

## The bug: never replace good odometry

The first version built sequential edges by ICP-matching consecutive keyframes.
The map got **3x worse** (4.11 -> 13.51 cm).

Isolating it took one test: feed the graph only ARKit edges and check it
reproduces the input. It did, to **0.00 mm** — so the optimiser and trajectory
warping were correct and the *edges* were wrong. Measuring those directly:

```
sequential ICP vs ARKit:  median 2.3 cm / 1.00 deg,  p90 11.8 cm / 4.46 deg
```

Over a 0.25 m / 1 s keyframe gap, ARKit is accurate to millimetres. ICP on a
sparse planar slice is not. Chaining 100 such errors together wrecked the map.
Lidar's job is loop closure — the one thing VIO cannot do — not odometry.

## Loop closures need a sanity gate

A near-square room has roughly 4-fold rotational symmetry, so ICP can snap to a
90 deg-rotated alignment that *scores better* than the truth. Unguarded, 2133 of
2232 edges were accepted and the map got 4x worse.

A loop closure may only refine the prior, never overrule it: matches claiming
more than 30 cm or 12 deg of correction are rejected outright, loop edges are
down-weighted against sequential ones, each keyframe accepts at most 4, and a
Huber loss bounds whatever still slips through.

## Dense depth (built, untested)

`SCENE_DEPTH` (wire type `0x04`) carries ARKit's 256x192 LiDAR depth as uint16
millimetres plus a confidence byte — 144 KB per frame at 5 Hz.

`depth.py` unprojects it into the world through the same pose-graph-corrected
trajectory, which is what keeps the dense and sparse clouds consistent.
Validated against synthetic ground truth: a flat wall at 2.0 m returns
z = -2.0000 with 0.0000 mm planarity.

Two sign traps, both of which produce plausible-looking wrong output:

- **Intrinsics must be scaled** from capture resolution (1920x1440) to depth
  resolution (256x192). Done on the phone; the scaled values travel per frame.
- ARKit's camera looks along **-Z** and image +v runs **down**, so unprojection
  negates both y and z. Getting either wrong mirrors the cloud.

Depth uses ARKit's clock, so the C1's serial lag does **not** apply to it —
mixing those up would shift the dense cloud against the sparse one.

## Why the room looks rotated

ARKit's Y is gravity-aligned and absolute, but X and Z are fixed by wherever the
phone pointed at session start. Measured: walk 1 is -10.71 deg off axis, walk 2
is +5.29 deg. Different per recording, which confirms the cause. The map is
correct; only the frame is arbitrary. A Manhattan alignment at render time will
square it up for floor-plan output.
