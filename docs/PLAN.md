# Plan

Seven phases, each with a gate. Detail lives in each phase's own README; this is
the overview and the reasoning behind the shape.

## Why gates at all

**Every failure in this system produces the same symptom: a blurry, smeared
point cloud.** Bad time sync smears it. Bad extrinsics smear it. Missing deskew
smears it. ARKit drift smears it.

Build it all at once and a soft-looking result gives you four suspects, no way
to isolate any of them, and every fix you try moves the same number. Gate each
phase and every stage has exactly one new suspect when it goes wrong.

This is a debuggability argument, not a compute one. The pipeline is otherwise
entirely offline batch processing — record everything, fuse later, with as much
compute as you like. Phase 0 exists only to get bytes off the devices.

## The phases

| # | Phase | Gate |
|---|---|---|
| 0 | [Transport](../phase0_transport/README.md) | ~10 Hz lidar, ~60 Hz poses, sustained |
| 1 | [Record & replay](../phase1_record_replay/README.md) | Replay twice → bit-identical |
| 2 | [Time sync](../phase2_time_sync/README.md) | Offset stable to a few ms across 3 runs |
| 3 | [Extrinsics](../phase3_extrinsics/README.md) | Lidar lands on ARKit walls ±2 cm |
| 4 | [Fusion](../phase4_fusion/README.md) | Room loop closes under ~5 cm |
| 5 | [Semantics](../phase5_semantics/README.md) | Labels stable across viewpoints |
| 6 | [Validation](../phase6_validation/README.md) | Measured error budget vs tape |

## Sequencing notes

**Phase 1 is not skippable.** It's the difference between a two-minute iteration
and a twenty-minute one, multiplied by a few hundred iterations.

**Phases 2 and 3 must both pass before Phase 4 means anything.** They're
separated precisely because their failure modes are indistinguishable.

**Phase 5 has a free tier.** ARKit already classifies its reconstruction mesh
(wall / floor / ceiling / table / seat / window / door). The capture app enables
it from day one, so recordings made during Phase 0 already carry labels. Reach
for CoreML only when that isn't enough.

## Architecture

```
C1 ──USB──> ┌──────────┐ <──USB (usbmuxd)── iPhone
            │ MacBook  │                    ARKit: pose, depth, frames
            │ fusion   │
            └──────────┘
```

The phone is a *sensor*, not a compute platform. Pose-graph optimisation in
Swift on-device is miserable; the same thing in Python against recorded files is
a pip install. Push work on-device later, only where latency demands it.

**v0 is a desk-or-cart rig** — everything is tethered to the laptop. That's fine
through Phase 4. The handheld mast build then needs a Pi on the rig streaming
lidar over the network, which is why `LidarSource` is an interface with one
implementation rather than a direct serial call.

## Division of labour between the sensors

Worth being explicit, because it tells you where effort pays off:

- **iPhone** — 6-DoF pose (VIO, already IMU-fused), dense depth to ~5 m,
  semantic labels, camera intrinsics. Drifts over a long walkthrough.
- **C1** — 360° planar geometry to ~12 m, better range accuracy, no pose and no
  semantics of its own.

Dense 3D geometry comes mostly from the phone. The C1 earns its place by seeing
past the phone's ~5 m limit and by supplying hard geometric constraints that
correct ARKit's drift.

And note the C1 is a *planar* scanner — held still it gives one horizontal
slice, a floor plan rather than a model. Its 3D contribution exists only because
the rig moves and the pose at each instant is known.
