# Phase 4 — Fusion

**Goal:** Deskew, accumulate into a map, and pose-graph optimise with ARKit odometry plus scan-match constraints.

**Gate:** Walk a loop around one room and close it with under ~5 cm of drift.

**Status:** Not started — blocked on Phase 3.

**Deskew is not optional.** The C1 sweeps at 10 Hz, so a walking rig moves
meaningfully mid-revolution. Every point needs its own interpolated pose, not one
pose per scan.

**Treat the lidar as 3D, not 2D.** The moment the rig tilts, the scan plane
leaves horizontal. Place each return via the full 6-DoF pose. Building it as a 2D
scan in a 2D world and retrofitting later is painful.

**Optimise offline on the Mac.** GTSAM/g2o on-device in Swift is miserable; in
Python against recorded files it is a pip install and a fast iteration loop.

Note where the 3D actually comes from: the C1 is a planar scanner and produces
volume only because the rig moves. Dense geometry is mostly the iPhone's; the C1
contributes long range and drift correction.
