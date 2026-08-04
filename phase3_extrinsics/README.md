# Phase 3 — Extrinsic calibration

**Goal:** Solve the 6-DoF rigid transform between the C1 scan plane and the ARKit camera frame.

**Gate:** Lidar returns land on ARKit's reconstructed wall planes within ~2 cm.

**Status:** Not started — blocked on Phase 2.

These six numbers appear nowhere in the sensor data. No amount of compute
recovers them — fuse with a wrong transform and you get a confidently wrong map.

They can eventually be folded into the Phase 4 batch optimisation and solved
jointly with the trajectory, which is standard targetless calibration. But that
problem is non-convex and needs a decent initialisation to avoid settling into a
garbage local minimum. This phase produces that initialisation, and more
importantly a way to tell whether it worked.

Rig notes that feed in here: MagSafe is a rotational joint, so the phone can
reseat a degree or two off unless the anti-rotation feature holds it. Align the
C1's 0° datum (the triangle on its top face) deliberately with the camera
forward axis — near-identity yaw makes every later error obviously real rather
than a 90° convention mix-up.
