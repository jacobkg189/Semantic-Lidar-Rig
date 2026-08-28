# semanticScanning

Handheld semantic mapping rig: **RPLidar C1** (360° planar, ~12 m) fused with an
**iPhone 17 Pro** (ARKit VIO, LiDAR depth, semantic labels), MagSafe-mounted
below the lidar on a shared rigid frame.

The C1 supplies long-range geometry and drift correction; the phone supplies
pose, dense short-range depth, and semantics. Neither alone does the job.

## This is part 1

The end goal is **autonomous drone navigation through the house** using the
scanned map. That reframes what "good enough" means — this is the mapping
front-end of a robotics stack, not a visualisation project.

Consequences that drive design decisions:

- **A point cloud is not a navigation map.** A drone needs occupied / free /
  **unknown** per voxel, built by ray-casting from sensor poses. Unknown must be
  treated as solid.
- **Glass and thin obstacles are the real danger.** Both sensors are near-IR and
  largely blind to windows, wires and chair legs. A sensing limit, not a software
  one.
- **Height coverage matters.** Drones fly where a hand-held rig never scanned.
- **Whole-house means multi-session registration.** Each ARKit session has an
  arbitrary origin — two walks of the same room came out -10.7 deg and +5.3 deg
  apart.
- **The map is only a prior.** The drone still needs onboard localisation at
  flight time, which is a separate system of comparable size to this one.

Order before anything flies: Phase 6 validation → occupancy grid with free-space
carving → coverage tooling → multi-session registration.

## Layout

Work is split by phase — each phase folder owns its own code, README, goal, and
gate.

```
shared/              driver + wire-format codec, used by every phase
phase0_transport/    ✅ get both sensor streams onto the Mac
phase1_record_replay/   ✅ record to disk, replay deterministically
phase2_time_sync/       🟡 clock model done; lidar lag provisional
phase3_extrinsics/      🟡 params cross-validated; residual is drift (Phase 4)
phase4_fusion/          ✅ pose graph; ⬜ dense depth awaiting a recording
phase5_semantics/       ✅ ARKit mesh classification, 8 classes
phase6_validation/      ⬜ measure the real error budget
phase7_occupancy/       ✅ occupancy grid — free/occupied/unknown for planning
ios/                 the iOS app (grows across phases, XcodeGen-managed)
test/                hardware bring-up bench test for the C1
docs/                PLAN.md (all phases) and WIRE_FORMAT.md (the contract)
```

`ios/` and `shared/` sit outside the phase folders because they're living
artifacts that most phases touch. Each phase README records what it changed in
them, so the per-phase view still holds.

## Setup

```bash
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
brew install xcodegen libimobiledevice
cd ios && xcodegen generate
```

## Phase status

| Phase | Goal | Gate | Status |
|---|---|---|---|
| 0 | Transport | Both streams at stable rates | ✅ **Passed** — 10.3 Hz lidar, 60.1 Hz poses |
| 1 | Record & replay | Replay deterministic + lossless | ✅ **Passed** — 1200 poses, 153 revs, exact |
| 2 | Time sync | Reproducible clock model | 🟡 clock model ✅; lag provisional, refined in Phase 3 |
| 3 | Extrinsics | Self-consistent walls | 🟡 params cross-validated (1.3 cm); walls 4.2 cm, drift-limited |
| 4 | Fusion | Loop closes; walls consistent | ✅ pose graph (3.4 cm, both walks); depth built |
| 5 | Semantics | Labels stable across viewpoints | ✅ **Passed** — 8 classes, 147k faces |
| 6 | Validation | Error budget vs tape measure | Not started |
| 7 | Occupancy | Navigable volume for planning | ✅ 18.4 m³ free, 6.5 m³ at 30 cm clearance |

Full detail, including why the gates are shaped this way, in
[docs/PLAN.md](docs/PLAN.md).

## Hardware bring-up

The C1 is verified working — 5102 Hz sample rate, 10.2 Hz scan rate, 358/360
angular coverage:

```bash
.venv/bin/python test/test_lidar.py
```

One C1 quirk worth knowing before it costs you an hour: the device will
acknowledge a `SCAN` command while wedged, returning a valid response descriptor
and then no samples at all, indefinitely. `STOP` does not clear this — only
`RESET` does. The driver resets on every connect.
