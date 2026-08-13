# Phase 5 — Semantics

**Goal:** Attach semantic labels to the fused geometry.

**Gate:** Labels stable across viewpoints rather than flickering per-frame.

## Status — ✅ working (ARKit's free tier; no CoreML needed)

From `recordings/2026-08-12T23-04-06` (90 s, 93 MB):

```
16 mesh anchors from 151 chunks -> 80,854 vertices, 147,791 classified faces

mesh surface area          C1 cloud            iPhone depth cloud
  wall     17.32 m2          wall     45.4%      wall     37.5%
  none     12.25             none     16.1%      none     32.0%
  ceiling   9.38             door     15.8%      ceiling   9.1%
  door      3.88             window   10.3%      door      8.0%
  floor     3.43             ceiling   8.7%      window    7.2%
  window    1.85             unlab.    2.9%      floor     2.7%
  seat      0.91             seat      0.4%      seat      2.1%
  table     0.76             floor     0.1%      table     1.4%
```

All eight ARKit categories present, computed on device.

## Why no model of our own

ARKit's scene reconstruction already classifies every mesh face as wall / floor
/ ceiling / table / seat / window / door. It is free, runs on device, is stable
across viewpoints by construction (the label lives on the mesh, not on a frame),
and covers essentially everything room measurement needs.

Reach for CoreML only when a class ARKit lacks is genuinely required. Note that
SAM-style *promptable* segmentation returns masks with no class names and would
not help here — that would need a semantic segmentation model or an
open-vocabulary one.

## How labelling works

Mesh chunks are **replacements keyed on anchor UUID**, not increments — ARKit
revises anchors continuously, so keeping every chunk would pile up stale
overlapping geometry. The reader keeps the newest per anchor: 151 chunks
collapse to 16 anchors.

Each point takes the class of the nearest mesh face centroid, within 20 cm.
Nearest-centroid rather than true point-to-triangle distance: faces are a few
centimetres across, so the difference is well inside the tolerance and it is far
cheaper.

Points further than 20 cm from any face stay **unlabelled** rather than
borrowing a distant class. That matters because the mesh only covers what the
camera saw, while the C1 sweeps 360 degrees.

## What the class distributions reveal

**The C1 sees almost no floor — 0.1% versus the phone's 2.7%.** It is a planar
scanner sweeping roughly horizontally, so it only catches the floor when tilted
steeply. The phone's depth camera sees it constantly.

**The C1 has more unlabelled points — 2.9% versus 0.0%.** Same cause in reverse:
the C1 sweeps 360 degrees and picks up geometry behind the operator that the
camera never looked at, so there is no mesh nearby to label it from.

Two sensors, complementary failure modes, both visible in one table.

## Remaining

**26% of faces are `none`.** ARKit declines to classify surfaces it is unsure
about — clutter, thin objects, poorly-lit regions. Slower sweeps and better
lighting reduce it; it cannot be eliminated.
