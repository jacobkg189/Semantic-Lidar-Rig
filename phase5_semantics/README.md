# Phase 5 — Semantics

**Goal:** Attach semantic labels to the fused geometry.

**Gate:** Labels stable across viewpoints rather than flickering per-frame.

**Status:** Not started — blocked on Phase 4.

**Start with ARKit's own mesh classification** — wall, floor, ceiling, table,
seat, window, door, free with `.meshWithClassification`, already enabled in the
capture app. That covers most of what room measurement needs before any CoreML
work happens.

Only add a segmentation model if you need finer labels. If you do, note that SAM
/ segment-anything is *promptable* segmentation and returns masks with no class
names — for actual labels you want a semantic segmentation model or an
open-vocabulary CLIP-based one.

The core tension: the camera sees ~70°, the lidar sees 360°. Most lidar returns
have no label available at any instant, so this is inherently an accumulation
-over-time problem. Decide explicitly what happens to unlabelled geometry —
leave it unknown, or propagate from geometric neighbours. That choice drives the
map representation.
