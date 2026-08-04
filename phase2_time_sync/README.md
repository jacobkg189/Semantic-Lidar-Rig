# Phase 2 — Time sync

**Goal:** Solve the fixed offset between ARKit's clock and the Mac's lidar arrival stamps.

**Gate:** Offset stable to within a few milliseconds across three separate recordings.

**Status:** Not started — blocked on Phase 1.

The C1 sends no timestamps at all — in standard scan mode each sample is 5 bytes
of quality/angle/distance. So this is not 'device clock vs phone clock', it is
'Mac arrival time vs ARKit time', with a fixed serial + USB latency baked in.

Method: rotate the rig sharply, then cross-correlate ARKit's angular velocity
against rotation derived from lidar scan-matching. The motion signature is
distinctive enough that a naive correlation gets close.

Must pass before Phase 4 means anything — bad sync and bad extrinsics produce an
identical smeared cloud, so debugging them together is far harder than
separately.
