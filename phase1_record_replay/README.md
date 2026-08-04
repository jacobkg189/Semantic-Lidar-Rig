# Phase 1 — Record & replay

**Goal:** Write both streams to disk with arrival timestamps, and replay them
through the pipeline identically.

**Gate:** A recording replays deterministically and losslessly, with timing
still intact.

## Status — ✅ PASSED

```
Session            recordings/2026-08-03T19-20-26  (20 s, 22 MB)
Deterministic      yes  sha256 b63f682673a25316
Counts match       1200 poses, 153 revs, 200 frames
Round trip         1200 poses decoded, 76968 lidar samples exact
poses              59.9 Hz median, worst gap 36 ms
lidar              10.0 Hz median, worst gap 104 ms
```

## Why this phase exists

Every phase from here on is a parameter you will tune dozens of times. Without
replay, each tweak costs a re-walk of the room instead of a re-run of a file.
It's the highest-leverage thing in the plan and the easiest to skip.

Both clocks (ARKit device time and Mac arrival time) are recorded, so Phase 2
can be solved offline from data captured today.

## Running it

```bash
python3 phase1_record_replay/record.py --seconds 30 --notes "kitchen loop"
python3 phase1_record_replay/check.py          # defaults to the latest session
```

The gate needs no hardware — that's the point.

## Format

One directory per session. Streams are separate files, not one interleaved log,
because Phases 2 and 3 want poses without paying to skip past JPEGs, and
fixed-size pose records make that a seek rather than a scan.

```
recordings/<timestamp>/
    manifest.json   metadata, counts, capabilities, git commit
    poses.bin       fixed 61-byte records (arrival + 53-byte wire payload)
    lidar.bin       variable-length revolutions
    frames.bin      length-prefixed JPEGs
```

**Payloads are stored verbatim as they arrived**, with an arrival timestamp
prepended. Nothing is re-encoded on the way to disk, so a recording cannot drift
from what the sensor actually sent, and replay is a read rather than a
reconstruction.

Lidar samples keep the C1's native fixed-point units (`angle_q6`, `dist_q2`).
The gate asserts every value lands exactly on the quantisation grid, so the
round trip is verified lossless rather than assumed close.

Roughly **1.1 MB/s**, almost entirely JPEG. `--no-frames` cuts it to a trickle
when you only need geometry.

## What the gate checks

"Replays deterministically" alone is nearly vacuous — re-reading a file twice
usually matches. So it checks four things:

1. **Determinism** — two replays hash identically. The merge sorts on
   `(arrival, stream, index)`; without that tiebreak, equal timestamps could
   legitimately order differently each run and the gate would be untestable.
2. **Counts** — manifest agrees with what's actually in the files, which catches
   a truncated or interrupted recording.
3. **Lossless round trip** — every pose payload decodes, every lidar sample is
   exact.
4. **Timing** — arrivals are monotonic, near nominal rate, and *not bunched*.

That last one is the important one. Arrival timestamps are what Phase 2 solves
the clock offset from, so a recording that bunches them is worthless for sync
while looking perfectly fine by every other measure. The check flags clusters of
near-zero gaps — exactly the failure Phase 0 hit.

## The bug this phase caught

The first recording attempt died instantly with `phone closed the connection`,
despite the app running and the TCP connect succeeding — it just sent **zero
bytes**.

A race in `PhoneServer.accept()`: it cancels the previous connection before
storing the new one, and `NWConnection` reports `.cancelled` asynchronously. The
stale callback landed *after* the replacement reported `.ready` and reset state
to `.listening`, at which point `send()`'s state guard silently dropped
everything, `HELLO` included.

Earlier reconnects had worked only because the timing fell the other way — a
genuine intermittent that would have resurfaced at the worst possible moment.

Fix: identity-check every connection callback against the current connection and
ignore stale ones. Verified by reconnecting twice in a row, which reproduced the
failure reliably before the fix.
