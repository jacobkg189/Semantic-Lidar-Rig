# Phase 0 — Transport

**Goal:** get both sensor streams onto the Mac at their native rates.

**Gate:** ~10 Hz lidar revolutions and ~60 Hz ARKit poses, sustained, no drops.

Passing says transport works. It says nothing about whether the two streams
*agree* — that's Phases 2 and 3.

## Why the Mac is the bridge

The iPhone cannot read the C1 directly. iOS has no public API for USB serial
(no CDC-ACM, no FTDI/CP2102 driver), and the sanctioned alternative is MFi,
which means a hardware auth chip and Apple's licensing programme.

So v0 plugs the lidar into the Mac and lets the Mac do everything: it's already
where the working driver lives, it's where the fusion will run, and it needs no
new hardware. The cost is that v0 is a desk-or-cart rig rather than handheld —
fine through Phase 4, at which point a Pi on the mast takes over.

## Direction of the connection

The **phone listens, the Mac connects** — backwards from the obvious setup, and
forced by usbmuxd. `iproxy` maps a Mac-local port onto a device port, so traffic
only flows Mac → phone at connect time.

Wired over USB, not Wi-Fi: lower latency, no AP-isolation surprises, the phone
charges while capturing, and it likely dodges iOS's local-network permission
prompt (which fails by silently dropping packets, not by erroring).

## Running it

Three terminals, in order:

```bash
# 1. tunnel — Mac localhost:5555 → iPhone:5555
iproxy 5555 5555

# 2. app — build to the device, keep it foregrounded and unlocked
cd ios && xcodegen generate && open SemanticScanner.xcodeproj

# 3. gate
.venv/bin/python phase0_transport/check.py --seconds 10
```

Either stream can be checked alone with `--lidar-only` / `--phone-only`, which
is the fastest way to tell which half is broken.

## Files

| File | Role |
|---|---|
| `check.py` | The gate. Counts both streams, flags rates below tolerance |
| `phone_link.py` | TCP client, framing, decode. Stamps arrival time |
| `lidar_source.py` | C1 on a background thread behind a swappable `LidarSource` |
| `../shared/protocol.py` | Wire codec — mirrors `ios/Sources/WireFormat.swift` |
| `../ios/Sources/` | ARKit session, `NWListener`, Swift encoder |

## Design notes

**Transport sits behind an interface on both sides.** `LidarSource` has one
implementation today (`SerialLidarSource`, local USB); the Pi build adds a
second and nothing downstream changes.

**Rates are measured from the first sample, not from the start command.** The C1
takes 2–3 s to spin up. Counting that dead time understates every rate — it made
the bench test report 6 Hz for a healthy 10 Hz device.

**Poses are never dropped; camera frames are.** Poses are 53 bytes and
load-bearing for everything downstream. JPEG frames are the fat stream and get
dropped under backpressure rather than stalling poses behind them.

**Both clocks are recorded, neither is trusted yet.** ARKit's timestamp
(`CACurrentMediaTime`) and the Mac's arrival stamp are different time domains.
Phase 0 uses arrival time only; Phase 2 solves the offset offline, which is why
both get written down.

## Status — ✅ PASSED

Both streams, simultaneously, on real hardware:

```
Lidar revolutions  74   (10.3 Hz, nominal 10)
Lidar samples      36848 (5150 Hz, nominal 5000)
ARKit poses        719  (60.1 Hz, nominal 60)
Camera frames      120  (10.1 Hz, rate-limited on the phone)
Device             iPhone18,1, iOS 26.5.2
Capabilities       scene-reconstruction, scene-depth, camera-frames
```

Also verified along the way:

- Swift and Python encoders produce **byte-identical** frames — checked by
  compiling `WireFormat.swift` standalone and diffing its output against
  `protocol.py`, rather than assuming the two hand-written codecs agree
- All three ARKit capability bits are present, so mesh classification is
  available — Phase 5's free semantic layer, already enabled in the capture
  config, so recordings made now carry it

## The bug this phase nearly shipped

The first passing run reported **99 Hz** for a 60 Hz pose stream, with a pose
count that was exactly right for 60 Hz. The count was correct and the rate was
not, which meant the messages had arrived in a burst.

Cause: a single loop drained both sensors alternately. While it blocked on the
C1's 2–3 s spin-up, the phone kept sending at 60 Hz into the socket buffer, and
every one of those poses got stamped later, together, when the loop finally got
round to reading.

That would have passed a naive gate and silently poisoned Phase 2 — arrival
timestamps are precisely what the clock offset is solved from, and bunched
stamps make it unsolvable in a way that looks like a maths problem rather than a
plumbing one.

Fixes: one reader thread per stream, each stamping at the moment of arrival, and
a gate that now fails on rates **above** nominal too. Hardware cannot outrun its
own clock, so a too-fast rate always means queueing somewhere.
