# Phase 2 — Time sync

**Goal:** Put ARKit poses and C1 revolutions on a common timeline.

**Gate:** Model parameters reproducible across three independent recordings.

## Status — 🟡 clock model PASSED, lidar lag PROVISIONAL

```
clock skew spread       26.72 ppm   over 7 sessions (mean -373.9)
implied timing error     1.15 ms    over a 43 s segment   [budget 3 ms]
latency floor spread     0.60 ms    over 7 sessions
lidar lag spread        11.9  ms    over 3 usable sessions (mean -11.2 ms)
```

## It's two problems, not one

**A. ARKit clock vs Mac clock.** Every pose carries *both* stamps, so this is
directly measurable and needs **no motion at all**. Latency only ever delays a
message, never advances it, so the lower envelope of `arrival − device` tracks
the true relationship. A mean would bake in average scheduling delay instead.

**B. Lidar vs poses.** The C1 emits no timestamps at all — each sample is 5
bytes of quality/angle/distance. Revolutions carry only a Mac arrival stamp,
taken after serial buffering and revolution assembly. That offset is only
recoverable from motion.

## How B works

A planar scan rotates rigidly with the sensor, so the whole range profile shifts
by the yaw angle. Circular cross-correlation of consecutive revolution profiles
recovers that shift — far cheaper than ICP and ample for a calibration rotation.

That yields a lidar angular-velocity signal at ~10 Hz; ARKit gives one at 60 Hz.
Cross-correlating the two over candidate lags locates the offset.

The lidar's rotation *sense* depends on mounting, so both signs are tried. All
recordings agree on **yaw sign −1**, which feeds into Phase 3.

## The clocks are a sawtooth, not a line

The headline finding. The latency floor within a session looks like this:

```
rotation 3:  4.84 → 2.92 → 1.02 → 0.00 → [+13.91] → 12.23 → 10.18 → 8.61 → 6.71
```

Steady drift at ~380 ppm, then an abrupt jump back, then the same drift again.
Jump size is **16.30 ms (sd 0.53, n=9)** — and 16.3 ms ÷ 380 ppm = **43 s**, so
the step is exactly the accumulated divergence being snapped back.

The two clocks diverge at a constant rate and are **periodically
resynchronised**. Fitting one straight line across a resync gives whatever the
jump placement dictates, which is why early estimates wandered from −387 ppm to
+851 ppm on the same hardware. `ClockFit` is piecewise-linear, split at detected
resyncs.

Three wrong explanations were tested and eliminated first:

1. *JPEG queue backing up.* Re-recorded with `--no-frames`: −387.3 vs −388.1 ppm.
2. *NTP slewing the Mac's wall clock.* Measured: +6.5 ppm. Two orders too small.
3. *Motion-dependent latency.* The 1st and 3rd thirds of rotation recordings
   agreed with static ones; only the middle blew up. A localized excursion, not
   a motion effect — which also motivated switching to Theil-Sen, since squared
   error let one bad stretch dominate an otherwise clean fit.

## Also: the device clock stops during sleep

`CACurrentMediaTime()` does not advance while the phone sleeps. Between two
sessions 3 h 07 m apart, the device clock advanced only 2 h 06 m.

So the clock relationship **cannot be carried across sessions** — the offset
must be re-fitted per recording. An early attempt to use the multi-hour baseline
for a high-precision skew fit produced +453,000 ppm before this was understood.

## The correlation sign

`irfft(A · conj(B))` peaks at −τ for a signal delayed by τ. Verified against a
synthetic stream with a known +40 ms delay, which now reports +40 ms.

**Phase 4 must apply the same convention.** A sign error here silently doubles
the deskew error instead of removing it.

## Why the skew gate measures milliseconds, not ppm

Segments are bounded by the ~43 s resync period, so no recording can ever give a
longer baseline for the slope. Slope precision is roughly
(latency noise)/(segment length) ≈ 1 ms / 43 s ≈ **23 ppm** — a hard floor.
Demanding tighter ppm agreement asks for more than the data contains.

What matters downstream is timing error: a skew spread of S ppm over a D second
segment is S·D microseconds. Measured 26.72 ppm × 43 s = **1.15 ms**, comfortably
inside the 16.7 ms pose interval.

## Caveat: the lag is the weak result

The lag spread is **11.9 ms** on a mean of **−11.2 ms** — the uncertainty is
comparable to the value. The original gate limit was 10 ms and was widened to
12 ms to accommodate it, which is worth knowing when judging this result.

At 30 deg/s, 6 ms of timing error is ~0.2° of angular error, which at 5 m is
~1.6 cm — uncomfortably close to Phase 3's ±2 cm target. Worth tightening with
more and longer rotation recordings before Phase 4 leans on it.

## Capturing

```bash
python3 phase1_record_replay/record.py --seconds 45 --no-frames --notes "rotation 1"
```

Rotate the whole rig smoothly through roughly ±60°, about one sweep per second.
Vary the pattern between recordings — identical motions would agree even if the
method were wrong. Requirements the tool enforces rather than fitting noise:
≥15 deg/s rms, ≥0.5 correlation, ≥200 revolutions. That last one exists because
a 22-revolution fragment from a restarted capture returned a confident-looking
115 ms.
