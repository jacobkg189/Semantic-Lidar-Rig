"""Time synchronisation estimators.

Two separate problems, which is easy to miss and worth stating plainly:

**A. ARKit clock vs Mac clock.** Every pose carries *both* stamps — ARKit's
`t_device_us` and the Mac's `t_arrival_us` — so this offset is directly
measurable and needs no motion at all. The only nuisance is transport latency,
which is variable but bounded below, so the lower envelope of
`arrival - device` gives the offset and its slope gives clock skew.

**B. Lidar vs poses.** The C1 emits no timestamps, so its revolutions carry only
a Mac arrival stamp taken after serial buffering and revolution assembly. That
fixed lag has to be recovered from the *motion itself*: rotate the rig, and both
sensors see the same angular-velocity signal at different apparent times.

Solving A first makes B better conditioned — it converts pose times into the Mac
clock without inheriting per-message arrival jitter.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

# Enough rotation that the correlation has something to lock onto. Below this
# the recording simply doesn't contain the information, and the honest answer
# is to say so rather than return a confident-looking number from noise.
MIN_MOTION_DPS = 15.0

# Raised from 0.5 on evidence. Injecting known delays into real recordings and
# measuring recovery showed correlation predicts accuracy sharply:
#
#   corr 0.88  ->  ±0.1 ms   across ±40 ms of injected delay
#   corr 0.82  ->  -5.3 ms   systematic
#   corr 0.78  ->  -5.1 ms   systematic
#
# Below ~0.85 the estimate carries several ms of bias, which is most of Phase 3's
# error budget. Better to reject the recording than to average bias into the
# answer.
MIN_CORRELATION = 0.85

# The method assumes rotation about the vertical. Pitch/roll doesn't shift the
# range profile the same way, so it adds signal the lidar can't match — it
# degrades correlation rather than helping.
MAX_TILT_RATIO = 0.5
# ~20 s at 10 Hz. Restarted or aborted captures leave short fragments behind,
# and a couple of seconds of data can still produce a high correlation purely by
# chance — one 22-revolution stub returned a confident-looking 115 ms.
MIN_REVOLUTIONS = 200

# Bare crystal drift is tens of ppm. Measured here: about -387 ppm, reproducible
# to under 1 ppm across sessions hours apart, and unchanged with the camera
# stream disabled. That rules out both crystal drift and a latency trend from a
# backing-up send queue.
#
# The likely source is that ARKit stamps frames on the camera capture clock,
# which is a different oscillator from the CPU timebase the Mac is compared
# against. Whatever the cause, it is real, stable, and large: 387 ppm is 116 ms
# of accumulated error over a five-minute walkthrough, so it has to be modelled
# rather than ignored. `ClockFit.to_mac()` applies it.
LARGE_SKEW_PPM = 100.0

LAG_SEARCH_US = 500_000  # ±0.5 s
GRID_HZ = 500.0

# The C1 takes 2–3 s to reach speed. While it's ramping, the revolution period
# is still changing, so dividing yaw-per-revolution by a stale dt yields a bogus
# angular velocity. Discard that window before correlating.
SPIN_UP_SKIP_S = 4.0

# Resync detection. The ARKit and Mac clocks diverge at a steady ~380 ppm and
# are then snapped back by the accumulated amount — measured at 16.30 ms
# (sd 0.53, n=9), which at 380 ppm implies a ~43 s period. Ordinary latency
# wobble is a few ms, so 8 ms separates the two cleanly.
JUMP_THRESHOLD_US = 8_000.0
MIN_SEG_POINTS = 12


@dataclass
class ClockSegment:
    """One run between resynchronisations, over which the mapping is linear."""

    t0_us: float
    t1_us: float
    intercept_us: float  # value of (arrival - device) at t0
    slope: float         # µs per µs; ×1e6 gives ppm


@dataclass
class ClockFit:
    segments: list[ClockSegment]
    skew_ppm: float         # pooled within-segment rate difference
    jump_count: int
    jump_median_us: float
    latency_p50_us: float   # median transport latency above the floor
    latency_p95_us: float
    n: int
    duration_s: float = 0.0
    skew_uncertainty_ppm: float = float("inf")

    @property
    def offset_us(self) -> float:
        """Intercept of the most recent segment. Only meaningful within this
        session — see the note in the README about why comparing it across
        sessions is not a thing."""
        return self.segments[-1].intercept_us if self.segments else 0.0

    @property
    def skew_large(self) -> bool:
        return abs(self.skew_ppm) > LARGE_SKEW_PPM

    @property
    def skew_reliable(self) -> bool:
        return self.skew_uncertainty_ppm < abs(self.skew_ppm) / 2.0

    def to_mac(self, t_device_us: np.ndarray) -> np.ndarray:
        """Convert ARKit timestamps into the Mac clock.

        Piecewise, because the two clocks are periodically resynchronised: they
        diverge at a steady rate and then get snapped back, so a single line
        fitted across a resync is wrong on both sides of it.
        """
        t = np.asarray(t_device_us, dtype=np.float64)
        if not self.segments:
            return t.copy()
        starts = np.array([s.t0_us for s in self.segments])
        idx = np.clip(np.searchsorted(starts, t, side="right") - 1, 0, len(self.segments) - 1)
        out = np.empty_like(t)
        for i, seg in enumerate(self.segments):
            m = idx == i
            if m.any():
                out[m] = t[m] + seg.intercept_us + seg.slope * (t[m] - seg.t0_us)
        return out


@dataclass
class LidarAlign:
    lag_us: float           # lidar arrival stamps trail the true capture by this
    correlation: float      # peak normalised correlation, 0..1
    sign: int               # lidar yaw direction relative to ARKit yaw
    motion_rms_dps: float
    n_revolutions: int
    tilt_rms_dps: float = 0.0

    @property
    def tilt_ratio(self) -> float:
        return self.tilt_rms_dps / self.motion_rms_dps if self.motion_rms_dps > 0 else 0.0

    @property
    def confident(self) -> bool:
        return (
            self.correlation >= MIN_CORRELATION
            and self.motion_rms_dps >= MIN_MOTION_DPS
            and self.n_revolutions >= MIN_REVOLUTIONS
            and self.tilt_ratio <= MAX_TILT_RATIO
        )

    @property
    def why_not(self) -> str:
        if self.n_revolutions < MIN_REVOLUTIONS:
            return f"only {self.n_revolutions} revolutions (need {MIN_REVOLUTIONS})"
        if self.motion_rms_dps < MIN_MOTION_DPS:
            return f"not enough rotation ({self.motion_rms_dps:.1f} deg/s)"
        if self.tilt_ratio > MAX_TILT_RATIO:
            return (f"too much pitch/roll ({self.tilt_rms_dps:.0f} deg/s vs "
                    f"{self.motion_rms_dps:.0f} yaw) — keep the rig level")
        if self.correlation < MIN_CORRELATION:
            return (f"weak correlation {self.correlation:.2f} (need "
                    f"{MIN_CORRELATION:.2f}) — rotate more smoothly")
        return ""


# --------------------------------------------------------------------------
# A. clock offset
# --------------------------------------------------------------------------

def _theil_sen(x: np.ndarray, y: np.ndarray, max_points: int = 800):
    """Median of all pairwise slopes.

    Least squares was the wrong tool here. A recording can contain a localized
    latency excursion — a few seconds where the floor wanders — and squared
    error lets that stretch dominate the whole fit: the middle third of one
    rotation capture pulled the slope from -370 ppm to +748 ppm while the
    surrounding data was perfectly well behaved.

    Theil-Sen tolerates up to ~29% contamination, so a bad stretch is outvoted
    instead of amplified.
    """
    if len(x) > max_points:  # keeps the pairwise expansion bounded
        idx = np.linspace(0, len(x) - 1, max_points).astype(int)
        x, y = x[idx], y[idx]

    i, j = np.triu_indices(len(x), k=1)
    dx = x[j] - x[i]
    ok = dx != 0
    slopes = (y[j] - y[i])[ok] / dx[ok]
    if not len(slopes):
        return 0.0, float(np.median(y)), float("inf")

    slope = float(np.median(slopes))
    intercept = float(np.median(y - slope * x))
    # Spread of the pairwise slopes, as an honest stand-in for uncertainty.
    iqr = float(np.percentile(slopes, 75) - np.percentile(slopes, 25))
    return slope, intercept, iqr / 2.0 / np.sqrt(len(x))


def fit_clock(t_device_us: np.ndarray, t_arrival_us: np.ndarray,
              window_s: float = 0.5) -> ClockFit:
    """Fit the lower envelope of (arrival - device).

    Latency is a positive-only perturbation: a message can be delayed but never
    arrive early. So the *minimum* of the difference tracks the true offset, and
    taking a mean here would bake in the average scheduling delay instead.
    """
    d = t_arrival_us.astype(np.float64) - t_device_us.astype(np.float64)
    t = t_device_us.astype(np.float64)

    # Per-window minima form the latency envelope.
    win = window_s * 1e6
    bins = ((t - t[0]) // win).astype(np.int64)
    env_t, env_d = [], []
    for b in np.unique(bins):
        m = bins == b
        i = np.argmin(d[m])
        env_t.append(t[m][i])
        env_d.append(d[m][i])
    env_t = np.asarray(env_t, dtype=np.float64)
    env_d = np.asarray(env_d, dtype=np.float64)

    # Split at resynchronisation events: the envelope falls steadily and then
    # steps back up by the accumulated divergence. Anything below the threshold
    # is ordinary latency wobble.
    jump_idx = np.where(np.diff(env_d) > JUMP_THRESHOLD_US)[0] if len(env_d) > 1 else np.array([], int)
    bounds = [0, *(jump_idx + 1), len(env_t)]
    pieces = [(bounds[i], bounds[i + 1]) for i in range(len(bounds) - 1)]

    slopes, segments = [], []
    for lo, hi in pieces:
        st, sd = env_t[lo:hi], env_d[lo:hi]
        if len(st) >= MIN_SEG_POINTS:
            s, b0, spread = _theil_sen(st - st[0], sd)
            slopes.append((s, spread, len(st)))
        else:
            s, b0 = 0.0, float(np.median(sd)) if len(sd) else 0.0
        segments.append(ClockSegment(float(st[0]), float(st[-1]), float(b0), float(s)))

    if slopes:
        # Pool by the longest segment's estimate; short pieces are noisy.
        slopes.sort(key=lambda x: -x[2])
        pooled = float(np.median([s for s, _, _ in slopes]))
        unc = slopes[0][1] * 1e6
    else:
        pooled, unc = 0.0, float("inf")

    # Re-slope every segment with the pooled rate: the physical divergence rate
    # is one number for the session, and short segments can't measure it.
    for seg in segments:
        seg.slope = pooled

    fit = ClockFit(
        segments=segments,
        skew_ppm=float(pooled * 1e6),
        jump_count=int(len(jump_idx)),
        jump_median_us=float(np.median(np.diff(env_d)[jump_idx])) if len(jump_idx) else 0.0,
        latency_p50_us=0.0,
        latency_p95_us=0.0,
        n=int(len(d)),
        duration_s=float((t[-1] - t[0]) / 1e6) if len(t) > 1 else 0.0,
        skew_uncertainty_ppm=float(unc),
    )

    excess = d - (fit.to_mac(t) - t)
    fit.latency_p50_us = float(np.percentile(excess, 50))
    fit.latency_p95_us = float(np.percentile(excess, 95))
    return fit


# --------------------------------------------------------------------------
# B. lidar alignment
# --------------------------------------------------------------------------

def yaw_rate_from_poses(t_us: np.ndarray, quats: np.ndarray):
    """Signed yaw rate about the world vertical, plus off-axis tilt rate (deg/s).

    ARKit runs with `.gravity` alignment, so world +Y is up and the Y component
    of the relative rotation vector is the yaw increment. X and Z are pitch/roll,
    returned separately as a technique diagnostic.
    """
    q = quats / np.linalg.norm(quats, axis=1, keepdims=True)
    q1, q2 = q[:-1], q[1:]

    # dq = q2 * conj(q1), the world-frame rotation from one pose to the next.
    x1, y1, z1, w1 = q1[:, 0], q1[:, 1], q1[:, 2], q1[:, 3]
    x2, y2, z2, w2 = q2[:, 0], q2[:, 1], q2[:, 2], q2[:, 3]
    dw = w2 * w1 + x2 * x1 + y2 * y1 + z2 * z1
    dx = w2 * x1 - x2 * w1 - y2 * z1 + z2 * y1
    dy = -w2 * y1 + y2 * w1 + z2 * x1 - x2 * z1
    dz = -w2 * z1 + z2 * w1 + x2 * y1 - y2 * x1

    # Small-angle: rotation vector ≈ 2 * vector part, sign-corrected by dw so the
    # shorter arc is always taken.
    s = np.sign(np.where(dw == 0, 1.0, dw))
    dt = np.diff(t_us) / 1e6
    good = dt > 0

    rate = np.zeros_like(dt)
    tilt = np.zeros_like(dt)
    rate[good] = np.degrees(2.0 * dy[good] * s[good]) / dt[good]
    tilt[good] = np.degrees(2.0 * np.hypot(dx, dz)[good]) / dt[good]
    return (t_us[:-1] + t_us[1:]) / 2.0, rate, tilt


def range_profile(samples, nbins: int = 720) -> np.ndarray:
    """Mean range per angular bin. Empty bins become NaN."""
    total = np.zeros(nbins)
    count = np.zeros(nbins)
    for angle, dist, _q in samples:
        if dist <= 0:
            continue
        b = int(angle / 360.0 * nbins) % nbins
        total[b] += dist
        count[b] += 1
    prof = np.full(nbins, np.nan)
    hit = count > 0
    prof[hit] = total[hit] / count[hit]
    return prof


def _prep(prof: np.ndarray) -> np.ndarray:
    """Fill gaps with the profile mean and zero-centre, so missing bins neither
    dominate the correlation nor bias it."""
    p = prof.copy()
    m = np.nanmean(p)
    if not np.isfinite(m):
        return np.zeros_like(p)
    p[~np.isfinite(p)] = m
    p -= p.mean()
    n = np.linalg.norm(p)
    return p / n if n > 0 else p


def yaw_delta_between(a: np.ndarray, b: np.ndarray, nbins: int = 720) -> float:
    """Yaw change (deg) between two revolutions, by circular cross-correlation.

    A planar scan rotates rigidly with the sensor, so the whole range profile
    shifts by the yaw angle. Correlating profiles recovers that shift directly —
    far cheaper than ICP and adequate for a deliberate calibration rotation.
    """
    fa, fb = np.fft.rfft(_prep(a)), np.fft.rfft(_prep(b))
    corr = np.fft.irfft(fb * np.conj(fa), n=nbins)
    k = int(np.argmax(corr))
    if k > nbins // 2:
        k -= nbins  # wrap to signed shift
    return k * 360.0 / nbins


def align_lidar(pose_t_us: np.ndarray, pose_quat: np.ndarray,
                rev_t_us: np.ndarray, rev_samples: list,
                skip_spin_up_s: float = SPIN_UP_SKIP_S) -> LidarAlign:
    # Drop the spin-up window. Until the motor reaches speed the revolution
    # period is still changing, so yaw-per-revolution divided by a wrong dt
    # gives a wrong angular velocity — which drags the correlation peak off.
    if len(rev_t_us) and skip_spin_up_s > 0:
        keep = rev_t_us >= rev_t_us[0] + skip_spin_up_s * 1e6
        rev_t_us = rev_t_us[keep]
        rev_samples = [s for s, k in zip(rev_samples, keep) if k]

    # ARKit yaw rate
    pt, prate, ptilt = yaw_rate_from_poses(pose_t_us, pose_quat)

    # Lidar yaw rate from consecutive profile correlation
    profiles = [range_profile(s) for s in rev_samples]
    lt, lrate = [], []
    for i in range(1, len(profiles)):
        dt = (rev_t_us[i] - rev_t_us[i - 1]) / 1e6
        if dt <= 0:
            continue
        lt.append((rev_t_us[i] + rev_t_us[i - 1]) / 2.0)
        lrate.append(yaw_delta_between(profiles[i - 1], profiles[i]) / dt)
    lt, lrate = np.asarray(lt, float), np.asarray(lrate, float)

    motion = float(np.sqrt(np.mean(lrate**2))) if len(lrate) else 0.0
    tilt = float(np.sqrt(np.mean(ptilt**2))) if len(ptilt) else 0.0
    if len(lt) < 8 or len(pt) < 8:
        return LidarAlign(0.0, 0.0, 1, motion, len(profiles), tilt)

    # Resample both onto a common uniform grid over the overlapping window.
    t0 = max(pt[0], lt[0])
    t1 = min(pt[-1], lt[-1])
    if t1 - t0 < 1e6:  # under a second of overlap
        return LidarAlign(0.0, 0.0, 1, motion, len(profiles), tilt)
    grid = np.arange(t0, t1, 1e6 / GRID_HZ)
    a = np.interp(grid, pt, prate)
    b = np.interp(grid, lt, lrate)

    def norm(v):
        v = v - v.mean()
        s = np.linalg.norm(v)
        return v / s if s > 0 else v

    a, b = norm(a), norm(b)

    max_lag = int(LAG_SEARCH_US / 1e6 * GRID_HZ)
    # Zero-padded FFT correlation: circular wrap would let one end of the
    # recording correlate against the other, which is meaningless here.
    nfft = 1 << int(np.ceil(np.log2(2 * len(a))))
    fa = np.fft.rfft(a, nfft)

    # The lidar's rotation sense relative to ARKit's isn't known a priori — it
    # depends on mounting. Try both and report which fits; that answer feeds
    # straight into Phase 3.
    best = (-1.0, 0.0, 1)
    for sign in (1, -1):
        c = np.fft.irfft(fa * np.conj(np.fft.rfft(sign * b, nfft)), nfft)
        window = np.concatenate([c[-max_lag:], c[: max_lag + 1]])
        k = int(np.argmax(window))
        peak = float(window[k])
        # Parabolic interpolation across the peak, so the answer isn't pinned to
        # the grid step (5 ms at 200 Hz would swamp the effect being measured).
        if 0 < k < len(window) - 1:
            y0, y1, y2 = window[k - 1], window[k], window[k + 1]
            denom = y0 - 2 * y1 + y2
            delta = 0.5 * (y0 - y2) / denom if denom != 0 else 0.0
        else:
            delta = 0.0
        # Negated: irfft(A · conj(B)) peaks at −τ for a signal delayed by τ.
        # Verified against a synthetic stream with a known +40 ms delay, which
        # this returns as +40 ms. Phase 4 must apply the same sign convention.
        lag_samples = -((k - max_lag) + float(np.clip(delta, -1, 1)))
        if peak > best[0]:
            best = (peak, lag_samples, sign)

    corr, lag_samples, sign = best
    return LidarAlign(
        lag_us=float(lag_samples / GRID_HZ * 1e6),
        correlation=float(corr),
        sign=sign,
        motion_rms_dps=motion,
        n_revolutions=len(profiles),
        tilt_rms_dps=tilt,
    )
