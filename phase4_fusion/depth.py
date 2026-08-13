"""Unproject ARKit depth frames into the world, using the fused trajectory.

The C1 and the phone are complementary and this is where that pays off:

  * the C1 is a *planar* scanner — it only produces 3D because the rig sweeps,
    so coverage depends on how you moved and can never include a surface you
    never swept across;
  * ARKit's LiDAR returns a dense 256x192 depth image of whatever the camera is
    pointed at, out to roughly 5 m, with no sweeping required.

So the C1 supplies long-range structure and the drift constraints, and the phone
supplies dense local surface. Both are placed with the same pose-graph-corrected
trajectory, which is what makes them consistent with each other.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

# ARKit marks each depth pixel low/medium/high. Low-confidence returns cluster on
# edges and dark or glossy surfaces and are frequently metres wrong, so they are
# dropped rather than averaged in.
MIN_CONFIDENCE = 1

# Beyond its useful range the phone's depth degrades badly; below this it is
# reading the operator or the rig itself.
MIN_DEPTH_M = 0.15
MAX_DEPTH_M = 5.0

# Depth pixels arrive at 5 Hz x 49k = 245k points/s, far more than needed for a
# map. Taking every Nth pixel keeps clouds tractable without losing structure.
PIXEL_STRIDE = 2


@dataclass
class DepthFrame:
    t_device_us: int
    t_arrival_us: int
    width: int
    height: int
    intrinsics: tuple[float, float, float, float]  # already scaled to width×height
    depth_m: np.ndarray      # H×W float32, 0 = no return
    confidence: np.ndarray   # H×W uint8


def unproject(frame: DepthFrame, stride: int = PIXEL_STRIDE) -> np.ndarray:
    """Depth image -> Nx3 points in the ARKit **camera** frame.

    ARKit's camera looks along -Z, so a depth of d metres puts the point at
    z = -d. Getting that sign wrong mirrors the whole cloud behind the camera,
    which still looks like a plausible room from some angles.
    """
    fx, fy, cx, cy = frame.intrinsics
    d = frame.depth_m[::stride, ::stride]
    c = frame.confidence[::stride, ::stride]

    h, w = d.shape
    v, u = np.mgrid[0:h, 0:w]
    u = u * stride
    v = v * stride

    ok = (d >= MIN_DEPTH_M) & (d <= MAX_DEPTH_M) & (c >= MIN_CONFIDENCE)
    if not ok.any():
        return np.empty((0, 3))

    d = d[ok]
    u = u[ok].astype(np.float64)
    v = v[ok].astype(np.float64)

    x = (u - cx) / fx * d
    y = (v - cy) / fy * d
    # Image +v runs downward while the camera's +Y runs up, hence the negation
    # on y as well as z.
    return np.stack([x, -y, -d], axis=-1)


def depth_to_world(frames: list[DepthFrame], traj, clock_to_mac) -> np.ndarray:
    """Place every depth frame in the world using the corrected trajectory.

    Depth carries ARKit's own timestamp, so it maps through the same clock model
    as poses — no lidar lag applies here, that offset belongs to the C1's serial
    path alone. Mixing the two up would shift the dense cloud against the sparse
    one by ~7 ms of motion.
    """
    lo, hi = traj.span_us
    out = []
    for f in frames:
        t = float(clock_to_mac(np.array([f.t_device_us]))[0])
        if not (lo <= t <= hi):
            continue
        pts = unproject(f)
        if len(pts) == 0:
            continue
        pos, rot = traj.at(np.array([t]))
        out.append(pts @ rot[0].T + pos[0])
    return np.concatenate(out) if out else np.empty((0, 3))


def load_depth_frames(reader, limit: int | None = None) -> list[DepthFrame]:
    """Read the depth stream out of a session."""
    from protocol import MsgType, decode  # shared module

    frames = []
    for rec in reader.depth():
        msg = decode(MsgType.SCENE_DEPTH, rec.payload)
        d, c = msg.as_arrays()
        frames.append(DepthFrame(
            t_device_us=msg.t_device_us,
            t_arrival_us=rec.t_arrival_us,
            width=msg.width, height=msg.height,
            intrinsics=msg.intrinsics,
            depth_m=d, confidence=c,
        ))
        if limit and len(frames) >= limit:
            break
    return frames
