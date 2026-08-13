# Wire format

The contract between the iPhone and the Mac. Changing it means changing both
sides, so it gets its own document and a version byte.

## Topology (v0)

```
C1 ──USB serial──> MacBook <──TCP over usbmuxd── iPhone
                   (local, no wire format)       (this document)
```

The lidar is plugged straight into the Mac, so there is exactly **one network
link in v0** and only the phone stream needs a wire format. When the rig moves
to a Pi on the mast, a second link appears (Pi → Mac) carrying lidar
revolutions; `LIDAR_REVOLUTION` below is reserved for that and unused today.

## Transport

TCP, tunnelled over USB by `usbmuxd`. The **iPhone listens**, the **Mac
connects** — `iproxy` maps a Mac-local port to a port on the device, so from
the Mac's point of view the phone is at `127.0.0.1`.

```
iproxy 5555 5555        # mac localhost:5555 -> iphone:5555
```

## Framing

Every message is length-prefixed. All integers **little-endian** (both ends are
ARM64, so this is the native ordering on each).

```
┌────────────┬────────────┬──────────────────┐
│ length u32 │ type   u8  │ payload (length) │
└────────────┴────────────┴──────────────────┘
```

`length` counts the payload only — it excludes the 5-byte header. A reader that
does not recognise a `type` must skip `length` bytes and continue, which is what
makes adding message types backward-compatible.

## Message types

### `0x01 HELLO` — first message on every connection

```
protocol_version   u16     must equal PROTOCOL_VERSION
device_name_len    u16
device_name        utf8    e.g. "iPhone17,2"
os_version_len     u16
os_version         utf8
capabilities       u32     bitfield, see below
```

Capability bits: `1<<0` scene reconstruction, `1<<1` scene depth,
`1<<2` camera frames. The Mac logs these; it does not require any of them.

### `0x02 POSE` — one per ARKit frame, ~60 Hz

```
t_device_us        u64     ARKit frame timestamp, microseconds
tx, ty, tz         f32×3   camera position, ARKit world frame, metres
qx, qy, qz, qw     f32×4   camera orientation, quaternion
fx, fy, cx, cy     f32×4   camera intrinsics, pixels
tracking_state     u8      0 unavailable, 1 limited, 2 normal
```

53 bytes; ~3 KB/s at 60 Hz. Cheap enough that there is no reason to decimate it.

`t_device_us` is in ARKit's own clock (`CACurrentMediaTime`), which is **not**
the Mac's clock. Phase 0 ignores it and stamps on arrival; Phase 2 solves for
the offset. Both timestamps get recorded so Phase 2 can be done offline.

ARKit's convention: right-handed, **−Z forward**, +Y up, origin at session start.

### `0x03 CAMERA_FRAME` — JPEG, rate-limited

```
t_device_us        u64
width, height      u16×2
jpeg_len           u32
jpeg               bytes
```

The fat stream. Rate-limit on the phone rather than dropping on the Mac — the
usbmuxd tunnel is fast but not unlimited, and a blocked write stalls the pose
stream behind it.

### `0x04 SCENE_DEPTH` — ARKit's LiDAR depth image

```
t_device_us        u64
width, height      u16×2   256×192 on current hardware
fx, fy, cx, cy     f32×4   intrinsics SCALED to the depth resolution
depth_mm           u16 × width*height   0 = no return
confidence         u8  × width*height   0 low, 1 medium, 2 high
```

The dense geometry the C1 cannot provide: it is a planar scanner and only sweeps
3D because the rig moves, so its coverage is inherently sparse and streaky.

Depth is millimetres in `u16` rather than `float16`: exact to 1 mm out to 65 m,
where half-precision floats lose resolution with range. Same bytes either way.

**Intrinsics are scaled**, not the camera's native values. `ARFrame.camera.intrinsics`
describes the full-resolution capture (e.g. 1920×1440); the depth map is 256×192,
so fx, fy, cx and cy are all multiplied by the ratio before sending. Unprojecting
with unscaled intrinsics yields a plausible-looking but badly wrong cloud.

~144 KB per frame, so it is rate-limited on the phone like camera frames.

### `0x05 MESH_CHUNK` — one ARKit mesh anchor, with semantic labels

```
t_device_us        u64
anchor_id          16 bytes   stable UUID; later chunks REPLACE earlier ones
transform          f32×16     anchor→world, column-major
vertex_count       u32
face_count         u32
vertices           f32×3 × vertex_count    anchor-local
faces              u32×3 × face_count      indices into vertices
classification     u8  × face_count
```

Classification values: `0` none, `1` wall, `2` floor, `3` ceiling, `4` table,
`5` seat, `6` window, `7` door. This is the free semantic layer — ARKit produces
it on device with no model of ours, and it covers most of what room measurement
needs.

ARKit grows and revises mesh anchors continuously, so chunks are **replacements
keyed on `anchor_id`**, not increments. The Mac keeps the most recent chunk per
id. Sending only changed anchors, rate-limited, keeps this well under the depth
stream's bandwidth despite there being many anchors.

### `0x10 LIDAR_REVOLUTION` — reserved, Pi bridge only

```
t_arrival_us       u64
rev_id             u32
sample_count       u16
samples            (angle_q6 u16, dist_q2 u16, quality u8) × count
```

Samples stay in the C1's native fixed-point units — `angle_q6 / 64.0` gives
degrees, `dist_q2 / 4.0` gives millimetres. Half the bytes of floats and no
precision lost.

## Versioning

Bump `PROTOCOL_VERSION` on any change to an existing message's layout. Adding a
new message type does not require a bump, because unknown types are skipped.
