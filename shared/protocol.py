"""Wire format codec. Mirrors ios/Sources/WireFormat.swift — change both together.

Spec lives in docs/WIRE_FORMAT.md.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum

PROTOCOL_VERSION = 1

HEADER = struct.Struct("<IB")  # length (payload only), type


class MsgType(IntEnum):
    HELLO = 0x01
    POSE = 0x02
    CAMERA_FRAME = 0x03
    SCENE_DEPTH = 0x04
    LIDAR_REVOLUTION = 0x10


class TrackingState(IntEnum):
    UNAVAILABLE = 0
    LIMITED = 1
    NORMAL = 2


class ProtocolError(Exception):
    pass


# Payload layouts
_HELLO_HEAD = struct.Struct("<H")
_POSE = struct.Struct("<Q3f4f4fB")
_FRAME_HEAD = struct.Struct("<QHHI")


@dataclass
class Hello:
    protocol_version: int
    device_name: str
    os_version: str
    capabilities: int

    CAP_SCENE_RECONSTRUCTION = 1 << 0
    CAP_SCENE_DEPTH = 1 << 1
    CAP_CAMERA_FRAMES = 1 << 2

    def capability_names(self) -> list[str]:
        names = []
        for bit, label in (
            (self.CAP_SCENE_RECONSTRUCTION, "scene-reconstruction"),
            (self.CAP_SCENE_DEPTH, "scene-depth"),
            (self.CAP_CAMERA_FRAMES, "camera-frames"),
        ):
            if self.capabilities & bit:
                names.append(label)
        return names


@dataclass
class Pose:
    t_device_us: int
    position: tuple[float, float, float]
    quaternion: tuple[float, float, float, float]  # x, y, z, w
    intrinsics: tuple[float, float, float, float]  # fx, fy, cx, cy
    tracking_state: int
    # Stamped by the Mac on arrival. Phase 2 solves for the offset between this
    # and t_device_us; until then this is the only clock we can trust.
    t_arrival_us: int = 0

    @property
    def tracking_label(self) -> str:
        try:
            return TrackingState(self.tracking_state).name.lower()
        except ValueError:
            return f"unknown({self.tracking_state})"


@dataclass
class CameraFrame:
    t_device_us: int
    width: int
    height: int
    jpeg: bytes
    t_arrival_us: int = 0


@dataclass
class UnknownMessage:
    """A type this build doesn't understand. Skipped, not fatal — that's what
    makes adding message types a non-breaking change."""

    type_id: int
    payload: bytes


def _read_pstring(buf: bytes, offset: int) -> tuple[str, int]:
    (n,) = struct.unpack_from("<H", buf, offset)
    offset += 2
    s = buf[offset : offset + n].decode("utf-8", errors="replace")
    return s, offset + n


def decode(type_id: int, payload: bytes):
    """Decode one payload into a message object."""
    if type_id == MsgType.HELLO:
        (version,) = _HELLO_HEAD.unpack_from(payload, 0)
        name, off = _read_pstring(payload, 2)
        os_version, off = _read_pstring(payload, off)
        (caps,) = struct.unpack_from("<I", payload, off)
        return Hello(version, name, os_version, caps)

    if type_id == MsgType.POSE:
        if len(payload) != _POSE.size:
            raise ProtocolError(f"POSE payload is {len(payload)}B, expected {_POSE.size}")
        f = _POSE.unpack(payload)
        return Pose(
            t_device_us=f[0],
            position=(f[1], f[2], f[3]),
            quaternion=(f[4], f[5], f[6], f[7]),
            intrinsics=(f[8], f[9], f[10], f[11]),
            tracking_state=f[12],
        )

    if type_id == MsgType.CAMERA_FRAME:
        t, w, h, n = _FRAME_HEAD.unpack_from(payload, 0)
        jpeg = payload[_FRAME_HEAD.size : _FRAME_HEAD.size + n]
        return CameraFrame(t, w, h, jpeg)

    return UnknownMessage(type_id, payload)


def encode(type_id: int, payload: bytes) -> bytes:
    """Frame a payload for sending. Only used by tests today — the Mac is
    receive-only in v0 — but keeping it here keeps the codec symmetric."""
    return HEADER.pack(len(payload), type_id) + payload


def encode_pose(p: Pose) -> bytes:
    return encode(
        MsgType.POSE,
        _POSE.pack(
            p.t_device_us, *p.position, *p.quaternion, *p.intrinsics, p.tracking_state
        ),
    )


def encode_hello(h: Hello) -> bytes:
    name = h.device_name.encode()
    os_version = h.os_version.encode()
    payload = (
        _HELLO_HEAD.pack(h.protocol_version)
        + struct.pack("<H", len(name))
        + name
        + struct.pack("<H", len(os_version))
        + os_version
        + struct.pack("<I", h.capabilities)
    )
    return encode(MsgType.HELLO, payload)
