"""On-disk recording format.

One directory per session. Streams live in separate files rather than one
interleaved log because Phases 2 and 3 want poses without paying to skip past
JPEGs, and fixed-size pose records make that a seek instead of a scan.

    recordings/2026-08-03T19-14-22/
        manifest.json     metadata, counts, capabilities, git commit
        poses.bin         fixed 61-byte records
        lidar.bin         variable-length revolutions
        frames.bin        length-prefixed JPEGs

Payloads are stored **verbatim as they arrived on the wire**, with an arrival
timestamp prepended. Nothing is re-encoded on the way to disk, so a recording
cannot drift from what the sensor actually sent — and replay is a read, not a
reconstruction.
"""

from __future__ import annotations

import json
import struct
import subprocess
from dataclasses import dataclass
from enum import IntEnum
from pathlib import Path
from typing import Iterator

FORMAT_VERSION = 1

# arrival(u64) + the 53-byte POSE payload, verbatim off the wire
POSE_REC = struct.Struct("<Q53s")
LIDAR_HEAD = struct.Struct("<QIH")     # arrival, rev_id, sample_count
LIDAR_SAMPLE = struct.Struct("<HHB")   # angle_q6, dist_q2, quality
FRAME_HEAD = struct.Struct("<QQHHI")   # arrival, device_ts, w, h, jpeg_len


class Stream(IntEnum):
    """Ordering here is also the tiebreak when two records share an arrival
    timestamp, which is what makes the merge in `records()` deterministic."""

    POSE = 0
    LIDAR = 1
    FRAME = 2


@dataclass(frozen=True)
class Record:
    stream: Stream
    t_arrival_us: int
    payload: bytes  # exactly the bytes that were on the wire


def _git_commit() -> str | None:
    try:
        out = subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"],
            capture_output=True, text=True, timeout=5,
        )
        return out.stdout.strip() or None
    except Exception:
        return None


class SessionWriter:
    def __init__(self, directory: Path, notes: str = ""):
        self.dir = Path(directory)
        self.dir.mkdir(parents=True, exist_ok=True)
        self._poses = open(self.dir / "poses.bin", "wb")
        self._lidar = open(self.dir / "lidar.bin", "wb")
        self._frames = open(self.dir / "frames.bin", "wb")
        self.counts = {"poses": 0, "lidar": 0, "frames": 0, "lidar_samples": 0}
        self.meta: dict = {
            "format_version": FORMAT_VERSION,
            "notes": notes,
            "git_commit": _git_commit(),
        }

    def __enter__(self) -> "SessionWriter":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def set_device(self, name: str, os_version: str, capabilities: list[str]) -> None:
        self.meta["device"] = {"name": name, "os": os_version, "capabilities": capabilities}

    def write_pose(self, t_arrival_us: int, payload: bytes) -> None:
        if len(payload) != 53:
            raise ValueError(f"pose payload is {len(payload)}B, expected 53")
        self._poses.write(POSE_REC.pack(t_arrival_us, payload))
        self.counts["poses"] += 1

    def write_lidar(self, t_arrival_us: int, rev_id: int, samples) -> None:
        """samples: iterable of Measurement, written in the C1's native
        fixed-point units so nothing is lost to float conversion."""
        body = bytearray()
        n = 0
        for s in samples:
            body += LIDAR_SAMPLE.pack(
                int(round(s.angle_deg * 64.0)) & 0xFFFF,
                int(round(s.distance_mm * 4.0)) & 0xFFFF,
                s.quality & 0xFF,
            )
            n += 1
        self._lidar.write(LIDAR_HEAD.pack(t_arrival_us, rev_id, n) + bytes(body))
        self.counts["lidar"] += 1
        self.counts["lidar_samples"] += n

    def write_frame(self, t_arrival_us: int, t_device_us: int, w: int, h: int, jpeg: bytes) -> None:
        self._frames.write(FRAME_HEAD.pack(t_arrival_us, t_device_us, w, h, len(jpeg)) + jpeg)
        self.counts["frames"] += 1

    def close(self) -> None:
        for f in (self._poses, self._lidar, self._frames):
            f.close()
        self.meta["counts"] = self.counts
        # Manifest is written last so its presence means the session is complete.
        (self.dir / "manifest.json").write_text(json.dumps(self.meta, indent=2) + "\n")


class SessionReader:
    def __init__(self, directory: Path):
        self.dir = Path(directory)
        manifest = self.dir / "manifest.json"
        if not manifest.exists():
            raise FileNotFoundError(
                f"{self.dir} has no manifest.json — recording was interrupted, "
                "or this isn't a session directory"
            )
        self.meta = json.loads(manifest.read_text())
        if self.meta.get("format_version") != FORMAT_VERSION:
            raise ValueError(
                f"session is format v{self.meta.get('format_version')}, "
                f"this build reads v{FORMAT_VERSION}"
            )

    @property
    def counts(self) -> dict:
        return self.meta.get("counts", {})

    def poses(self) -> Iterator[Record]:
        data = (self.dir / "poses.bin").read_bytes()
        for off in range(0, len(data) - POSE_REC.size + 1, POSE_REC.size):
            t, payload = POSE_REC.unpack_from(data, off)
            yield Record(Stream.POSE, t, payload)

    def lidar(self) -> Iterator[Record]:
        data = (self.dir / "lidar.bin").read_bytes()
        off = 0
        while off + LIDAR_HEAD.size <= len(data):
            t, _rev, n = LIDAR_HEAD.unpack_from(data, off)
            end = off + LIDAR_HEAD.size + n * LIDAR_SAMPLE.size
            if end > len(data):
                break  # truncated tail from an interrupted recording
            yield Record(Stream.LIDAR, t, data[off:end])
            off = end

    def frames(self) -> Iterator[Record]:
        data = (self.dir / "frames.bin").read_bytes()
        off = 0
        while off + FRAME_HEAD.size <= len(data):
            t, _dev, _w, _h, n = FRAME_HEAD.unpack_from(data, off)
            end = off + FRAME_HEAD.size + n
            if end > len(data):
                break
            yield Record(Stream.FRAME, t, data[off:end])
            off = end

    def records(self) -> Iterator[Record]:
        """All streams merged into arrival order.

        Deterministic by construction: sorted on (arrival, stream, index), so
        equal timestamps resolve the same way on every replay. Without that
        tiebreak two replays of one file could legitimately differ in order,
        and the Phase 1 gate would be untestable.
        """
        merged = [
            (r.t_arrival_us, r.stream.value, i, r)
            for i, r in enumerate(list(self.poses()) + list(self.lidar()) + list(self.frames()))
        ]
        merged.sort(key=lambda x: (x[0], x[1], x[2]))
        for _, _, _, r in merged:
            yield r


def decode_lidar(payload: bytes) -> tuple[int, int, list[tuple[float, float, int]]]:
    """(rev_id, t_arrival_us, [(angle_deg, distance_mm, quality)])"""
    t, rev_id, n = LIDAR_HEAD.unpack_from(payload, 0)
    out = []
    off = LIDAR_HEAD.size
    for _ in range(n):
        a, d, q = LIDAR_SAMPLE.unpack_from(payload, off)
        out.append((a / 64.0, d / 4.0, q))
        off += LIDAR_SAMPLE.size
    return rev_id, t, out


def latest_session(root: Path = Path("recordings")) -> Path:
    sessions = sorted(p for p in Path(root).glob("*") if (p / "manifest.json").exists())
    if not sessions:
        raise FileNotFoundError(f"no complete sessions under {root}/")
    return sessions[-1]
