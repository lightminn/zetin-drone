"""Shared logic for the drone camera stream: MJPEG parsing, the recording index
schema, recording summaries and camera-to-laptop clock alignment.

Standard library only. ``record_camera.py`` (recording) and ``camera_to_mp4.py``
(post-processing) build on this module; the stream format is produced by
``firmware/camera/esp32cam_stream``.
"""

from __future__ import annotations

import datetime
import re
from dataclasses import dataclass


class StreamFormatError(ValueError):
    """The byte stream does not follow the multipart/x-mixed-replace format."""


@dataclass(frozen=True)
class StreamPart:
    jpeg: bytes
    capture_s: float | None
    seq: int | None
    # Value passed to the feed() call in which this part's HEADERS completed.
    # The recorder passes the receive time here: the camera paces its sends at
    # the byte-rate cap, so the last byte of a frame arrives up to ~100 ms after
    # the header. Header arrival keeps that pacing delay out of clock alignment.
    tag: object = None


INDEX_FIELDS = (
    "frame",
    "segment",
    "camera_seq",
    "offset",
    "size",
    "capture_s",
    "recv_epoch_s",
    "recv_monotonic_s",
    "recv_time",
)

_BOUNDARY_RE = re.compile(r'boundary\s*=\s*"?([^";\s]+)"?', re.IGNORECASE)


def boundary_from_content_type(value: str) -> bytes:
    match = _BOUNDARY_RE.search(value or "")
    if match is None:
        raise StreamFormatError(f"no multipart boundary in Content-Type {value!r}")
    return match.group(1).encode("ascii")


def format_recv_time(epoch_s: float) -> str:
    """Same ``HH:MM:SS.mmm`` local-time form as the flight log ``Timestamp`` column."""
    return datetime.datetime.fromtimestamp(epoch_s).strftime("%H:%M:%S.%f")[:-3]


class MjpegParser:
    """Incremental parser for the camera's multipart stream.

    Bodies are read by ``Content-Length``, never by searching for the boundary,
    so JPEG bytes that happen to contain the boundary are kept intact. The bytes
    right after a body must be the next boundary; anything else means the length
    lied and raises ``StreamFormatError``.
    """

    _SEEK, _HEADERS, _BODY, _AFTER = range(4)

    def __init__(self, boundary: bytes):
        self._open = b"--" + boundary + b"\r\n"
        self._delim = b"\r\n" + self._open
        self._buf = bytearray()
        self._state = self._SEEK
        self._headers: dict[str, str] = {}
        self._length = 0
        self._tag: object = None

    def feed(self, data: bytes, tag: object = None) -> list[StreamPart]:
        self._buf += data
        parts: list[StreamPart] = []
        while True:
            if self._state == self._SEEK:
                at = self._buf.find(self._open)
                if at < 0:
                    # keep a tail that may be the start of a split boundary
                    keep = len(self._open) - 1
                    if len(self._buf) > keep:
                        del self._buf[: len(self._buf) - keep]
                    return parts
                del self._buf[: at + len(self._open)]
                self._state = self._HEADERS
            elif self._state == self._HEADERS:
                end = self._buf.find(b"\r\n\r\n")
                if end < 0:
                    return parts
                self._headers = _parse_headers(bytes(self._buf[:end]))
                del self._buf[: end + 4]
                self._length = _content_length(self._headers)
                self._tag = tag
                self._state = self._BODY
            elif self._state == self._BODY:
                if len(self._buf) < self._length:
                    return parts
                jpeg = bytes(self._buf[: self._length])
                del self._buf[: self._length]
                parts.append(
                    StreamPart(
                        jpeg,
                        _optional_float(self._headers.get("x-timestamp")),
                        _optional_int(self._headers.get("x-frame-seq")),
                        self._tag,
                    )
                )
                self._state = self._AFTER
            else:  # _AFTER: the next boundary must follow immediately
                have = min(len(self._buf), len(self._delim))
                if bytes(self._buf[:have]) != self._delim[:have]:
                    raise StreamFormatError("part body is not followed by the boundary (Content-Length mismatch)")
                if have < len(self._delim):
                    return parts
                del self._buf[: len(self._delim)]
                self._state = self._HEADERS

    def finish(self) -> bool:
        """Call when the connection ends. Returns True if an incomplete part was dropped."""
        truncated = self._state == self._BODY or (self._state == self._HEADERS and len(self._buf) > 0)
        self._buf.clear()
        self._state = self._SEEK
        return truncated


def _parse_headers(block: bytes) -> dict[str, str]:
    headers: dict[str, str] = {}
    for line in block.decode("latin-1").split("\r\n"):
        name, sep, value = line.partition(":")
        if sep:
            headers[name.strip().lower()] = value.strip()
    return headers


def _content_length(headers: dict[str, str]) -> int:
    raw = headers.get("content-length")
    if raw is None:
        raise StreamFormatError("part without Content-Length")
    if not raw.isdigit():
        raise StreamFormatError(f"non-numeric Content-Length {raw!r}")
    return int(raw)


def _optional_float(value):
    if value is None or value == "":
        return None
    return float(value)


def _optional_int(value):
    if value is None or value == "":
        return None
    return int(value)


@dataclass(frozen=True)
class RecordingSummary:
    frames: int
    segments: int
    seq_gaps: int
    effective_fps: float
    longest_gap_s: float  # longest gap INSIDE a connection segment
    reconnect_gaps_s: tuple[float, ...]  # last frame of segment k -> first of k+1


def _segments(rows):
    """Rows grouped by segment, in recording order."""
    groups: list[list[dict]] = []
    current = None
    for r in rows:
        seg = int(r["segment"])
        if seg != current:
            groups.append([])
            current = seg
        groups[-1].append(r)
    return groups


def summarize(rows) -> RecordingSummary:
    """Summary of a recording index. Values may be CSV strings.

    Rates and gaps use the monotonic receive clock so a wall-clock jump (NTP)
    during the recording does not distort them.
    """
    groups = _segments(rows)
    seq_gaps = 0
    intervals = 0
    span = 0.0
    longest = 0.0
    reconnects: list[float] = []
    prev_last = None
    for group in groups:
        mono = [float(r["recv_monotonic_s"]) for r in group]
        if prev_last is not None:
            reconnects.append(mono[0] - prev_last)
        prev_last = mono[-1]
        for a, b in zip(mono, mono[1:]):
            longest = max(longest, b - a)
        if len(mono) >= 2:
            intervals += len(mono) - 1
            span += mono[-1] - mono[0]
        seqs = [int(r["camera_seq"]) for r in group if r.get("camera_seq", "") != ""]
        for a, b in zip(seqs, seqs[1:]):
            seq_gaps += max(0, b - a - 1)
    return RecordingSummary(
        frames=len(rows),
        segments=len(groups),
        seq_gaps=seq_gaps,
        effective_fps=intervals / span if span > 0 else 0.0,
        longest_gap_s=longest,
        reconnect_gaps_s=tuple(reconnects),
    )
