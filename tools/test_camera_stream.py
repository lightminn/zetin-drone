import datetime
import sys
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from camera_stream import (  # noqa: E402
    MjpegParser,
    StreamFormatError,
    StreamPart,
    boundary_from_content_type,
    format_recv_time,
    summarize,
)


BOUNDARY = b"zetincamframe"
JPEG = b"\xff\xd8abc\xff\xd9"


def make_part(jpeg, ts="3000.000123", seq=7, headers=None):
    """Bytes for one part exactly as the camera sketch sends it."""
    if headers is None:
        headers = [b"Content-Type: image/jpeg", b"Content-Length: %d" % len(jpeg)]
        if ts is not None:
            headers.append(b"X-Timestamp: " + ts.encode())
        if seq is not None:
            headers.append(b"X-Frame-Seq: %d" % seq)
    return b"\r\n--" + BOUNDARY + b"\r\n" + b"\r\n".join(headers) + b"\r\n\r\n" + jpeg


def make_stream(parts):
    return b"".join(make_part(*p) if isinstance(p, tuple) else make_part(p) for p in parts)


def row(segment, seq, mono, epoch=None, capture=None, frame=0):
    return {
        "frame": str(frame),
        "segment": str(segment),
        "camera_seq": "" if seq is None else str(seq),
        "offset": "0",
        "size": "1",
        "capture_s": "" if capture is None else repr(capture),
        "recv_epoch_s": repr(mono if epoch is None else epoch),
        "recv_monotonic_s": repr(mono),
        "recv_time": "",
    }


class MjpegParserTest(unittest.TestCase):
    def test_single_part(self):
        parts = MjpegParser(BOUNDARY).feed(make_part(JPEG))
        self.assertEqual(parts, [StreamPart(JPEG, 3000.000123, 7)])

    def test_split_at_every_byte(self):
        jpegs = [JPEG, b"\xff\xd8" + b"x" * 300 + b"\xff\xd9", b"\xff\xd8\xff\xd9"]
        data = make_stream([(j, f"{i}.000001", i) for i, j in enumerate(jpegs)])
        parser = MjpegParser(BOUNDARY)
        out = []
        for i in range(len(data)):
            out.extend(parser.feed(data[i : i + 1]))
        self.assertEqual([p.jpeg for p in out], jpegs)
        self.assertEqual([p.seq for p in out], [0, 1, 2])

    def test_payload_containing_boundary(self):
        tricky = b"\xff\xd8" + b"\r\n--zetincamframe\r\n" + b"\xff\xd9"
        parts = MjpegParser(BOUNDARY).feed(make_stream([tricky, JPEG]))
        self.assertEqual([p.jpeg for p in parts], [tricky, JPEG])

    def test_missing_content_length_raises(self):
        data = make_part(JPEG, headers=[b"Content-Type: image/jpeg"])
        with self.assertRaises(StreamFormatError):
            MjpegParser(BOUNDARY).feed(data)

    def test_non_numeric_content_length_raises(self):
        data = make_part(JPEG, headers=[b"Content-Length: abc"])
        with self.assertRaises(StreamFormatError):
            MjpegParser(BOUNDARY).feed(data)

    def test_missing_optional_headers_are_none(self):
        parts = MjpegParser(BOUNDARY).feed(make_part(JPEG, ts=None, seq=None))
        self.assertEqual(parts, [StreamPart(JPEG, None, None)])

    def test_header_names_case_insensitive(self):
        data = make_part(
            JPEG,
            headers=[b"content-length: %d" % len(JPEG), b"x-timestamp: 1.5", b"x-frame-seq: 3"],
        )
        self.assertEqual(MjpegParser(BOUNDARY).feed(data), [StreamPart(JPEG, 1.5, 3)])

    def test_garbage_before_first_boundary_is_dropped(self):
        parts = MjpegParser(BOUNDARY).feed(b"HTTP noise" + make_part(JPEG))
        self.assertEqual([p.jpeg for p in parts], [JPEG])

    def test_boundary_from_content_type(self):
        self.assertEqual(
            boundary_from_content_type("multipart/x-mixed-replace;boundary=zetincamframe"), BOUNDARY
        )
        self.assertEqual(
            boundary_from_content_type('multipart/x-mixed-replace; boundary="zetincamframe"'), BOUNDARY
        )
        with self.assertRaises(StreamFormatError):
            boundary_from_content_type("image/jpeg")

    def test_tag_is_header_arrival_chunk(self):
        data = make_part(b"\xff\xd8" + b"y" * 100 + b"\xff\xd9")
        cut = data.index(b"\r\n\r\n") + 4 + 10  # headers complete, body still incomplete
        parser = MjpegParser(BOUNDARY)
        self.assertEqual(parser.feed(data[:cut], tag="A"), [])
        parts = parser.feed(data[cut:], tag="B")
        self.assertEqual(len(parts), 1)
        self.assertEqual(parts[0].tag, "A")

    def test_length_mismatch_raises(self):
        body = b"\xff\xd8" + b"z" * 20 + b"\xff\xd9"
        bad = make_part(
            body, headers=[b"Content-Length: %d" % (len(body) - 2), b"X-Frame-Seq: 0"]
        )
        with self.assertRaises(StreamFormatError):
            MjpegParser(BOUNDARY).feed(bad + make_part(JPEG))

    def test_finish_reports_truncated_part(self):
        data = make_part(b"\xff\xd8" + b"q" * 50 + b"\xff\xd9")
        parser = MjpegParser(BOUNDARY)
        parser.feed(data[:-10])
        self.assertTrue(parser.finish())

        parser = MjpegParser(BOUNDARY)
        self.assertEqual(len(parser.feed(data)), 1)
        self.assertFalse(parser.finish())


class RecvTimeTest(unittest.TestCase):
    def test_format_recv_time_matches_flight_log(self):
        t = 1_793_000_000.123456
        expected = datetime.datetime.fromtimestamp(t).strftime("%H:%M:%S.%f")[:-3]
        self.assertEqual(format_recv_time(t), expected)


class SummarizeTest(unittest.TestCase):
    def test_summarize_counts_gaps_within_segment_only(self):
        rows = [row(0, 0, 0.0), row(0, 1, 0.1), row(0, 3, 0.2), row(1, 0, 1.0), row(1, 1, 1.1)]
        summary = summarize(rows)
        self.assertEqual(summary.seq_gaps, 1)
        self.assertEqual(summary.segments, 2)
        self.assertEqual(summary.frames, 5)

    def test_summarize_fps_and_longest_gap(self):
        rows = [row(0, i, t) for i, t in enumerate([0.0, 0.1, 0.2, 0.9])]
        summary = summarize(rows)
        self.assertAlmostEqual(summary.longest_gap_s, 0.7, delta=1e-9)
        self.assertAlmostEqual(summary.effective_fps, 3 / 0.9, delta=1e-9)

        two = [row(0, 0, 0.0), row(0, 1, 1.0), row(1, 0, 5.0), row(1, 1, 5.5), row(1, 2, 6.0)]
        self.assertAlmostEqual(summarize(two).effective_fps, 3 / 2.0, delta=1e-9)

    def test_summarize_reconnect_gaps(self):
        rows = [row(0, 0, 0.0), row(0, 1, 1.0), row(1, 0, 3.5), row(1, 1, 4.0)]
        summary = summarize(rows)
        self.assertEqual(len(summary.reconnect_gaps_s), 1)
        self.assertAlmostEqual(summary.reconnect_gaps_s[0], 2.5, delta=1e-9)
        self.assertAlmostEqual(summary.longest_gap_s, 1.0, delta=1e-9)

    def test_summarize_ignores_wall_clock(self):
        rows = [row(0, 0, 0.0, epoch=100.0), row(0, 1, 0.1, epoch=50.0)]
        self.assertAlmostEqual(summarize(rows).longest_gap_s, 0.1, delta=1e-9)

    def test_summarize_empty(self):
        summary = summarize([])
        self.assertEqual((summary.frames, summary.segments, summary.seq_gaps), (0, 0, 0))
        self.assertEqual(summary.effective_fps, 0.0)


if __name__ == "__main__":
    unittest.main()
