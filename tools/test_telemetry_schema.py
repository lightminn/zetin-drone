import sys
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from telemetry_schema import (  # noqa: E402
    CSV_FIELDS,
    TELEMETRY_FIELDS,
    parse_telemetry_packet,
    sample_to_csv_row,
)


def packet(field_count):
    values = [str(index + 0.5) for index in range(9)]
    values.extend(str(index) for index in range(9, field_count))
    return ",".join(values)


class TelemetryCompatibilityTest(unittest.TestCase):
    def test_10_field_packet_keeps_extended_values_unknown(self):
        sample = parse_telemetry_packet(packet(10))
        self.assertEqual(9, sample["Throttle"])
        self.assertIsNone(sample["Fault_RC"])
        self.assertEqual(len(CSV_FIELDS), len(sample_to_csv_row("00:00:00.000", sample)))

    def test_14_field_packet_populates_legacy_fault_and_rc_fields(self):
        sample = parse_telemetry_packet(packet(14))
        self.assertEqual(13, sample["RC_Dropped_Pkts"])
        self.assertIsNone(sample["Fault_IMU1"])

    def test_21_field_packet_leaves_armed_unknown(self):
        sample = parse_telemetry_packet(packet(21))
        self.assertEqual(20, sample["Calibration_OK"])
        self.assertIsNone(sample["Armed"])

    def test_22_field_packet_populates_armed(self):
        sample = parse_telemetry_packet(packet(22))
        self.assertEqual(21, sample["Armed"])
        self.assertEqual(22, len(TELEMETRY_FIELDS))
        self.assertEqual(23, len(CSV_FIELDS))

    def test_short_packet_is_rejected(self):
        with self.assertRaises(ValueError):
            parse_telemetry_packet(packet(9))

    def test_empty_required_field_is_rejected(self):
        # 필수 필드가 빈 문자열이면 None이 수신 도구의 포맷/연산을 죽이므로
        # 파서 단계에서 거부해야 한다.
        with self.assertRaises(ValueError):
            parse_telemetry_packet("1.0,2.0,3.0,4,5,6,7,8,9,")
        with self.assertRaises(ValueError):
            parse_telemetry_packet(",2.0,3.0,4,5,6,7,8,9,10")

    def test_empty_extended_field_stays_unknown(self):
        sample = parse_telemetry_packet(packet(10) + ",,1")
        self.assertIsNone(sample["Fault_RC"])
        self.assertEqual(1, sample["Fault_Critical"])


if __name__ == "__main__":
    unittest.main()
