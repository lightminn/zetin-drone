import sys
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "scripts"))

import telemetry_schema  # noqa: E402
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


GAIN_NAMES = (
    "Kp_Angle_Roll",
    "Kp_Angle_Pitch",
    "Kp_Angle_Yaw",
    "Kp_Rate_Roll",
    "Kp_Rate_Pitch",
    "Kp_Rate_Yaw",
    "Ki_Rate_Roll",
    "Ki_Rate_Pitch",
    "Ki_Rate_Yaw",
    "Kd_Rate_Roll",
    "Kd_Rate_Pitch",
    "Kd_Rate_Yaw",
)


class TelemetryCompatibilityTest(unittest.TestCase):
    def test_10_field_packet_keeps_extended_values_unknown(self):
        sample = parse_telemetry_packet(packet(10))
        self.assertEqual(9, sample["Throttle"])
        self.assertIsNone(sample["Fault_RC"])
        self.assertEqual(30, len(sample_to_csv_row("00:00:00.000", sample)))

    def test_14_field_packet_populates_legacy_fault_and_rc_fields(self):
        sample = parse_telemetry_packet(packet(14))
        self.assertEqual(13, sample["RC_Dropped_Pkts"])
        self.assertIsNone(sample["Fault_IMU1"])

    def test_21_field_packet_populates_cascade_diagnostics(self):
        sample = parse_telemetry_packet(packet(21))
        self.assertEqual(20, sample["Calibration_OK"])
        for name in (
            "Motor_M1",
            "Motor_M2",
            "Motor_M3",
            "Motor_M4",
            "PID_Loop_Hz",
            "TgtRate_Roll",
            "TgtRate_Pitch",
            "TgtRate_Yaw",
        ):
            self.assertIsNone(sample[name])
        self.assertEqual(29, len(TELEMETRY_FIELDS))
        self.assertEqual(30, len(CSV_FIELDS))

    def test_29_field_packet_parses_new_observability_types(self):
        line = ",".join(
            (
                "1.25", "-2.50", "3.75",
                "4.50", "-5.25", "6.00",
                "0.100", "-0.200", "1.000",
                "1100", "0", "1", "123", "4", "0", "1", "0", "1", "0", "0", "1",
                "1101", "1102", "1103", "1104", "998",
                "12.50", "-23.75", "0.00",
            )
        )

        sample = parse_telemetry_packet(line)

        for name, expected in (
            ("Motor_M1", 1101),
            ("Motor_M2", 1102),
            ("Motor_M3", 1103),
            ("Motor_M4", 1104),
            ("PID_Loop_Hz", 998),
        ):
            self.assertEqual(expected, sample[name])
            self.assertIs(type(sample[name]), int)
        for name, expected in (
            ("TgtRate_Roll", 12.5),
            ("TgtRate_Pitch", -23.75),
            ("TgtRate_Yaw", 0.0),
        ):
            self.assertEqual(expected, sample[name])
            self.assertIs(type(sample[name]), float)

    def test_explicit_type_map_covers_new_field_types(self):
        for name in ("Motor_M1", "Motor_M2", "Motor_M3", "Motor_M4", "PID_Loop_Hz"):
            self.assertIs(telemetry_schema.TELEMETRY_FIELD_TYPES[name], int)
        for name in ("TgtRate_Roll", "TgtRate_Pitch", "TgtRate_Yaw"):
            self.assertIs(telemetry_schema.TELEMETRY_FIELD_TYPES[name], float)

    def test_short_packet_is_rejected(self):
        with self.assertRaises(ValueError):
            parse_telemetry_packet(packet(9))


class GainsPacketTest(unittest.TestCase):
    def test_gains_packet_detection(self):
        self.assertTrue(telemetry_schema.is_gains_packet("  GAINS,1.0  "))
        self.assertFalse(telemetry_schema.is_gains_packet("GAINS"))
        self.assertFalse(telemetry_schema.is_gains_packet(packet(10)))

    def test_gains_packet_parses_all_named_values_as_floats(self):
        line = "GAINS," + ",".join(str(index / 10) for index in range(1, 13))

        gains = telemetry_schema.parse_gains_packet(line)

        self.assertEqual(list(GAIN_NAMES), list(gains))
        self.assertEqual(0.1, gains["Kp_Angle_Roll"])
        self.assertEqual(1.2, gains["Kd_Rate_Yaw"])
        self.assertTrue(all(type(value) is float for value in gains.values()))

    def test_gains_packet_rejects_wrong_field_count(self):
        with self.assertRaises(ValueError):
            telemetry_schema.parse_gains_packet("GAINS,1.0,2.0")


if __name__ == "__main__":
    unittest.main()
