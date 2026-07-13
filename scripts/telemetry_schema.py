"""Shared parser for the drone UDP telemetry and CSV schema.

The first 10 fields are common to legacy firmware. The first 14 fields retain
the format used by firmware/archive/legacy_flight/dual_imu_pid_pwm, and fields
14-20 are diagnostics added by firmware/flight/dual_imu_cascade_pwm.
"""

import math


TELEMETRY_FIELDS = (
    "Roll",
    "Pitch",
    "Yaw",
    "Gyro_X",
    "Gyro_Y",
    "Gyro_Z",
    "Accel_X",
    "Accel_Y",
    "Accel_Z",
    "Throttle",
    "Fault_RC",
    "Fault_Critical",
    "RC_Total_Pkts",
    "RC_Dropped_Pkts",
    "Fault_IMU1",
    "Fault_IMU2",
    "Fault_Disagree",
    "Active_IMUs",
    "Mixer_Scaled",
    "Fault_Attitude",
    "Calibration_OK",
)

CSV_FIELDS = ("Timestamp",) + TELEMETRY_FIELDS
REQUIRED_FIELD_COUNT = 10
FLOAT_FIELD_COUNT = 9


def _parse_finite_float(raw, name):
    value = float(raw)
    if not math.isfinite(value):
        raise ValueError(f"{name} is not finite")
    return value


def _parse_integer(raw, name):
    value = _parse_finite_float(raw, name)
    if not value.is_integer():
        raise ValueError(f"{name} is not an integer")
    return int(value)


def parse_telemetry_packet(line):
    """Parse a 10-, 14-, or 21-field packet into a fixed-schema dict.

    Fields unavailable in legacy packets are returned as ``None`` so CSV
    files retain the full header without inventing healthy/fault values.
    Extra future fields are ignored after the known 21 fields.
    """

    parts = [part.strip() for part in line.strip().split(",")]
    if len(parts) < REQUIRED_FIELD_COUNT:
        raise ValueError(
            f"telemetry has {len(parts)} fields; need at least {REQUIRED_FIELD_COUNT}"
        )

    sample = dict.fromkeys(TELEMETRY_FIELDS)
    for index, name in enumerate(TELEMETRY_FIELDS):
        if index >= len(parts):
            break
        raw = parts[index]
        if raw == "":
            continue
        if index < FLOAT_FIELD_COUNT:
            sample[name] = _parse_finite_float(raw, name)
        else:
            sample[name] = _parse_integer(raw, name)
    return sample


def sample_to_csv_row(timestamp, sample):
    """Return a row matching ``CSV_FIELDS``; unknown legacy values stay blank."""

    return [timestamp] + [
        "" if sample[name] is None else sample[name] for name in TELEMETRY_FIELDS
    ]


def active_fault_names(sample):
    """Return concise names for currently asserted fault fields."""

    fields = (
        ("Fault_RC", "RC"),
        ("Fault_Critical", "CRIT"),
        ("Fault_IMU1", "IMU1"),
        ("Fault_IMU2", "IMU2"),
        ("Fault_Disagree", "DISAGREE"),
        ("Fault_Attitude", "TILT"),
    )
    return [label for name, label in fields if sample.get(name) == 1]
