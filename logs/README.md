# Flight logs

[`receive_telemetry.py`](../scripts/receive_telemetry.py) and
[`monitor_telemetry.py`](../scripts/monitor_telemetry.py) create files here with
the convention:

```text
flight_log_YYYY-MM-DD_HHMMSS.csv
```

Current CSV files contain the PC receive time followed by the 21 firmware
telemetry fields, for 22 columns total:

```text
Timestamp,
Roll, Pitch, Yaw,
Gyro_X, Gyro_Y, Gyro_Z,
Accel_X, Accel_Y, Accel_Z,
Throttle,
Fault_RC, Fault_Critical,
RC_Total_Pkts, RC_Dropped_Pkts,
Fault_IMU1, Fault_IMU2, Fault_Disagree,
Active_IMUs, Mixer_Scaled, Fault_Attitude, Calibration_OK
```

The shared parser accepts 10-field packets ending at `Throttle` and 14-field
packets ending at `RC_Dropped_Pkts`. Fields not present in those legacy packets
are written as blank cells. `Timestamp` is always added on the PC and is never
part of the UDP datagram.

Analyze a generated log from the repository root:

```bash
python scripts/analyze_flight_log.py logs/flight_log_YYYY-MM-DD_HHMMSS.csv
```

The schema does not claim battery-voltage, individual motor-output, or PID-term
columns. See [`udp_protocol.md`](../docs/udp_protocol.md) for the wire format.
