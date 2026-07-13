# UDP control and telemetry protocol

The current flight-controller candidate and ground tools use this endpoint.

```text
Transport: UDP
Drone address: 192.168.4.1
Port: 4210
Registration: any incoming packet identifies the ground-station endpoint;
              current receivers send "connect" periodically.
```

The firmware runs a SoftAP with the SSID `Drone_Tuning`. The address and port
are defined by
[`dual_imu_cascade_pwm.ino`](../firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino).

## Ground station to drone

Commands are UTF-8/ASCII text datagrams. Whitespace separates arguments.

```text
start
stop
rc <seq> <roll> <pitch> <yaw>
th <microseconds>
yaw <0|1>
pa|ia|da <value>
pr|ir|dr <value>
pp|ip|dp <value>
py|iy|dy <value>
```

- `start` arms control only after the firmware's safety conditions pass;
  `stop` disarms immediately.
- `rc` carries the packet sequence and target roll, pitch, and yaw angles.
- `th` sets the base ESC pulse width in microseconds; firmware limits still
  apply.
- `yaw` enables or disables yaw control.
- `pa/ia/da` set the shared roll and pitch inner rate gains. The `r`, `p`, and
  `y` variants set roll, pitch, and yaw inner rate gains independently.

Any received datagram also registers its source address as the telemetry
destination. [`receive_telemetry.py`](../scripts/receive_telemetry.py) and
[`monitor_telemetry.py`](../scripts/monitor_telemetry.py) periodically send
`connect` for this purpose.

## Drone to ground station

Telemetry is a comma-separated datagram with these 21 fields in exact order:

```text
Roll, Pitch, Yaw,
Gyro_X, Gyro_Y, Gyro_Z,
Accel_X, Accel_Y, Accel_Z,
Throttle,
Fault_RC, Fault_Critical,
RC_Total_Pkts, RC_Dropped_Pkts,
Fault_IMU1, Fault_IMU2, Fault_Disagree,
Active_IMUs, Mixer_Scaled, Fault_Attitude, Calibration_OK
```

`telemetry_schema.py` also accepts historical packet lengths:

- A 10-field packet ends at `Throttle`.
- A 14-field packet ends at `RC_Dropped_Pkts`.
- Values absent from a legacy packet become blank cells in the normalized CSV.
- `Timestamp` is not sent by the drone. Ground tools add it as the first CSV
  column, producing 22 columns for a current packet.

The shared implementation is
[`telemetry_schema.py`](../scripts/telemetry_schema.py).
