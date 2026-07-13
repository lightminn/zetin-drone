# ZETIN Drone

ESP32-S3, dual ICM42670 IMUs, and PWM ESC control are the current development
stack. Motor PWM and raw IMU acquisition are bench-verified; closed-loop
stabilization remains experimental.

## Start here

- [Project overview](docs/project_overview.md)
- [Current flight-controller candidate](firmware/flight/dual_imu_cascade_pwm/)
- [Firmware and diagnostic guide](firmware/README.md)
- [PC control and telemetry tools](scripts/README.md)
- [UDP protocol and telemetry schema](docs/udp_protocol.md)
- [Firmware lifecycle catalog](docs/firmware_catalog.md)

## Quick verification

```bash
arduino-cli compile --warnings all --fqbn esp32:esp32:esp32s3 \
  --build-path /tmp/zetin-flight-build \
  firmware/flight/dual_imu_cascade_pwm

/home/light/anaconda3/bin/python -m py_compile scripts/*.py
/home/light/anaconda3/bin/python tools/check_repo_layout.py
```

## Safety

Remove propellers for bench tests. Verify power polarity, pin assignments,
motor order, and correction direction before any restrained flight test.
Archived experiments are unsupported and may be unsafe.
