# Firmware lifecycle catalog

Only rows marked `current` belong to the maintained ESP32-S3 + ICM42670 + PWM
path. Archived sketches are preserved as-is; they are not repaired or
build-verified. Remove propellers before running any motor-control sketch.

| Lifecycle | Target MCU | Sensor | Motor protocol | Safety/build note | Link |
|---|---|---|---|---|---|
| current | ESP32-S3 | dual ICM42670 | PWM | Experimental flight-controller candidate; current build passes | [`dual_imu_cascade_pwm`](../firmware/flight/dual_imu_cascade_pwm/) |
| current | ESP32-S3 | none | PWM | Propellers off; 4-channel ESC bench test | [`motor_pwm_bench`](../firmware/diagnostics/motor_pwm_bench/) |
| current | ESP32-S3 | one ICM42670 | none | Raw SPI read; uses the separate 18/19/23/5 pinout | [`icm42670_single_raw`](../firmware/diagnostics/icm42670_single_raw/) |
| current | ESP32-S3 | dual ICM42670 | none | Raw dual-IMU SPI read | [`icm42670_dual_raw`](../firmware/diagnostics/icm42670_dual_raw/) |
| current | ESP32-S3 | dual ICM42670 | PWM | Propellers off; loop timing and estimator diagnostic | [`icm42670_dual_loop_debug`](../firmware/diagnostics/icm42670_dual_loop_debug/) |
| current | ESP32-S3 | dual ICM42670 | none | PCB v1.5.2 sensor bring-up diagnostic | [`board_v1_5_2_dual_imu`](../firmware/diagnostics/board_v1_5_2_dual_imu/) |
| archived | ESP32-S3 | one ICM42670 | PWM | Superseded single-IMU PID; unsupported | [`single_imu_pid_pwm`](../firmware/archive/legacy_flight/single_imu_pid_pwm/) |
| archived | ESP32-S3 | dual ICM42670 | PWM | Superseded dual-IMU PID; unsupported | [`dual_imu_pid_pwm`](../firmware/archive/legacy_flight/dual_imu_pid_pwm/) |
| archived | ESP32-S3 | one ICM42670 | PWM | Superseded cascade experiment; unsupported | [`single_imu_cascade_pwm`](../firmware/archive/legacy_flight/single_imu_cascade_pwm/) |
| archived | ESP32-S3 | one ICM42670 | PWM | Kalman/PID flight experiment; unsupported | [`single_imu_kalman_pid_pwm`](../firmware/archive/legacy_flight/single_imu_kalman_pid_pwm/) |
| archived | ESP32-S3 | one ICM42670 | none | Kalman attitude experiment; unsupported | [`icm42670_kalman_attitude`](../firmware/archive/filter_experiments/icm42670_kalman_attitude/) |
| archived | ESP32 family | none | DShot600 | Fixed single-motor output; unsupported | [`single_motor_fixed_throttle`](../firmware/archive/dshot/single_motor_fixed_throttle/) |
| archived | ESP32 family | none | DShot600 | Serial-controlled single-motor experiment; unsupported | [`single_motor_serial_control`](../firmware/archive/dshot/single_motor_serial_control/) |
| archived | ESP32 family | MPU6500 | DShot600 | Tilt-to-throttle experiment; unsupported | [`mpu6500_tilt_throttle`](../firmware/archive/dshot/mpu6500_tilt_throttle/) |
| archived | ESP32 family | MPU6500 | DShot600 | Single-motor PID bench experiment; unsupported | [`mpu6500_pid_bench`](../firmware/archive/dshot/mpu6500_pid_bench/) |
| archived | ESP32 family | none | DShot600 | **Unsafe: commands four motors near full output**; do not run as a bench test | [`four_motor_full_throttle_unsafe`](../firmware/archive/dshot/four_motor_full_throttle_unsafe/) |
| archived | ESP32-S3 | simulated inputs | DShot600 | Incomplete dual-core motor allocator; unsupported | [`motor_allocator_dual_core`](../firmware/archive/dshot/motor_allocator_dual_core/) |
| archived | ESP32-S3 | simulated inputs | DShot600 | Depends on stale PlatformIO skeleton; unsupported | [`motor_allocator_dshot`](../firmware/archive/platformio_skeleton/examples/motor_allocator_dshot/) |
| archived | ESP32-S3 | BMM350 | none | Standalone magnetometer read; unsupported | [`bmm350_read`](../firmware/archive/other_sensors/bmm350_read/) |
| archived | ESP32-S3 | BMM350 | none | PCB v1.5.2 magnetometer diagnostic; unsupported | [`board_v1_5_2_bmm350`](../firmware/archive/other_sensors/board_v1_5_2_bmm350/) |
| archived | ESP32-S3 | GPS UART | none | Raw NMEA passthrough; unsupported | [`gps_uart_passthrough`](../firmware/archive/other_sensors/gps_uart_passthrough/) |
| archived | Arduino-compatible | US100 | none | Distance experiment with old dependency; unsupported | [`us100_distance`](../firmware/archive/other_sensors/us100_distance/) |
| archived | STM32 F411 | UART | none | Alternate-MCU receive experiment; unsupported | [`stm32_f411_uart_rx`](../firmware/archive/other_mcus/stm32_f411_uart_rx/) |
