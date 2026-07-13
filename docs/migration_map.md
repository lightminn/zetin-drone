# Repository migration map

This table is the authoritative lookup from the former repository layout to
the cleanup layout. Every new path exists in this revision; every old path was
removed or renamed.

| Old path | New path | Lifecycle |
|---|---|---|
| `firmware/examples/DUAL_IMU_CASCADE/DUAL_IMU_CASCADE.ino` | `firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino` | current |
| `firmware/examples/PWM_TEST/PWM_TEST.ino` | `firmware/diagnostics/motor_pwm_bench/motor_pwm_bench.ino` | current |
| `firmware/examples/IMU_TEST_RAW/IMU_TEST_RAW.ino` | `firmware/diagnostics/icm42670_single_raw/icm42670_single_raw.ino` | current |
| `firmware/examples/DUAL_IMU_RAW_TEST/DUAL_IMU_RAW_TEST.ino` | `firmware/diagnostics/icm42670_dual_raw/icm42670_dual_raw.ino` | current |
| `firmware/examples/DUAL_IMU_PID_DEBUG/DUAL_IMU_PID_DEBUG.ino` | `firmware/diagnostics/icm42670_dual_loop_debug/icm42670_dual_loop_debug.ino` | current |
| `firmware/examples/PCB_1.5.2_TEST/PCB_1.5.2_TEST.ino` | `firmware/diagnostics/board_v1_5_2_dual_imu/board_v1_5_2_dual_imu.ino` | current |
| `firmware/examples/PWM_TEST_IMU_PID/PWM_TEST_IMU_PID.ino` | `firmware/archive/legacy_flight/single_imu_pid_pwm/single_imu_pid_pwm.ino` | archived |
| `firmware/examples/PWM_TEST_DUAL_IMU_PID/PWM_TEST_DUAL_IMU_PID.ino` | `firmware/archive/legacy_flight/dual_imu_pid_pwm/dual_imu_pid_pwm.ino` | archived |
| `firmware/examples/DUAL_LOOP_TEST/DUAL_LOOP_TEST.ino` | `firmware/archive/legacy_flight/single_imu_cascade_pwm/single_imu_cascade_pwm.ino` | archived |
| `firmware/examples/KALMAN_FLIGHT_TEST/KALMAN_FLIGHT_TEST.ino` | `firmware/archive/legacy_flight/single_imu_kalman_pid_pwm/single_imu_kalman_pid_pwm.ino` | archived |
| `firmware/examples/KALMAN_TEST/KALMAN_TEST.ino` | `firmware/archive/filter_experiments/icm42670_kalman_attitude/icm42670_kalman_attitude.ino` | archived |
| `firmware/examples/DSHOT_TEST/DSHOT_TEST.ino` | `firmware/archive/dshot/single_motor_fixed_throttle/single_motor_fixed_throttle.ino` | archived |
| `firmware/examples/DSHOT_TEST_PERCENT/DSHOT_TEST_PERCENT.ino` | `firmware/archive/dshot/single_motor_serial_control/single_motor_serial_control.ino` | archived |
| `firmware/examples/DSHOT_TEST_IMU/DSHOT_TEST_IMU.ino` | `firmware/archive/dshot/mpu6500_tilt_throttle/mpu6500_tilt_throttle.ino` | archived |
| `firmware/examples/DSHOT_TEST_IMU_PID/DSHOT_TEST_IMU_PID.ino` | `firmware/archive/dshot/mpu6500_pid_bench/mpu6500_pid_bench.ino` | archived |
| `firmware/examples/RMT_TEST/RMT_TEST.ino` | `firmware/archive/dshot/four_motor_full_throttle_unsafe/four_motor_full_throttle_unsafe.ino` | archived |
| `firmware/examples/ZETIN_Drone_Standalone_Test/ZETIN_Drone_Standalone_Test.ino` | `firmware/archive/dshot/motor_allocator_dual_core/motor_allocator_dual_core.ino` | archived |
| `firmware/examples/MOTOR_ALGO_TEST/MOTOR_ALGO_TEST.ino` | `firmware/archive/platformio_skeleton/examples/motor_allocator_dshot/motor_allocator_dshot.ino` | archived |
| `firmware/examples/BMM_TEST/BMM_TEST.ino` | `firmware/archive/other_sensors/bmm350_read/bmm350_read.ino` | archived |
| `firmware/examples/PCB_1.5.2_BMM_TEST/PCB_1.5.2_BMM_TEST.ino` | `firmware/archive/other_sensors/board_v1_5_2_bmm350/board_v1_5_2_bmm350.ino` | archived |
| `firmware/examples/GPS_TEST_RAW/GPS_TEST_RAW.ino` | `firmware/archive/other_sensors/gps_uart_passthrough/gps_uart_passthrough.ino` | archived |
| `firmware/examples/US100_TEST/US100_TEST.ino` | `firmware/archive/other_sensors/us100_distance/us100_distance.ino` | archived |
| `firmware/examples/F411_test/F411_test.ino` | `firmware/archive/other_mcus/stm32_f411_uart_rx/stm32_f411_uart_rx.ino` | archived |
| `firmware/src` | `firmware/archive/platformio_skeleton/src` | archived |
| `firmware/lib` | `firmware/archive/platformio_skeleton/lib` | archived |
| `firmware/include` | `firmware/archive/platformio_skeleton/include` | archived |
| `firmware/platformio_config/platformio.ini` | `firmware/archive/platformio_skeleton/platformio.ini` | archived |
| `test/README` | `firmware/archive/platformio_skeleton/test/README` | archived |
| `scripts/Drone_Control_Dualsense.py` | `scripts/control_dualsense.py` | current |
| `scripts/Drone_Tuning.py` | `scripts/tune_pid.py` | current |
| `scripts/Drone_Reciever.py` | `scripts/receive_telemetry.py` | current |
| `scripts/Drone_Monitor.py` | `scripts/monitor_telemetry.py` | current |
| `scripts/Drone_Analasys.py` | `scripts/analyze_flight_log.py` | current |
| `scripts/drone_telemetry.py` | `scripts/telemetry_schema.py` | current |
| `scripts/dual_imu_pid_debug_receiver.py` | `scripts/receive_dual_imu_debug.py` | current |
| `scripts/Controller_test.py` | `scripts/test_dualsense_input.py` | current |
| `scripts/GPS_Reciever.py` | `scripts/archive/receive_gps_udp_legacy.py` | archived |
| `test/tcp_test.py` | `scripts/archive/test_tcp_legacy.py` | archived |
| `docs/ONBOARDING.md` | `docs/project_overview.md` | current |
| `docs/superpowers/specs/2026-05-14-dual-imu-pid-design.md` | `docs/history/2026-05-14-dual-imu-pid-design.md` | historical |
| `docs/superpowers/plans/2026-05-14-dual-imu-pid.md` | `docs/history/2026-05-14-dual-imu-pid-implementation-plan.md` | historical |
| `docs/presentation.c` | `docs/archive/pid_dshot_presentation_snippet.c` | archived |
| `docs/commit_message.txt` | `docs/archive/commit_message_snapshot.txt` | archived |
| `test/README.md` | `scripts/archive/README.md` | folded into archive note |

The machine-local files `.gemini/settings.json`, `.vscode/extensions.json`, and
`.vscode/settings.json` were removed from version control. Their directories,
along with `.codex/`, `.claude/`, and `AGENTS.md`, are intentionally ignored.
