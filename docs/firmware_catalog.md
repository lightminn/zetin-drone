# 펌웨어 수명주기 카탈로그

`현행`으로 표시된 행만 유지되는 ESP32-S3 + ICM42670 + PWM 경로에 속한다.
보관된 스케치는 원형 그대로 보존하며, 복구하거나 빌드 검증하지 않는다.
모터 제어 스케치를 실행하기 전에는 프로펠러를 제거한다.

| 수명주기 | 대상 MCU | 센서 | 모터 프로토콜 | 안전/빌드 메모 | 링크 |
|---|---|---|---|---|---|
| 현행 | ESP32-S3 | 듀얼 ICM42670 | PWM | 실험 단계의 비행 제어 후보. 현행 빌드 통과 | [`dual_imu_cascade_pwm`](../firmware/flight/dual_imu_cascade_pwm/) |
| 현행 | ESP32-S3 | 없음 | PWM | 프로펠러 제거 상태. 4채널 ESC 벤치 테스트 | [`motor_pwm_bench`](../firmware/diagnostics/motor_pwm_bench/) |
| 현행 | ESP32-S3 | ICM42670 1개 | 없음 | raw SPI 읽기. 별도 18/19/23/5 핀 배치 사용 | [`icm42670_single_raw`](../firmware/diagnostics/icm42670_single_raw/) |
| 현행 | ESP32-S3 | 듀얼 ICM42670 | 없음 | 듀얼 IMU raw SPI 읽기 | [`icm42670_dual_raw`](../firmware/diagnostics/icm42670_dual_raw/) |
| 현행 | ESP32-S3 | 듀얼 ICM42670 | PWM | 프로펠러 제거 상태. 루프 주기·추정기 진단 | [`icm42670_dual_loop_debug`](../firmware/diagnostics/icm42670_dual_loop_debug/) |
| 현행 | ESP32-S3 | 듀얼 ICM42670 | 없음 | PCB v1.5.2 센서 브링업 진단 | [`board_v1_5_2_dual_imu`](../firmware/diagnostics/board_v1_5_2_dual_imu/) |
| 보관 | ESP32-S3 | ICM42670 1개 | PWM | 대체된 단일 IMU PID. 미지원 | [`single_imu_pid_pwm`](../firmware/archive/legacy_flight/single_imu_pid_pwm/) |
| 보관 | ESP32-S3 | 듀얼 ICM42670 | PWM | 대체된 듀얼 IMU PID. 미지원 | [`dual_imu_pid_pwm`](../firmware/archive/legacy_flight/dual_imu_pid_pwm/) |
| 보관 | ESP32-S3 | ICM42670 1개 | PWM | 대체된 캐스케이드 실험. 미지원 | [`single_imu_cascade_pwm`](../firmware/archive/legacy_flight/single_imu_cascade_pwm/) |
| 보관 | ESP32-S3 | ICM42670 1개 | PWM | Kalman/PID 비행 실험. 미지원 | [`single_imu_kalman_pid_pwm`](../firmware/archive/legacy_flight/single_imu_kalman_pid_pwm/) |
| 보관 | ESP32-S3 | ICM42670 1개 | 없음 | Kalman 자세 추정 실험. 미지원 | [`icm42670_kalman_attitude`](../firmware/archive/filter_experiments/icm42670_kalman_attitude/) |
| 보관 | ESP32 계열 | 없음 | DShot600 | 고정 단일 모터 출력. 미지원 | [`single_motor_fixed_throttle`](../firmware/archive/dshot/single_motor_fixed_throttle/) |
| 보관 | ESP32 계열 | 없음 | DShot600 | 시리얼 제어 단일 모터 실험. 미지원 | [`single_motor_serial_control`](../firmware/archive/dshot/single_motor_serial_control/) |
| 보관 | ESP32 계열 | MPU6500 | DShot600 | 기울기-스로틀 실험. 미지원 | [`mpu6500_tilt_throttle`](../firmware/archive/dshot/mpu6500_tilt_throttle/) |
| 보관 | ESP32 계열 | MPU6500 | DShot600 | 단일 모터 PID 벤치 실험. 미지원 | [`mpu6500_pid_bench`](../firmware/archive/dshot/mpu6500_pid_bench/) |
| 보관 | ESP32 계열 | 없음 | DShot600 | **위험: 4개 모터를 최대 출력 근처로 구동함**. 벤치 테스트로 실행 금지 | [`four_motor_full_throttle_unsafe`](../firmware/archive/dshot/four_motor_full_throttle_unsafe/) |
| 보관 | ESP32-S3 | 시뮬레이션 입력 | DShot600 | 미완성 듀얼코어 모터 할당기. 미지원 | [`motor_allocator_dual_core`](../firmware/archive/dshot/motor_allocator_dual_core/) |
| 보관 | ESP32-S3 | 시뮬레이션 입력 | DShot600 | 오래된 PlatformIO 골격에 의존. 미지원 | [`motor_allocator_dshot`](../firmware/archive/platformio_skeleton/examples/motor_allocator_dshot/) |
| 보관 | ESP32-S3 | BMM350 | 없음 | 단독 자기계 읽기. 미지원 | [`bmm350_read`](../firmware/archive/other_sensors/bmm350_read/) |
| 보관 | ESP32-S3 | BMM350 | 없음 | PCB v1.5.2 자기계 진단. 미지원 | [`board_v1_5_2_bmm350`](../firmware/archive/other_sensors/board_v1_5_2_bmm350/) |
| 보관 | ESP32-S3 | GPS UART | 없음 | raw NMEA 패스스루. 미지원 | [`gps_uart_passthrough`](../firmware/archive/other_sensors/gps_uart_passthrough/) |
| 보관 | Arduino 호환 | US100 | 없음 | 오래된 의존성 기반 거리 측정 실험. 미지원 | [`us100_distance`](../firmware/archive/other_sensors/us100_distance/) |
| 보관 | STM32 F411 | UART | 없음 | 대체 MCU 수신 실험. 미지원 | [`stm32_f411_uart_rx`](../firmware/archive/other_mcus/stm32_f411_uart_rx/) |
