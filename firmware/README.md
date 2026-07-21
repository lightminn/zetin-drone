# Firmware

이 디렉터리의 현행 범위는 **ESP32-S3 + ICM42670 + PWM 제어**뿐이다.
비행 후보 펌웨어와 하드웨어 진단 스케치만 아래 경로에서 유지한다.
나머지 실험 코드는 [`archive/`](archive/README.md)에 원형 보관하며 빌드
지원 대상으로 보지 않는다.

## 현행 비행 펌웨어

- [`flight/dual_imu_cascade_pwm/`](flight/dual_imu_cascade_pwm/): 듀얼
  ICM42670, 바깥쪽 자세 루프와 안쪽 각속도 루프, 4채널 PWM을 사용하는
  현행 비행 제어 후보. 실기 비행 안정성이 검증 완료됐다는 뜻은 아니다.

핀 배치: 모터 FL/RR/FR/RL = GPIO 4/5/6/7, SPI SCK/MISO/MOSI =
GPIO 12/13/11, IMU1/IMU2 CS = GPIO 10/9.

## 현행 진단 스케치

- [`diagnostics/motor_pwm_bench/`](diagnostics/motor_pwm_bench/): GPIO
  4/5/6/7의 4개 ESC PWM 출력 벤치 테스트.
- [`diagnostics/icm42670_single_raw/`](diagnostics/icm42670_single_raw/):
  단일 ICM42670 원시값 출력. SPI SCK/MISO/MOSI/CS = GPIO 18/19/23/5.
  이 배치는 PCB v1.5.2 진단과 다르다.
- [`diagnostics/icm42670_dual_raw/`](diagnostics/icm42670_dual_raw/): 듀얼
  ICM42670 원시값 출력. SPI SCK/MISO/MOSI = GPIO 12/13/11, CS =
  GPIO 10/9.
- [`diagnostics/icm42670_dual_loop_debug/`](diagnostics/icm42670_dual_loop_debug/):
  듀얼 IMU 자세/각속도 루프와 UDP 진단 텔레메트리. 모터 = GPIO
  4/5/6/7, SPI SCK/MISO/MOSI = GPIO 12/13/11, CS = GPIO 10/9.
- [`diagnostics/board_v1_5_2_dual_imu/`](diagnostics/board_v1_5_2_dual_imu/):
  PCB v1.5.2 듀얼 ICM42670 확인. SPI SCK/MISO/MOSI = GPIO 12/13/11,
  CS = GPIO 10/9.

## 빌드

Arduino CLI와 ESP32-S3 보드 패키지를 사용한다. 빌드 산출물은 저장소 밖
`/tmp`에 둔다. 필요한 라이브러리는 `ICM42670P`(비행·IMU 진단)와
`ESP32Servo`(`motor_pwm_bench`)다:

```bash
arduino-cli core install esp32:esp32
arduino-cli lib install ICM42670P ESP32Servo
```

```bash
sketch=firmware/flight/dual_imu_cascade_pwm
name=$(basename "$sketch")
arduino-cli compile --warnings all \
  --fqbn esp32:esp32:esp32s3 \
  --build-path "/tmp/zetin-$name" \
  "$sketch"
```

진단 스케치도 `sketch` 경로만 바꿔 같은 방식으로 빌드한다. 보관 코드의
의존성이나 빌드는 복구하지 않는다.

## 안전

- 모터를 구동하는 모든 벤치 시험과 펌웨어 업로드는 프로펠러를 제거한
  상태에서 진행한다.
- 배터리 연결 전에 전압과 극성을 다시 확인한다.
- 모터 번호, 회전 방향, PWM 최소값과 비상 정지 동작을 확인하기 전에는
  기체를 띄우지 않는다.
- [`archive/`](archive/README.md)의 모터 제어 코드는 특히 안전하다고
  가정하면 안 된다.
