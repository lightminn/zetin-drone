# ZETIN Drone

현재 개발 스택은 ESP32-S3, 듀얼 ICM42670 IMU, PWM ESC 제어다. 모터 PWM과
raw IMU 취득은 벤치에서 검증됐지만, 폐쇄루프 자세제어는 아직 실험 단계다.

## 여기서 시작

- [프로젝트 개요](docs/project_overview.md)
- [현행 비행 제어 후보](firmware/flight/dual_imu_cascade_pwm/)
- [flix 기반 쿼터니언 제어 후보](firmware/flight/dual_imu_flix_quat_pwm/)
- [펌웨어 및 진단 가이드](firmware/README.md)
- [PC 제어·텔레메트리 도구](scripts/README.md)
- [UDP 프로토콜과 텔레메트리 스키마](docs/udp_protocol.md)
- [펌웨어 수명주기 카탈로그](docs/firmware_catalog.md)

## 빠른 확인

```bash
for sketch in firmware/flight/*/; do
  arduino-cli compile --warnings all --fqbn esp32:esp32:esp32s3 \
    --build-path "/tmp/zetin-$(basename "$sketch")" "$sketch"
done

python3 -m py_compile scripts/*.py
python3 tools/check_repo_layout.py
python3 -m unittest discover -s tools -p "test_*.py"
```

## 안전

벤치 테스트에서는 프로펠러를 제거한다. 제한된 비행 테스트 전에 전원 극성,
핀 배치, 모터 순서, 보정 방향을 확인한다. 보관된 실험 코드는 지원 대상이
아니며 안전하지 않을 수 있다.
