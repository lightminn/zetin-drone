# 비행 로그

[`receive_telemetry.py`](../scripts/receive_telemetry.py)와
[`monitor_telemetry.py`](../scripts/monitor_telemetry.py)는 다음 규칙으로 이곳에
파일을 생성한다.

```text
flight_log_YYYY-MM-DD_HHMMSS.csv
```

현행 CSV 파일은 PC 수신 시각 다음에 펌웨어 텔레메트리 21개 필드가 이어져
총 22개 열을 가진다.

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

공유 파서는 `Throttle`에서 끝나는 10필드 패킷과 `RC_Dropped_Pkts`에서 끝나는
14필드 패킷도 받아들인다. 이 과거 패킷에 없는 필드는 빈 셀로 기록된다.
`Timestamp`는 항상 PC에서 추가하며 UDP 데이터그램의 일부가 아니다.

저장소 루트에서 생성된 로그를 분석한다.

```bash
python scripts/analyze_flight_log.py logs/flight_log_YYYY-MM-DD_HHMMSS.csv
```

이 스키마는 배터리 전압, 개별 모터 출력, PID 항 관련 열을 주장하지 않는다.
와이어 포맷은 [`udp_protocol.md`](../docs/udp_protocol.md)를 참고한다.
