# UDP 제어·텔레메트리 프로토콜

현행 비행 제어 후보와 지상 도구는 이 엔드포인트를 사용한다.

```text
전송: UDP
드론 주소: 192.168.4.1
포트: 4210
등록: 수신되는 모든 패킷이 지상국 엔드포인트를 식별한다.
      현행 수신기는 주기적으로 "connect"를 보낸다.
```

펌웨어는 SSID `Drone_Tuning`으로 SoftAP를 운영한다. 주소와 포트는
[`dual_imu_cascade_pwm.ino`](../firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino)에
정의돼 있다.

## 지상국 → 드론

명령은 UTF-8/ASCII 텍스트 데이터그램이다. 인자는 공백으로 구분한다.

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

- `start`는 펌웨어의 안전 조건을 통과한 뒤에만 제어를 시동한다.
  `stop`은 즉시 시동을 해제한다.
- `rc`는 패킷 시퀀스와 목표 roll, pitch, yaw 각도를 담는다.
- `th`는 기본 ESC 펄스 폭을 마이크로초 단위로 설정한다. 펌웨어의 제한은
  그대로 적용된다.
- `yaw`는 yaw 제어를 켜거나 끈다.
- `pa/ia/da`는 공유되는 roll·pitch 안쪽 각속도 게인을 설정한다. `r`, `p`,
  `y` 변형은 각각 roll, pitch, yaw 안쪽 각속도 게인을 독립적으로 설정한다.

수신된 데이터그램은 그 출발지 주소를 텔레메트리 목적지로도 등록한다.
[`receive_telemetry.py`](../scripts/receive_telemetry.py)와
[`monitor_telemetry.py`](../scripts/monitor_telemetry.py)는 이를 위해 주기적으로
`connect`를 보낸다.

## 드론 → 지상국

텔레메트리는 다음 21개 필드를 정확한 순서로 담은 쉼표 구분 데이터그램이다.

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

`telemetry_schema.py`는 과거 패킷 길이도 함께 받아들인다.

- 10필드 패킷은 `Throttle`에서 끝난다.
- 14필드 패킷은 `RC_Dropped_Pkts`에서 끝난다.
- 과거 패킷에 없는 값은 정규화된 CSV에서 빈 셀이 된다.
- `Timestamp`는 드론이 보내지 않는다. 지상 도구가 CSV의 첫 열로 추가하므로
  현행 패킷은 22개 열이 된다.

공유 구현은
[`telemetry_schema.py`](../scripts/telemetry_schema.py)에 있다.
