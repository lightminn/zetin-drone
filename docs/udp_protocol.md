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
[`dual_imu_flix_quat_pwm`](../firmware/flight/dual_imu_flix_quat_pwm/)도
같은 엔드포인트·명령·텔레메트리 스키마를 구현한다. 단, 게인 명령의 단위가
SI(rad 기반)이고 yaw 각도 부호가 반대(CCW+)이므로
[해당 README](../firmware/flight/dual_imu_flix_quat_pwm/README.md)를 참조한다.

## 지상국 → 드론

명령은 UTF-8/ASCII 텍스트 데이터그램이다. 인자는 공백으로 구분한다.

```text
start
stop
rc <seq> <roll> <pitch> <yaw>
th <microseconds>
yaw <0|1>

# 안쪽 각속도 PID 게인
pa|ia|da <value>      # roll+pitch 공통 P/I/D
pr|ir|dr <value>      # roll P/I/D
pp|ip|dp <value>      # pitch P/I/D
py|iy|dy <value>      # yaw P/I/D
rp|ri|rd <value>      # roll+pitch 공통 P/I/D (pa|ia|da와 동일)
yp|yi|yd <value>      # yaw P/I/D (py|iy|dy와 동일)

# 바깥 각도 P 게인
ap <value>            # roll+pitch 공통
ar|at|ay <value>      # roll / pitch / yaw
```

- `start`는 캘리브레이션 성공, 기울기 정상, 사용 가능한 IMU 존재, IMU
  일치 조건을 모두 통과한 뒤에만 시동하며, latch된 fault를 해제하고
  스로틀 창을 기본값(base 1100, min 1050, max 1250)으로 리셋한다.
  `stop`은 즉시 시동을 해제한다.
- `rc`는 패킷 시퀀스와 목표 roll, pitch, yaw 각도를 담는다. roll·pitch
  목표는 ±30°로 제한된다. 시퀀스가 이전보다 작거나 같은 패킷(지연
  도착·중복)은 폐기된다.
- `th`는 기본 ESC 펄스 폭을 마이크로초 단위로 설정한다. 1000~1900으로
  제한되며, min/max 스로틀 창을 기본값 ±150 마진으로 함께 재설정한다.
- `yaw`는 yaw 각도 유지(바깥 루프)를 켜거나 끈다. 꺼져 있어도 yaw 각속도
  감쇠(안쪽 rate 루프, 목표 0)는 항상 동작한다. 켜는 순간 현재 추정 yaw
  각도를 setpoint로 동기화해 점프를 방지한다.
- 게인 값은 유한한 양수만 수락한다. 주력 펌웨어(cascade)는 오타로 인한
  과대 게인을 막기 위해 angle P는 20, rate PID는 10을 상한으로 거부하고
  거부 사유를 시리얼에 남긴다. 보조 실험인 flix 후보는 0~100을 수락한다.

수신된 데이터그램은 그 출발지 주소를 텔레메트리 목적지로도 등록한다.
[`receive_telemetry.py`](../scripts/receive_telemetry.py)와
[`monitor_telemetry.py`](../scripts/monitor_telemetry.py)는 이를 위해 주기적으로
`connect`를 보낸다.

## 드론 → 지상국

텔레메트리는 다음 22개 필드를 정확한 순서로 담은 쉼표 구분 데이터그램이다.

```text
Roll, Pitch, Yaw,
Gyro_X, Gyro_Y, Gyro_Z,
Accel_X, Accel_Y, Accel_Z,
Throttle,
Fault_RC, Fault_Critical,
RC_Total_Pkts, RC_Dropped_Pkts,
Fault_IMU1, Fault_IMU2, Fault_Disagree,
Active_IMUs, Mixer_Scaled, Fault_Attitude, Calibration_OK,
Armed
```

- `Armed`는 펌웨어 safety lock의 반전값이다. `start`가 거부되거나
  펌웨어가 스스로 시동을 해제한 것을 지상국이 이 필드로 감지한다.

`telemetry_schema.py`는 과거 패킷 길이도 함께 받아들인다.

- 10필드 패킷은 `Throttle`에서 끝난다.
- 14필드 패킷은 `RC_Dropped_Pkts`에서 끝난다.
- 21필드 패킷은 `Calibration_OK`에서 끝난다 (`Armed` 도입 이전 펌웨어).
- 과거 패킷에 없는 값은 정규화된 CSV에서 빈 셀이 된다.
- `Timestamp`는 드론이 보내지 않는다. 지상 도구가 CSV의 첫 열로 추가하므로
  현행 패킷은 23개 열이 된다.

공유 구현은
[`telemetry_schema.py`](../scripts/telemetry_schema.py)에 있다.
