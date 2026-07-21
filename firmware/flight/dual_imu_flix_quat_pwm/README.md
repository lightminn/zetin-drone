# dual_imu_flix_quat_pwm

[flix](https://github.com/okalachev/flix) (MIT, Oleg Kalachev) 아키텍처를
ZETIN 하드웨어(ESP32-S3 + 듀얼 ICM42670 + PWM ESC)에 이식한 비행 펌웨어.
`dual_imu_cascade_pwm`과 별개의 후보이며, UDP 프로토콜과 지상 도구 호환성은
유지한다.

## dual_imu_cascade_pwm과의 차이

| 항목 | dual_imu_cascade_pwm | 이 펌웨어 |
|---|---|---|
| 자세 추정 | 축별 Euler 상보필터 | 쿼터니언 적분 + 착지 시 가속도 보정 + 비행 중 수평 가정 보정 (flix `estimate.ino`) |
| 자세 오차 | 축별 각도 차 | up-벡터 회전벡터 (flix `control.ino`) |
| 내부 단위 | deg, deg/s, µs | rad, rad/s, 정규화 토크 (1.0 = 모터 전 구간 1000µs) |
| outer loop | 250Hz | 1kHz |
| yaw 부호 | 위에서 CW+ | 위에서 CCW+ (오른손 FLU) |
| 믹서 | 자체 부호 | flix 부호 (아래 참조) |

듀얼 IMU 융합·freeze/불일치 감시·시동 게이트·태스크 워치독·UDP 파서는
`dual_imu_cascade_pwm`의 검증된 코드를 그대로 가져왔다. ICM42670 하드웨어
LPF(자이로 121Hz, 가속도 25Hz), 저역 비교 기반 disagree 감시, 중복 `start`
무시, `Armed` 필드를 포함한 22필드 텔레메트리도 두 펌웨어가 동일하다.
`vector.h`, `quaternion.h`, `lpf.h`는 flix 원본 그대로이고 `pid.h`는
dt 명시·측정값 미분 D항·조건부 적분으로 수정한 버전이다.

## 좌표계와 부호 (첫 비행 전 반드시 벤치 검증)

body frame은 오른손 FLU(x 앞, y 왼쪽, z 위). IMU1 sensor frame 기준
`body = (-sensor_y, +sensor_x, +sensor_z)`.

- roll+ = 오른쪽이 내려감, pitch+ = 기수가 내려감, yaw+ = 위에서 볼 때 CCW.
- 텔레메트리 roll/pitch 부호는 구형 펌웨어와 같고 **yaw 부호는 반대**다.
- 구형 펌웨어의 가속도 x,y 부호 매핑은 자이로 매핑과 강체 조건에서 양립할 수
  없어 자이로 기준으로 통일했다. 정지 시 body accel이 (0,0,+1g) 근처가
  아니면 캘리브레이션이 실패하고 시동이 거부된다.

### 벤치 부호 점검 절차 (프로펠러 제거 상태)

1. 부팅 후 시리얼에서 `[CALIB] OK`와 rest accel body가 `(≈0, ≈0, ≈+1)`인지 확인.
2. `monitor_telemetry.py`를 켜고 기체를 손으로 움직여 확인:
   - 오른쪽으로 기울이면 Roll이 +로 증가
   - 기수를 아래로 숙이면 Pitch가 +로 증가
   - 위에서 볼 때 반시계로 돌리면 Yaw가 +로 증가
   하나라도 다르면 `sensorToBody()` 매핑을 수정하고 다시 검증한다.
3. `start` + `th 1150` 상태에서 기체를 오른쪽으로 기울이면 오른쪽 모터
   (M2 RR, M3 FR)가 빨라지는지, 기수를 숙이면 앞 모터(M1 FL, M3 FR)가
   빨라지는지 확인한다.
4. yaw는 `yaw 1`로 켠 뒤 3번과 같은 방식으로 확인한다. 프로펠러 회전
   방향(FL/RR=CW, FR/RL=CCW)이 실제 장착과 다르면 믹서의 yaw 부호를
   반전해야 한다.

## 믹서

```text
M1 FL(GPIO4, CW)  = t + tx - ty + tz
M2 RR(GPIO5, CW)  = t - tx + ty + tz
M3 FR(GPIO6, CCW) = t - tx - ty - tz
M4 RL(GPIO7, CCW) = t + tx + ty - tz
```

desaturation은 자세 차동 명령을 우선 보존하고 collective를 이동하는
기존 방식을 유지한다.

## 게인 단위 (UDP 명령은 기존과 동일한 문자)

| 명령 | 대상 | 단위 | 기본값 |
|---|---|---|---|
| `ap`/`ar`/`at` | 각도 P (roll/pitch) | (rad/s)/rad | 6.0 |
| `ay` | 각도 P (yaw) | (rad/s)/rad | 3.0 |
| `rp`,`pa` / `ri`,`ia` / `rd`,`da` | 각속도 PID (roll+pitch) | 토크/(rad/s) | 0.05 / 0.2 / 0.001 |
| `yp`,`py` / `yi`,`iy` / `yd`,`dy` | 각속도 PID (yaw) | 토크/(rad/s) | 0.3 / 0 / 0 |

**주의: `dual_imu_cascade_pwm`의 µs/(deg/s) 게인 값과 호환되지 않는다.**
기본값은 flix의 것으로, 기체 질량·프로펠러가 다르므로 낮은 스로틀 창에서
다시 튜닝해야 한다. 적분항 한계는 토크 0.3, D항 LPF는 약 40Hz다.

## 빌드

```bash
arduino-cli compile --warnings all --fqbn esp32:esp32:esp32s3 \
  --build-path /tmp/zetin-dual_imu_flix_quat_pwm \
  firmware/flight/dual_imu_flix_quat_pwm
```
