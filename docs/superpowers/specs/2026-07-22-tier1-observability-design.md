# Tier 1 관측성 — 설계 문서

작성일: 2026-07-22 · 대상 펌웨어: `firmware/flight/dual_imu_cascade_pwm`

## 1. 목적

현재 텔레메트리(21필드)로는 **PID 캐스케이드 루프 본체를 검증할 수 없다** —
모터 출력, 바깥루프가 안쪽루프에 주는 목표 각속도, 실제 루프 주파수가 전부
빠져 있어 "바깥루프 명령이 틀린 건지 / 안쪽루프 추종이 틀린 건지 / 믹서 매핑이
틀린 건지"를 구분할 수 없다. 이 관측 신호를 추가해 이후의 PID 튜닝·검증과
페일세이프 재설계(레벨+하강)의 전제를 만든다.

비목표(YAGNI): P/I/D 항 개별 분해·적분기 상태 노출은 이번 범위에서 제외한다
(요청 시 후속). 별도 고속 디버그 스트림도 만들지 않는다(상시 텔레메트리로 충분).

## 2. 추가 텔레메트리 필드 (21 → 29)

field 21 **뒤에 append**한다. append-only이므로 기존 파서·소비자를 깨지 않는다
(짧은 패킷은 새 필드가 `None`으로 남는다).

| # | 필드 | 타입 | 의미 |
|---|---|---|---|
| 22–25 | `Motor_M1`..`Motor_M4` | int | 실제 기록된 PWM µs (믹서 출력, disarmed 시 1000) |
| 26 | `PID_Loop_Hz` | int | pid_task 실측 루프 주파수 (정상 ≈ 1000) |
| 27–29 | `TgtRate_Roll/Pitch/Yaw` | float | 바깥루프 → 안쪽루프 목표 각속도 (dps) |

검증 논리: `TgtRate_*`(신규) + `Gyro_*`(기존, 실제 각속도) + `Motor_M*`(신규)로
바깥루프 명령 → 안쪽루프 추종 → 믹서 매핑 전 구간이 관측된다.

## 3. 펌웨어 변경 (`dual_imu_cascade_pwm.ino`)

상시(always-on), 디버그 플래그로 감싸지 **않는다**(지연 계측과 다름).

- **모터 출력 노출**: `pid_task`가 매 tick `volatile int motorOut[4]`에 `mix.motor[i]`를
  기록. `safety_lock`/`stopMotors()` 경로에서는 1000으로 채운다.
- **목표 각속도 노출**: `targetRateRoll/Pitch/Yaw`를 `volatile float tgtRate[3]`에 기록.
  잠금 시 0.
- **루프 주파수**: `pid_task`가 이터레이션 카운터를 증가시키고, `millis()` 기준
  ~1초마다 스냅샷 → `volatile int pidLoopHz`. (기존 `angleX` 등과 동일한 cross-core
  volatile 공유 패턴, 새 락 불필요.)
- **`sendTelemetry()`**: 위 8필드를 기존 21필드 뒤에 추가. `udp.printf` 포맷 확장.
- **게인 readback**: 새 명령 `gains` 수신 시 `GAINS,<12 floats>` UDP 패킷 1회 송신.
  순서: `Kp_Angle_Roll, Kp_Angle_Pitch, Kp_Angle_Yaw,
  Kp_Rate_Roll, Kp_Rate_Pitch, Kp_Rate_Yaw,
  Ki_Rate_Roll, Ki_Rate_Pitch, Ki_Rate_Yaw,
  Kd_Rate_Roll, Kd_Rate_Pitch, Kd_Rate_Yaw`. (게인 명령엔 지금까지 readback이
  없어 튜닝 명령 반영 확인이 불가했던 공백을 해소.)

팀원 워치독(`esp_task_wdt`)과의 상호작용: 추가는 볼래틸 저장 몇 개뿐이라
pid_task 타이밍에 무의미하다(워치독 500ms 여유 대비). 실기 재검증으로 확인한다.

## 4. `scripts/telemetry_schema.py` 리팩터

현재 "앞 9개 float, 나머지 int"(`FLOAT_FIELD_COUNT=9`) 규칙은 목표 각속도(float)가
정수 진단 필드 뒤에 오므로 성립하지 않는다. **필드별 명시적 타입맵**으로 일반화한다:

- `TELEMETRY_FIELDS`에 위 8필드를 순서대로 추가.
- 각 필드에 `float`/`int` 타입을 명시하는 매핑을 도입하고, `parse_telemetry_packet`이
  이를 참조하도록 변경. `FLOAT_FIELD_COUNT` 경계 규칙 제거.
- `REQUIRED_FIELD_COUNT = 10` 유지. 짧은(10/14/21) 패킷 하위호환 유지(새 필드 `None`).
- `GAINS,...` 패킷은 텔레메트리가 아니므로 `parse_telemetry_packet`에 넣지 않는다.
  수신 스크립트가 `GAINS` 접두사를 먼저 분리 처리한다(파싱 예외 방지).

## 5. 소비자 영향

- `receive_telemetry.py`(CSV): `CSV_FIELDS = ("Timestamp",) + TELEMETRY_FIELDS`,
  `sample_to_csv_row`이 `TELEMETRY_FIELDS`를 순회 → **자동 확장**. 변경 불필요.
- `control_dualsense.py`, `tune_pid.py`(팀원이 공유 파서 소비자로 전환): 새 필드가
  파서를 통해 자동 노출. 표시 추가는 선택.
- `monitor_telemetry.py`: 모터/목표각속도 표시 한 줄 추가(관측용, 선택적).
- `analyze_flight_log.py`: 새 CSV 컬럼은 자동 기록됨. 플롯 추가는 선택(후속).
- `docs/udp_protocol.md`: `gains` 명령과 확장 텔레메트리 필드를 문서에 추가(팀원이
  이미 명령 세트를 정리해 둠).

## 6. 테스트

- **호스트**: `tools/test_telemetry_schema.py` 확장 — 신규 29필드 파싱, 타입맵(정수/실수
  정확성), 하위호환(10/14/21 필드 → 새 필드 `None`), `GAINS` 접두사 분리.
- **네이티브**: 해당 없음(순수 제어수학 테스트는 텔레메트리 포맷과 무관). 단 sketch가
  컴파일되는지는 기존 native 빌드가 커버.
- **실기 (필수)**: 제어/텔레메트리 경로가 바뀌므로 40항목 + 재시동 소크 재실행.
  신규 필드 sanity: disarmed 시 `Motor_M*`=1000·`TgtRate_*`≈0, `PID_Loop_Hz`≈1000,
  armed+기울임 시 `TgtRate_*`가 각도 오차 방향으로 반응, `gains` 명령이 현재 게인
  echo. 팀원 워치독 오탐 없음 재확인.

## 7. 위험

- 하드웨어 검증된 상시 제어경로 수정 → **실기 재검증 필수**(§6).
- 텔레메트리 패킷 크기 증가(21→29필드): 20Hz UDP에서 대역폭 무시 가능.
- append-only·타입맵으로 하위호환 보장. `GAINS` 패킷은 접두사 분리로 파서 오염 방지.
