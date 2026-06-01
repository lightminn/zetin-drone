# ZETIN Drone — 프로젝트 개요 

> 이 문서는 레포 전체(펌웨어 / 제어 스크립트 / 문서 / 로그 / 커밋 히스토리)를 한 번에 훑고,
> **"무엇이 실제로 동작하고, 무엇이 아직 실험 단계인지"** 를 기준으로 정리한 안내서입니다.
> 이론 + 핵심 개념 + 짧은 코드 위주로, 큰 그림을 먼저 잡는 데 목적이 있습니다.

---

## 0. 먼저 알아야 할 단 한 가지 — 성숙도(Maturity)

이 프로젝트에서 **"진짜로 검증되어 믿고 쓸 수 있는 것"** 과 **"실험 중인 것"** 을 구분하는 게 가장 중요합니다.
시간을 잘못 쓰는 가장 흔한 경우가, 실험 코드를 "완성된 기능"으로 착각하고 거기서부터 디버깅을 시작하는 것입니다.

| 기능 | 상태 | 어디서 |
|---|---|---|
| **PWM으로 ESC/브러시리스 모터 속도 제어** | ✅ **확실히 동작** | `examples/PWM_TEST` |
| **IMU(ICM42670) raw 가속도/자이로 읽기** | ✅ **확실히 동작** | `examples/IMU_TEST_RAW`, `DUAL_IMU_RAW_TEST` |
| 지자기 센서(BMM350) 읽기 | ✅ 동작 | `examples/BMM_TEST` |
| WiFi(SoftAP) + UDP 통신 / 텔레메트리 / 로깅 | ✅ 동작 | PID 예제 + `scripts/` |
| 상보필터 자세 추정(roll/pitch 각도) | 🟡 동작하나 검증 한정적 | PID 예제 안에 포함 |
| DShot 디지털 모터 프로토콜 | 🟡 신호는 나가나 통합 안 됨 | `examples/DSHOT_*` |
| **PID로 실제 드론 자세 안정화 / 호버링** | 🔴 **전부 실험적, 안정 비행 미달성** | `examples/*PID*`, `KALMAN_*`, `DUAL_LOOP_TEST` |
| 듀얼 IMU 융합 / 칼만 필터 / GPS / 초음파 고도 | 🔴 실험·미완 | 해당 예제들 |

> **결론:** 모터를 원하는 세기로 돌리는 것과 센서값을 읽는 것은 신뢰해도 됩니다.
> 하지만 "드론이 스스로 수평을 잡는다"는 부분은 **아직 연구 단계**이며, 비행 영상/로그가 있어도 "튜닝이 성공한 한 순간"일 뿐 일반적으로 안정적이지 않습니다.

---

## 1. 프로젝트 전체 구조 (큰 그림)

```
┌─────────────────────────┐         WiFi (ESP32 SoftAP)          ┌──────────────────────────┐
│   드론 (ESP32-S3)        │  ◀── UDP 4210, 텔레메트리/명령 ──▶   │   PC (Python scripts)     │
│                         │                                      │                          │
│  IMU ×2 (SPI)           │   명령:  rc / start / stop / th /    │  Drone_Control_Dualsense │
│  지자기 BMM350 (I2C)    │          PID 게인(pp,dp,pr,dr...)     │  Drone_Tuning (콘솔)      │
│  ESC ×4 → 브러시리스모터 │                                      │  Drone_Monitor / Analasys │
│  (US100 초음파, GPS)    │   텔레메트리: 14필드 CSV 한 줄       │  → logs/*.csv 로 기록     │
└─────────────────────────┘                                      └──────────────────────────┘
```

- **펌웨어(드론 쪽)**: ESP32-S3 + Arduino 프레임워크. 센서를 읽어 자세를 추정하고, PID로 모터 출력을 계산해 ESC에 PWM을 보냅니다.
- **스크립트(PC 쪽)**: 드론을 조종(컨트롤러/콘솔)하고, 실시간 모니터링하고, 비행 후 로그를 분석/튜닝합니다.
- **통신**: 드론이 WiFi **AP(공유기)** 역할을 하고(`SSID: Drone_Tuning`, `PW: 12345678`), PC가 거기 접속해 **UDP**로 주고받습니다. 드론 IP는 항상 `192.168.4.1`.

### 디렉토리 맵
```
firmware/
  src/      ← ⚠️ 사실상 버려진 스켈레톤 (아래 "함정" 참고). 진짜 코드 아님.
  lib/      ← 위 src용 헤더 (motor/sensor/connect). 역시 미사용에 가까움.
  examples/ ← ★ 실제 작동/실험 코드는 전부 여기. 폴더당 .ino 한 개.
  platformio_config/platformio.ini
scripts/    ← PC측 Python (조종/튜닝/모니터/분석/수신)
logs/       ← 비행 CSV 로그
docs/       ← 설계 spec / 구현 plan / 발표자료 / (이 문서)
test/       ← TCP 통신 테스트 등
```

> **중요 함정 1:** `firmware/src/main.cpp`, `sensor.cpp`, `motor.cpp`는 초기 골격입니다.
> - `sensor.cpp`의 센서값은 **실제 센서가 아니라 `rand()`로 만든 시뮬레이션**입니다.
> - `motor.cpp` 맨 첫 줄에 `// this motor control system has been canceled` — 최소자승 기반 전압배분 실험인데 **폐기됨**.
> - `connect.cpp`는 집 공유기에 붙는 **TCP** 방식(옛날 버전). 지금 비행 코드는 **SoftAP + UDP**.
>
> 코드를 처음 읽을 때는 **`firmware/examples/` 안의 `.ino` 파일들**을 보세요. `src/`는 무시해도 됩니다.

---

## 2. 하드웨어 구성

| 부품 | 모델 | 인터페이스 | 비고 |
|---|---|---|---|
| MCU | **ESP32-S3** (DevKitC-1) | — | 듀얼코어 + WiFi 내장 |
| IMU(관성센서) | **ICM-42670-P** ×2 | SPI | 가속도계+자이로. 듀얼은 노이즈/고장 대비용 |
| 지자기 | BMM350 | I2C | 방위(yaw) 보정용, 아직 융합 안 함 |
| 모터 구동 | ESC ×4 + 브러시리스 모터 ×4 | PWM(또는 DShot) | 쿼드콥터 X자 배치 |
| (옵션) 초음파 | US100 | UART/펄스 | 고도, 실험 |
| (옵션) GPS | **Adafruit Ultimate GPS** (MTK3339 / PA6H) | UART(NMEA) | 위치, 실험 |

> **GPS 메모:** Adafruit Ultimate GPS는 표준 **NMEA / UART, 기본 9600 baud** 모듈입니다.
> 현재 `examples/GPS_TEST_RAW`가 이미 이 방식(9600 NMEA + `EN` 전원핀 HIGH + 빨간 FIX LED 확인)으로 짜여 있어 **그대로 호환**됩니다.
> 핀: `GPS_RX=44, GPS_TX=43, GPS_EN=40` (ESP32 RX↔GPS TX 교차 연결). FIX LED는 미수신 시 ~1초 깜빡, 위성 고정(fix) 시 ~15초에 한 번 깜빡입니다.
> 파싱은 보통 **TinyGPS++** 라이브러리를, 업데이트 주기/baud 변경은 **PMTK** 명령을 씁니다.

### IMU 핀(현재 비행 코드 기준)
```
SPI : SCK=12, MISO=13, MOSI=11
IMU1 CS=10,  IMU2 CS=9
모터 PWM : M1=4(FL), M2=5(RR), M3=6(FR), M4=7(RL)
```

---

## 3. 드론 제어 이론 (이 프로젝트에 필요한 만큼만)

### 3.1 쿼드콥터는 어떻게 나는가
- 모터 4개가 **X자**로 배치됩니다. 대각선끼리 같은 방향으로 돌고(예: CW), 나머지 대각선은 반대(CCW).
  → 반대로 도는 쌍이 있어야 기체가 토크로 빙글빙글 돌지 않습니다(yaw 반작용 상쇄).
- 조종은 4가지 양을 모터 출력에 더하고 빼서 만듭니다:
  - **Throttle(상승력)**: 4개 모두 ↑ → 전체 추력 ↑
  - **Roll(좌우 기울기)**: 한쪽 두 개 ↑, 반대쪽 두 개 ↓
  - **Pitch(앞뒤 기울기)**: 앞 두 개 ↑, 뒤 두 개 ↓
  - **Yaw(제자리 회전)**: CW 모터 쌍과 CCW 모터 쌍의 균형을 깸

### 3.2 모터 믹싱(Mixing)
PID가 계산한 roll/pitch/yaw 보정량을 각 모터에 분배하는 식입니다. 현재 코드(`PWM_TEST_DUAL_IMU_PID`):
```cpp
// base_throttle: 공통 추력,  pid_roll/pitch/yaw: 자세 보정량
M1(FL) = base - pitch + roll - yaw
M2(RR) = base + pitch - roll - yaw
M3(FR) = base - pitch - roll + yaw
M4(RL) = base + pitch + roll + yaw
```
믹싱의 **부호(±)는 모터 위치/회전방향과 반드시 일치**해야 합니다. 부호 하나만 틀려도 드론이 즉시 뒤집힙니다.

### 3.3 ESC와 모터 신호 — ✅ 여기서부터가 "확실히 되는 것"
ESC(전자변속기)는 MCU의 신호를 받아 브러시리스 모터의 회전 속도를 만듭니다.
가장 표준적인 방식은 **RC 서보 PWM**: 50Hz 주기에서 펄스 폭으로 명령합니다.
- `1000µs` = 0%(정지),  `2000µs` = 100%

```cpp
// examples/PWM_TEST — ESP32Servo 라이브러리로 가장 단순하게
motor1.setPeriodHertz(50);
motor1.attach(pin, 1000, 2000);
motor1.writeMicroseconds(1300);   // ≈ 30% 출력
```
PID 예제들은 더 빠른 갱신을 위해 ESP32 LEDC 하드웨어 PWM(400Hz, 14bit)을 직접 씁니다:
```cpp
ledcAttach(pin, 400, 14);
// us(1000~2000) → duty 로 환산해서 출력
uint32_t duty = (us * 16383UL) / 2500;   // 2500us 주기 기준
ledcWrite(pin, duty);
```
> **ESC 아밍(Arming):** 전원 인가 후 일정 시간 "0% 신호(1000µs)"를 줘야 ESC가 켜지며 "삐-삐" 소리를 냅니다.
> 이 과정 없이 바로 명령하면 위험하므로 `PWM_TEST`는 일부러 **7초 카운트다운** 후 시작합니다.

> **DShot(실험):** PWM 대신 디지털로 스로틀 값을 보내는 방식(`examples/DSHOT_*`). 신호 자체는 나가지만 비행 코드에 통합되지 않았습니다.

### 3.4 IMU와 자세 추정 — ✅ raw 읽기까지가 "확실히 되는 것"
**IMU = 가속도계 + 자이로**. 둘 다 단독으로는 자세각을 정확히 주지 못합니다.

| 센서 | 측정 | 장점 | 단점 |
|---|---|---|---|
| 가속도계 | 중력 방향(기울기) | 장기적으로 정확(드리프트 없음) | 진동/가속에 매우 민감(순간값 노이즈↑) |
| 자이로 | 각속도(°/s) | 순간 변화에 정확·매끄러움 | 적분하면 오차 누적(드리프트) |

**raw 읽기 (검증됨):**
```cpp
// examples/IMU_TEST_RAW — ICM42670, SPI
IMU.startAccel(1600, 16);    // ODR 1600Hz, ±16g
IMU.startGyro(1600, 2000);   // ODR 1600Hz, ±2000 dps
IMU.getDataFromRegisters(e);
float ax = e.accel[0] / 2048.0;  // ±16g → g 단위
float gx = e.gyro[0]  / 16.4;    // ±2000dps → °/s 단위
```

**상보필터(Complementary Filter)** — 두 센서의 장점만 융합 (🟡 동작하나 비행 검증은 한정적):
```cpp
// accel로 계산한 "절대 각도" + gyro 적분으로 만든 "변화량" 을 가중 결합
accAngle = atan2(ay, sqrt(ax*ax + az*az)) * 180/PI;     // 가속도 기반 기울기
angle = α * (angle + gyro*dt) + (1-α) * accAngle;        // α ≈ 0.99~0.999
```
- α가 1에 가까울수록 자이로를 더 신뢰(매끄럽지만 천천히 드리프트 보정).
- 이 프로젝트는 **가속도 크기에 따라 α를 바꾸는 "적응형 상보필터"** 를 실험합니다:
  정지/호버링이면 α를 낮춰 가속도로 적극 보정, 급기동/진동이면 α를 1에 가깝게 해 자이로만 신뢰.
- (칼만 필터 버전도 `KALMAN_TEST`에 있으나 실험 단계.)

### 3.5 PID 제어 — 🔴 전부 실험적
목표 각도(`target`)와 현재 각도(`angle`)의 오차를 줄이도록 모터 보정량을 만드는 고전 제어기:
```
출력 = Kp·(오차) + Ki·∫(오차) + Kd·(오차의 변화율)
```
- **P (비례)**: 오차에 비례해 즉시 반응. 너무 크면 진동.
- **I (적분)**: 작은 정상상태 오차를 시간을 두고 제거. 너무 크면 흔들림 누적(windup).
- **D (미분)**: 변화 속도에 제동을 걸어 오버슈트/진동 억제. 보통 자이로(각속도)를 그대로 D항에 사용.

실제 코드(각 축 한 줄):
```cpp
pid_roll = errorRoll*Kp_Roll + errorSumRoll*Ki_Roll - lpf_gx*Kd_Roll;
//          └P항────────────┘  └I항(적분누적)──────┘  └D항=각속도에 제동─┘
```
현재 기본 게인: `Kp=2.5, Ki=0.005, Kd=1.2` (roll/pitch), yaw는 대부분 꺼둠.

> **왜 아직 "실험적"인가:** 자세 추정 오차, 진동, 센서 드리프트, 모터/프레임 비대칭, 게인 튜닝이 전부 맞물려야 안정 비행이 됩니다.
> 이 프로젝트는 **반복적으로 한쪽으로 쏠리는 드리프트**와 **기울기가 커지면 발산해 뒤집히는 문제**를 겪었고, 그걸 해결하려는 시도들이 곧 `examples/`의 여러 PID 변형들입니다.

---

## 4. 비행 펌웨어 아키텍처 (대표: `PWM_TEST_DUAL_IMU_PID`)

ESP32-S3의 듀얼코어 + FreeRTOS를 이렇게 나눠 씁니다:
```
Core 1 : pid_task()   — 1kHz 메인 제어 루프 (센서읽기→자세추정→PID→모터)
Core 0 : udp_task()   — PC 명령 수신 (rc/start/stop/게인...)
Core 1 : loop()       — 50ms마다 텔레메트리 14필드 전송
```
한 사이클 흐름:
```
IMU1·IMU2 읽기 → (IMU2 축부호 보정 + 바이어스 차감) → 두 IMU 평균 → LPF
   → 상보필터로 angleX/Y(롤/피치), 자이로 적분으로 angleZ(요)
   → 안전검사(RC 끊김 / 과도한 기울기 / IMU 멈춤)  → 걸리면 모터 정지(safety_lock)
   → PID 계산 → 모터 믹싱 → 4개 ESC에 PWM
```

### 안전장치(Fail-safe) — 반드시 이해할 것
- `start` 명령 전에는 항상 `safety_lock = true` (모터 안 돎).
- RC 패킷 500ms 이상 끊기면 정지(`check_rc_timeout`).
- roll/pitch가 ±45° 넘으면 정지(`check_attitude`).
- IMU 값이 멈추면(frozen) 정지. 듀얼이면 한쪽 고장 시 멀쩡한 쪽으로 fallback.

### 통신 프로토콜 (PC ↔ 드론)
**PC → 드론 명령(UDP 문자열):**
| 명령 | 의미 |
|---|---|
| `start` / `stop` | 시동(ARM) / 비상정지(DISARM) |
| `rc <seq> <roll> <pitch> <yaw>` | 목표 자세각(시퀀스 번호 포함) |
| `th <값>` | 기본 스로틀(예: `th 1150`) |
| `yaw <0/1>` | yaw 제어 on/off |
| `pp/dp/ip`, `pr/dr/ir`, `pa/da/ia`, `py/dy/iy <값>` | PID 게인 실시간 튜닝(p=pitch, r=roll, a=양쪽공통, y=yaw) |

**드론 → PC 텔레메트리(콤마 14필드, 한 줄):**
```
angleX, angleY, angleZ, gx, gy, gz, ax, ay, az, base_throttle,
fault_rc, fault_imu, rcTotalPkts, rcDroppedPkts
```

---

## 5. PC측 스크립트 (`scripts/`)

| 파일 | 역할 |
|---|---|
| `Drone_Control_Dualsense.py` | **PS5 듀얼센스 컨트롤러로 조종**. 스틱→목표각(±15°), 버튼→start/stop, 트리거→스로틀. `rc/th/start/stop` 송신 + 텔레메트리 수신 |
| `Drone_Tuning.py` | **콘솔에서 명령 직접 타이핑**. PID 게인 튜닝/시동에 사용(가장 단순) |
| `Drone_Monitor.py` | 실시간 센서/상태 모니터링 |
| `Drone_Analasys.py` | `logs/*.csv` 불러와 그래프로 분석 |
| `Drone_Reciever.py` / `dual_imu_pid_debug_receiver.py` | 텔레메트리 수신·기록(디버그) |
| `Controller_test.py` | 컨트롤러 입력 정상 확인 |
| `GPS_Reciever.py` | GPS 수신 테스트 |

설치: `pip install pygame pandas matplotlib`

---

## 6. 개발 환경 / 빌드 / 실행

- **펌웨어 빌드**: PlatformIO(VS Code) 또는 `arduino-cli`.
  - 보드: `esp32-s3-devkitc-1`, 프레임워크: Arduino
  - 의존 라이브러리: ICM42670P(IMU), DShotRMT(DShot), DFRobot_BMM350(지자기), ESP32Servo(PWM) 등
  - 예제 컴파일:
    ```bash
    arduino-cli compile --fqbn esp32:esp32:esp32s3 firmware/examples/PWM_TEST/
    ```
- **각 예제는 독립 `.ino`** 입니다. 하나 골라 빌드→업로드→시리얼 모니터(115200) 확인.
- **조종 흐름**: 드론 켜기 → PC를 `Drone_Tuning`(pw `12345678`) WiFi에 연결 → `scripts/Drone_Tuning.py` 실행 → `th 1150` → `start`.

---

## 7. 처음 해볼 순서 (추천)

1. **`PWM_TEST`** 업로드 → (프로펠러 빼고) 모터가 30%까지 부드럽게 오르내리는지 확인. *모터 제어 감 잡기.*
2. **`IMU_TEST_RAW`** 업로드 → 시리얼 플로터로 가속도/자이로 raw값 보기. 기체를 기울이면 어떤 축이 변하는지 손으로 확인. *센서 감 잡기.*
3. **`DUAL_IMU_RAW_TEST`** → IMU 두 개의 축 방향/부호가 어떻게 다른지 직접 관찰. *좌표계가 왜 까다로운지 체감.*
4. 그 다음에야 **`PWM_TEST_DUAL_IMU_PID`** 의 코드를 읽으며 위 3.4~3.5 이론과 매칭. 비행은 **반드시 안전장치/프로펠러 제거/테스트 리그** 위에서.

---

## 8. 알려진 함정과 교훈 (실제로 우리가 겪은 것들)

이 부분이 처음 보는 사람에게 가장 값집니다. 같은 함정을 반복하지 마세요.

1. **`firmware/src/`는 함정.** 진짜 코드는 `examples/`. `sensor.cpp`는 시뮬레이션, `motor.cpp`는 폐기됨, `connect.cpp`는 옛 TCP 방식.
2. **IMU 축 부호 문제.** IMU2는 IMU1 대비 **X, Z축 부호가 반대**(Y는 동일). raw 읽은 직후 `IMU2_SIGN_{X,Y,Z}`로 보정해 같은 기체 좌표계로 맞춘 뒤에 평균/융합해야 합니다. 안 맞추면 두 센서가 서로 상쇄됩니다.
3. **자이로 바이어스 드리프트 = yaw가 슬금슬금 도는 원인.** 부팅 시 2초간 정지 상태에서 자이로 평균을 측정해 **바이어스로 빼줍니다**(`calibrate_bias`). 캘리브레이션 중에는 절대 기체를 건드리지 말 것.
4. **자세 추정/믹싱 부호는 검증 끝남.** `DUAL_IMU_PID_DEBUG`로 손으로 확인했습니다:
   - `angleX(+)` = 오른쪽 날개 아래로(우로 롤), `angleY(+)` = 기수 아래로(앞으로 피치), `angleZ(+)` = 위에서 봤을 때 시계방향 yaw.
   - 그러니 비행이 이상할 때 **추정/믹싱 부호를 먼저 의심하지 말고**, 게인·진동·드리프트를 보세요.
5. **전원 극성 확인.** 코드 수정 후에는 손으로 기체를 잡고 자세 안정화가 제대로 되는지 확인한 뒤 이륙할 것.

### 더 읽을 거리 (레포 내부)
- 설계 의도/근거: `docs/superpowers/specs/2026-05-14-dual-imu-pid-design.md`
- 단계별 구현 기록: `docs/superpowers/plans/2026-05-14-dual-imu-pid.md`
- 각 디렉토리별 `README.md` (firmware / scripts / logs / test)
- 비행 로그: `logs/*.csv` → `scripts/Drone_Analasys.py`로 시각화

---

## 9. 한 문단 요약

ZETIN 드론은 **ESP32-S3** 위에서 **IMU로 자세를 추정 → PID로 모터 보정량을 계산 → ESC에 PWM** 을 보내고,
PC가 **WiFi/UDP**로 조종·튜닝·로깅하는 구조입니다.
**모터 PWM 제어와 IMU raw 읽기는 신뢰할 수 있는 기반**이고, **그 위의 PID 자세 안정화는 아직 실험·튜닝 단계**입니다.
코드를 볼 때는 `firmware/examples/`만 보고(`src/`는 무시), 비행 관련은 항상 프로펠러를 빼고 안전장치를 켠 채로 다루세요.
