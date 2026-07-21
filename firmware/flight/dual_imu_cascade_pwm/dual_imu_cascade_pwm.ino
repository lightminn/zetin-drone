#include <Arduino.h>
#include <SPI.h>
#include <ICM42670P.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_task_wdt.h>

// ==========================================================
// 1. 튜닝 파라미터
// ==========================================================
// [Outer] 각도 P: 각도 오차 -> 목표 각속도(dps)
volatile float Kp_Angle_Roll  = 6.0f;
volatile float Kp_Angle_Pitch = 6.0f;
volatile float Kp_Angle_Yaw   = 3.0f;

// [Inner] 각속도 PID: 각속도 오차 -> 모터 파워
volatile float Kp_Rate_Roll  = 0.50f, Ki_Rate_Roll  = 0.005f, Kd_Rate_Roll  = 0.015f;
volatile float Kp_Rate_Pitch = 0.50f, Ki_Rate_Pitch = 0.005f, Kd_Rate_Pitch = 0.015f;
volatile float Kp_Rate_Yaw   = 1.50f, Ki_Rate_Yaw   = 0.05f,  Kd_Rate_Yaw   = 0.0f;

volatile int  base_throttle = 1000;
volatile int  min_throttle  = 1050;
volatile int  max_throttle  = 1300;
volatile bool yaw_enabled   = false;   // "yaw 1" 로 활성화

// ==========================================================
// 2. 시스템 상수
// ==========================================================
const char* WIFI_SSID    = "Drone_Tuning";
const char* WIFI_PASS    = "12345678";
const int   UDP_PORT     = 4210;
const int   WIFI_CHANNEL = 6;

const int pinM1   = 4;   // FL
const int pinM2   = 5;   // RR
const int pinM3   = 6;   // FR
const int pinM4   = 7;   // RL
const int SPI_CS1 = 10;  // IMU1
const int SPI_CS2 = 9;   // IMU2

const int ESC_FREQ   = 400;
const int ESC_RES    = 14;
const int ESC_PERIOD = 2500;                 // us
const int ESC_MAXDUTY = (1 << ESC_RES) - 1;  // magic number 제거

const float GYRO_SCALE   = 1.0f / 16.4f;     // raw -> deg/s
const float ACCEL_SCALE  = 1.0f / 2048.0f;   // raw -> g
const float SAFETY_ANGLE = 60.0f;
const float YAW_DEADZONE = 0.3f;
const float MAX_TARGET_ANGLE_RP = 30.0f;     // UDP 오입력에 대한 자세 명령 제한
const float MAX_TARGET_RATE_RP  = 300.0f;    // outer-loop 출력 제한 (deg/s)
const float MAX_TARGET_RATE_YAW = 180.0f;

// IMU2는 x,z축이 IMU1 대비 반전 (y는 동일). 첫 파일 기준 그대로.
const float IMU2_SIGN_X = -1.0f;
const float IMU2_SIGN_Y =  1.0f;
const float IMU2_SIGN_Z = -1.0f;
static const float IMU1_SIGN[3] = { 1.0f, 1.0f, 1.0f };
static const float IMU2_SIGN[3] = { IMU2_SIGN_X, IMU2_SIGN_Y, IMU2_SIGN_Z };

// --- 안전/redundancy 임계값 (하드웨어 맞춰 튜닝 필요) ---
const uint32_t RC_TIMEOUT_MS      = 500;
const uint32_t PID_WDT_TIMEOUT_MS = 500;     // pid_task 정지 시 강제 재부팅 (1kHz 루프 대비 큰 여유)
const int32_t  FROZEN_DELTA_RAW   = 1;       // 6축 raw 변화량 합이 이 이하면 정지 의심 (LSB)
const uint32_t IMU_FROZEN_MS      = 300;     // 그 상태가 이만큼 지속되면 freeze 확정
const float    GYRO_DISAGREE_DPS  = 15.0f;   // 두 IMU 각속도 차이 임계 (deg/s)
const uint32_t IMU_DISAGREE_MS    = 150;     // 불일치 지속 시간

const int   BIAS_CALIB_SAMPLES   = 2000;
const float BIAS_MOVEMENT_THRESH = 1.0f;     // 축별 stddev 상한 (deg/s)
const int   BIAS_CALIB_RETRIES   = 3;

const int OUTER_DIV = 4;                      // outer loop = 1kHz / 4 = 250Hz

// ==========================================================
// 3. 유틸
// ==========================================================
class LowPassFilter {
public:
  float alpha, last = 0.0f;
  LowPassFilter(float cutoff_hz, float dt) {
    float rc = 1.0f / (2.0f * PI * cutoff_hz);
    alpha = dt / (rc + dt);
  }
  float update(float in) { last += alpha * (in - last); return last; }
  void reset(float value = 0.0f) { last = value; }
};

// Arduino sketch preprocessor의 자동 함수 원형보다 먼저 보여야 하는 반환 타입.
struct MotorMix {
  int motor[4];
  bool scaled;
};

// IMU별 freeze 감시 (raw 레지스터 값이 멈췄는지)
struct FreezeMon {
  int16_t  lastGyro[3] = {0, 0, 0};
  int16_t  lastAccel[3] = {0, 0, 0};
  uint32_t since = 0;
  bool     init  = false;
};
static bool checkFreeze(FreezeMon &m, const inv_imu_sensor_event_t &e, uint32_t nowMs) {
  if (!m.init) {
    for (int k = 0; k < 3; k++) {
      m.lastGyro[k] = e.gyro[k];
      m.lastAccel[k] = e.accel[k];
    }
    m.init = true;
    return false;
  }

  // abs(x)+abs(y)+abs(z)의 변화만 보면 축 변화가 서로 상쇄될 수 있다.
  // 각 레지스터의 변화량을 직접 더해 통신 정지/동일 프레임 반복을 감시한다.
  int32_t delta = 0;
  for (int k = 0; k < 3; k++) {
    delta += abs((int32_t)e.gyro[k] - m.lastGyro[k]);
    delta += abs((int32_t)e.accel[k] - m.lastAccel[k]);
  }

  bool frozen = false;
  if (delta <= FROZEN_DELTA_RAW) {
    if (m.since == 0) m.since = nowMs;
    else if (nowMs - m.since >= IMU_FROZEN_MS) frozen = true;
  } else {
    m.since = 0;
  }
  for (int k = 0; k < 3; k++) {
    m.lastGyro[k] = e.gyro[k];
    m.lastAccel[k] = e.accel[k];
  }
  return frozen;
}

// ==========================================================
// 4. 시스템 변수
// ==========================================================
WiFiUDP   udp;
char      packetBuffer[256];
IPAddress laptopIP;
int       laptopPort            = 0;
bool      connectionEstablished = false;

ICM42670 IMU1(SPI, SPI_CS1);
ICM42670 IMU2(SPI, SPI_CS2);

volatile bool  safety_lock  = true;
volatile float targetAngleX = 0.0f, targetAngleY = 0.0f, targetAngleZ = 0.0f;

volatile float angleX = 0.0f, angleY = 0.0f, angleZ = 0.0f; // 추정 각도
volatile float gyroX  = 0.0f, gyroY  = 0.0f, gyroZ  = 0.0f; // 융합 각속도 (body frame)
volatile float accX   = 0.0f, accY   = 0.0f, accZ   = 0.0f; // 융합 가속도 (g)

float errorSumRoll = 0.0f, errorSumPitch = 0.0f, errorSumYaw = 0.0f;

float gyro_bias1[3] = {0,0,0};
float gyro_bias2[3] = {0,0,0};

volatile uint32_t lastRcMs        = 0;
volatile bool     fault_rc        = false;
volatile bool     fault_imu1      = false;   // IMU1 freeze
volatile bool     fault_imu2      = false;   // IMU2 freeze
volatile bool     fault_disagree  = false;   // 두 IMU 불일치 (중재 불가)
volatile bool     fault_attitude  = false;   // 과도 기울기
volatile int      active_imus     = 2;       // 현재 사용 중인 IMU 수 (telemetry)
volatile bool     mixer_scaled    = false;   // 자세 mixer가 축소됐는지
volatile bool     calibration_ok  = false;

// 재시동 판단용 현재 센서 상태. fault_imu* / fault_disagree는 비행 중 latch된다.
volatile bool     imu1_frozen_now = false;
volatile bool     imu2_frozen_now = false;
volatile bool     imu_disagree_now = false;

// scripts/control_dualsense.py 프로토콜 호환용
volatile uint32_t lastRcSeq       = 0;
volatile uint32_t rcTotalPkts     = 0;
volatile uint32_t rcDroppedPkts   = 0;

// ==========================================================
// 5. 모터
// ==========================================================
void writeMotor(int pin, int us) {
  us = constrain(us, 1000, 2000);
  uint32_t duty = ((uint32_t)us * ESC_MAXDUTY) / ESC_PERIOD;
  ledcWrite(pin, duty);
}
void stopMotors() {
  writeMotor(pinM1, 1000); writeMotor(pinM2, 1000);
  writeMotor(pinM3, 1000); writeMotor(pinM4, 1000);
}

// 자세 차동 명령을 먼저 보존하고 collective를 이동한다. 그래도 범위를 넘을 때만
// 모든 자세 명령을 같은 비율로 축소해 토크 비율을 유지한다.
static MotorMix mixAndDesaturate(float roll, float pitch, float yaw,
                                 int throttle, int minMotor, int maxMotor) {
  MotorMix out;
  minMotor = constrain(minMotor, 1000, 2000);
  maxMotor = constrain(maxMotor, minMotor, 2000);

  float diff[4] = {
    -pitch + roll - yaw,  // M1: FL
     pitch - roll - yaw,  // M2: RR
    -pitch - roll + yaw,  // M3: FR
     pitch + roll + yaw   // M4: RL
  };

  float minDiff = diff[0], maxDiff = diff[0];
  for (int i = 1; i < 4; i++) {
    minDiff = min(minDiff, diff[i]);
    maxDiff = max(maxDiff, diff[i]);
  }

  const float available = (float)(maxMotor - minMotor);
  const float span = maxDiff - minDiff;
  float scale = 1.0f;
  if (span > available && span > 0.0f) scale = available / span;
  out.scaled = scale < 0.9999f;

  if (out.scaled) {
    for (int i = 0; i < 4; i++) diff[i] *= scale;
    minDiff *= scale;
    maxDiff *= scale;
  }

  const float collectiveLo = minMotor - minDiff;
  const float collectiveHi = maxMotor - maxDiff;
  const float collective = min(max((float)throttle, collectiveLo), collectiveHi);

  for (int i = 0; i < 4; i++) {
    out.motor[i] = constrain((int)lroundf(collective + diff[i]), minMotor, maxMotor);
  }
  return out;
}

// ==========================================================
// 6. Gyro bias 캘리브레이션
// ==========================================================
// sign[]은 각 IMU를 drone(IMU1) frame으로 맞추는 부호.
static bool measure_imu_bias(ICM42670 &imu, const float sign[3],
                             float bias_out[3], float sd_out[3]) {
  double sum[3] = {0,0,0}, sum_sq[3] = {0,0,0};
  inv_imu_sensor_event_t e = {};
  int samples = 0;
  int attempts = 0;
  const int maxAttempts = BIAS_CALIB_SAMPLES + 100;
  while (samples < BIAS_CALIB_SAMPLES && attempts < maxAttempts) {
    attempts++;
    if (imu.getDataFromRegisters(e) != 0) {
      delayMicroseconds(1000);
      continue;
    }
    float g[3] = {
      sign[0] * e.gyro[0] * GYRO_SCALE,
      sign[1] * e.gyro[1] * GYRO_SCALE,
      sign[2] * e.gyro[2] * GYRO_SCALE
    };
    for (int k = 0; k < 3; k++) { sum[k] += g[k]; sum_sq[k] += (double)g[k]*g[k]; }
    samples++;
    delayMicroseconds(1000);
  }
  if (samples != BIAS_CALIB_SAMPLES) return false;
  for (int k = 0; k < 3; k++) {
    double mean = sum[k] / samples;
    double var  = sum_sq[k] / samples - mean*mean;
    if (var < 0) var = 0;
    bias_out[k] = (float)mean;
    sd_out[k]   = (float)sqrt(var);
  }
  return true;
}

static bool calibrate_bias() {
  float sd1[3], sd2[3];
  for (int a = 1; a <= BIAS_CALIB_RETRIES; a++) {
    Serial.printf("[CALIB] attempt %d/%d (hold still)...\n", a, BIAS_CALIB_RETRIES);
    bool read1 = measure_imu_bias(IMU1, IMU1_SIGN, gyro_bias1, sd1);
    bool read2 = measure_imu_bias(IMU2, IMU2_SIGN, gyro_bias2, sd2);
    if (!read1 || !read2) {
      Serial.printf("[CALIB] sensor read failed (imu1=%d imu2=%d)\n", (int)read1, (int)read2);
      continue;
    }
    float max_sd = max(max(max(sd1[0],sd1[1]),sd1[2]),
                       max(max(sd2[0],sd2[1]),sd2[2]));
    Serial.printf("[CALIB] IMU1 %.3f %.3f %.3f | IMU2 %.3f %.3f %.3f (maxSD %.3f)\n",
                  gyro_bias1[0],gyro_bias1[1],gyro_bias1[2],
                  gyro_bias2[0],gyro_bias2[1],gyro_bias2[2], max_sd);
    if (max_sd <= BIAS_MOVEMENT_THRESH) { Serial.println("[CALIB] OK"); return true; }
    Serial.println("[CALIB] movement detected, retry");
  }
  Serial.println("[CALIB] FAIL: reboot and calibrate on a stationary surface");
  return false;
}

// ==========================================================
// 7. 자세 추정용 적응 alpha (가속도 신뢰도 기반)
// ==========================================================
const float ACC_DEV_SOFT = 0.10f, ACC_DEV_HARD = 0.30f;
const float ALPHA_STATIC = 0.99f, ALPHA_NORMAL = 0.995f, ALPHA_DYN = 0.999f;
static inline float compute_alpha(float ax, float ay, float az) {
  float dev = fabsf(sqrtf(ax*ax+ay*ay+az*az) - 1.0f);
  if (dev < ACC_DEV_SOFT) return ALPHA_STATIC;
  if (dev < ACC_DEV_HARD) return ALPHA_NORMAL;
  return ALPHA_DYN;
}

// ==========================================================
// 8. PID 태스크 (Core 1, 1kHz)
// ==========================================================
void pid_task(void *pv) {
  // 모터 정지 수단(stopMotors, RC timeout 검사)이 모두 이 태스크 안에 있으므로,
  // SPI 행업 등으로 태스크가 블로킹되면 모터가 마지막 PWM으로 고정된다.
  // 태스크 워치독이 panic 재부팅을 강제하고, 재부팅 후 ESC는 PWM 신호
  // 소실로 정지하며 setup()의 stopMotors()가 정지 신호를 복원한다.
  esp_task_wdt_add(NULL);

  const TickType_t period = pdMS_TO_TICKS(1);   // vTaskDelayUntil 기반 (busy-wait 제거)
  TickType_t wake = xTaskGetTickCount();
  const float dt = 0.001f;

  LowPassFilter lpfD_Roll(40, dt), lpfD_Pitch(40, dt), lpfD_Yaw(40, dt);
  float prevGyroX = 0.0f, prevGyroY = 0.0f, prevGyroZ = 0.0f;

  FreezeMon fm1, fm2;
  uint32_t disagreeSince = 0;

  float targetRateRoll = 0, targetRatePitch = 0, targetRateYaw = 0;
  uint8_t outerCnt = 0;
  uint32_t lastMicros = micros();
  bool wasLocked = true;

  inv_imu_sensor_event_t e1 = {}, e2 = {};

  while (true) {
    vTaskDelayUntil(&wake, period);
    esp_task_wdt_reset();
    TickType_t afterWake = xTaskGetTickCount();
    if ((TickType_t)(afterWake - wake) > period) {
      // 큰 지연 뒤 밀린 tick을 연속 실행하지 않는다(센서 stale read/D항 spike 방지).
      wake = afterWake;
    }
    uint32_t nowMs = millis();
    uint32_t nowUs = micros();
    float realDt = (nowUs - lastMicros) / 1e6f;
    lastMicros = nowUs;
    // 지연 후 vTaskDelayUntil이 catch-up할 때 지나치게 작은 dt로 D항이 튀는 것을 방지.
    if (realDt < 0.0002f || realDt > 0.01f) realDt = dt;

    // ---------- 센서 읽기 ----------
    int readStatus1 = IMU1.getDataFromRegisters(e1);
    int readStatus2 = IMU2.getDataFromRegisters(e2);

    // 읽기 오류 시 event가 갱신되지 않으므로 동일 프레임으로 취급되어 freeze로 귀결된다.
    // fault는 비행 중 자동 복구하지 않고 다음 수동 start 때만 재평가/해제한다.
    bool frozen1 = checkFreeze(fm1, e1, nowMs);
    bool frozen2 = checkFreeze(fm2, e2, nowMs);
    imu1_frozen_now = frozen1;
    imu2_frozen_now = frozen2;
    if (frozen1 && !fault_imu1) Serial.printf("[FAULT] IMU1 FROZEN (read=%d)\n", readStatus1);
    if (frozen2 && !fault_imu2) Serial.printf("[FAULT] IMU2 FROZEN (read=%d)\n", readStatus2);
    if (frozen1) fault_imu1 = true;
    if (frozen2) fault_imu2 = true;

    // IMU1 sensor frame 기준 각속도 (IMU2 축 정렬 + 개별 bias 보정)
    float g1[3] = {
      e1.gyro[0]*GYRO_SCALE - gyro_bias1[0],
      e1.gyro[1]*GYRO_SCALE - gyro_bias1[1],
      e1.gyro[2]*GYRO_SCALE - gyro_bias1[2] };
    float g2[3] = {
      IMU2_SIGN_X*e2.gyro[0]*GYRO_SCALE - gyro_bias2[0],
      IMU2_SIGN_Y*e2.gyro[1]*GYRO_SCALE - gyro_bias2[1],
      IMU2_SIGN_Z*e2.gyro[2]*GYRO_SCALE - gyro_bias2[2] };
    float a1[3] = { (float)e1.accel[0], (float)e1.accel[1], (float)e1.accel[2] };
    float a2[3] = { IMU2_SIGN_X*e2.accel[0], IMU2_SIGN_Y*e2.accel[1], IMU2_SIGN_Z*e2.accel[2] };

    // 현재 샘플 불일치는 재시동 거부 판단에도 사용한다.
    bool disagreeSample = false;
    if (!frozen1 && !frozen2) {
      for (int k = 0; k < 3; k++) {
        if (fabsf(g1[k] - g2[k]) > GYRO_DISAGREE_DPS) disagreeSample = true;
      }
    }
    imu_disagree_now = disagreeSample;

    // latch된 센서는 비행 중 값이 다시 움직여도 자동 재투입하지 않는다.
    bool h1 = !fault_imu1, h2 = !fault_imu2;

    // ---------- IMU 융합 (진짜 redundancy) ----------
    // 둘 다 정상: 불일치 검사 후 평균 / 하나만 정상: 그것만 / 중재 불가: fault
    if (h1 && h2) {
      if (disagreeSample) {
        if (disagreeSince == 0) disagreeSince = nowMs;
        else if (nowMs - disagreeSince >= IMU_DISAGREE_MS) {
          if (!fault_disagree) Serial.println("[FAULT] DUAL IMU DISAGREEMENT");
          fault_disagree = true;
        }
      } else {
        disagreeSince = 0;
      }
    } else {
      disagreeSince = 0;
    }

    float fg[3], fa[3];
    if (h1 && h2 && !fault_disagree) {
      for (int k = 0; k < 3; k++) { fg[k] = (g1[k]+g2[k])*0.5f; fa[k] = (a1[k]+a2[k])*0.5f; }
      active_imus = 2;
    } else if (h1 && !h2) {
      for (int k = 0; k < 3; k++) { fg[k] = g1[k]; fa[k] = a1[k]; }
      active_imus = 1;
    } else if (h2 && !h1) {
      for (int k = 0; k < 3; k++) { fg[k] = g2[k]; fa[k] = a2[k]; }
      active_imus = 1;
    } else {
      // 둘 다 freeze 또는 서로 안 맞음 -> 신뢰 불가
      active_imus = 0;
      safety_lock = true;
      mixer_scaled = false;
      errorSumRoll = errorSumPitch = errorSumYaw = 0.0f;
      targetRateRoll = targetRatePitch = targetRateYaw = 0.0f;
      lpfD_Roll.reset(); lpfD_Pitch.reset(); lpfD_Yaw.reset();
      wasLocked = true;
      stopMotors();
      continue;
    }

    // 실기 검증된 sensor -> body frame 변환(archive/legacy_flight/dual_imu_pid_pwm과 동일):
    // body roll X=-sensor Y, pitch Y= sensor X, yaw Z=-sensor Z.
    const float bodyGx = -fg[1];
    const float bodyGy =  fg[0];
    const float bodyGz = -fg[2];
    const float bodyAx =  fa[1] * ACCEL_SCALE;
    const float bodyAy = -fa[0] * ACCEL_SCALE;
    const float bodyAz =  fa[2] * ACCEL_SCALE;
    gyroX = bodyGx; gyroY = bodyGy; gyroZ = bodyGz;
    accX = bodyAx; accY = bodyAy; accZ = bodyAz;

    // ---------- 자세 추정 (상보필터) ----------
    angleX += bodyGx * realDt;
    angleY += bodyGy * realDt;
    if (fabsf(bodyGz) > YAW_DEADZONE) angleZ += bodyGz * realDt;

    float alpha = compute_alpha(bodyAx, bodyAy, bodyAz);
    float accAngleX = atan2f(bodyAy, sqrtf(bodyAx*bodyAx + bodyAz*bodyAz)) * 180.0f / PI;
    float accAngleY = atan2f(-bodyAx, sqrtf(bodyAy*bodyAy + bodyAz*bodyAz)) * 180.0f / PI;
    angleX = alpha * angleX + (1.0f - alpha) * accAngleX;
    angleY = alpha * angleY + (1.0f - alpha) * accAngleY;

    // ---------- 안전 검사 ----------
    if (fabsf(angleX) > SAFETY_ANGLE || fabsf(angleY) > SAFETY_ANGLE) {
      if (!fault_attitude) Serial.printf("[FAULT] OVER-TILT R:%.1f P:%.1f\n", angleX, angleY);
      fault_attitude = true;
      safety_lock = true;
    }
    if (!safety_lock && (nowMs - lastRcMs > RC_TIMEOUT_MS)) {
      fault_rc = true; safety_lock = true;
      Serial.println("[FAULT] RC TIMEOUT");
    }
    if (safety_lock) {
      mixer_scaled = false;
      errorSumRoll = errorSumPitch = errorSumYaw = 0.0f;
      targetRateRoll = targetRatePitch = targetRateYaw = 0.0f;
      prevGyroX = bodyGx; prevGyroY = bodyGy; prevGyroZ = bodyGz;
      lpfD_Roll.reset(); lpfD_Pitch.reset(); lpfD_Yaw.reset();
      outerCnt = 0;
      wasLocked = true;
      stopMotors();
      continue;
    }

    // 잠금 해제 첫 tick의 D kick과 이전 목표 rate 잔류를 제거한다.
    if (wasLocked) {
      prevGyroX = bodyGx; prevGyroY = bodyGy; prevGyroZ = bodyGz;
      lpfD_Roll.reset(); lpfD_Pitch.reset(); lpfD_Yaw.reset();
      targetRateRoll = targetRatePitch = targetRateYaw = 0.0f;
      outerCnt = 0;
      wasLocked = false;
    }

    // ---------- Outer loop (250Hz): 각도 -> 목표 각속도 ----------
    const bool yawOn = yaw_enabled;
    if (outerCnt == 0) {
      targetRateRoll = constrain((targetAngleX - angleX) * Kp_Angle_Roll,
                                 -MAX_TARGET_RATE_RP, MAX_TARGET_RATE_RP);
      targetRatePitch = constrain((targetAngleY - angleY) * Kp_Angle_Pitch,
                                  -MAX_TARGET_RATE_RP, MAX_TARGET_RATE_RP);
      targetRateYaw = yawOn
                    ? constrain((targetAngleZ - angleZ) * Kp_Angle_Yaw,
                                -MAX_TARGET_RATE_YAW, MAX_TARGET_RATE_YAW)
                    : 0.0f;
    }
    outerCnt++;
    if (outerCnt >= OUTER_DIV) outerCnt = 0;
    if (!yawOn) targetRateYaw = 0.0f;

    // ---------- Inner loop (1kHz): 각속도 PID ----------
    float eRoll  = targetRateRoll  - bodyGx;
    float ePitch = targetRatePitch - bodyGy;
    float eYaw   = targetRateYaw   - bodyGz;

    // D는 gyro 미분에만 건다 (setpoint/outer loop 오염 제거)
    float dRoll  = lpfD_Roll.update((bodyGx - prevGyroX) / realDt);
    float dPitch = lpfD_Pitch.update((bodyGy - prevGyroY) / realDt);
    float dYaw   = lpfD_Yaw.update((bodyGz - prevGyroZ) / realDt);
    prevGyroX = bodyGx; prevGyroY = bodyGy; prevGyroZ = bodyGz;

    float pidRoll  = Kp_Rate_Roll *eRoll  + Ki_Rate_Roll *errorSumRoll  - Kd_Rate_Roll *dRoll;
    float pidPitch = Kp_Rate_Pitch*ePitch + Ki_Rate_Pitch*errorSumPitch - Kd_Rate_Pitch*dPitch;
    float pidYaw   = yawOn
                   ? (Kp_Rate_Yaw*eYaw + Ki_Rate_Yaw*errorSumYaw - Kd_Rate_Yaw*dYaw)
                   : 0.0f;

    // ---------- 모터 desaturation + 포화 기반 anti-windup ----------
    const int throttle = base_throttle;
    MotorMix mix = mixAndDesaturate(pidRoll, pidPitch, pidYaw,
                                    throttle, min_throttle, max_throttle);
    mixer_scaled = mix.scaled;

    // scale은 자세 명령이 실제로 잘린 경우다. collective 이동만 일어난 경우에는
    // 자세 authority가 보존되므로 적분을 계속한다.
    if (throttle > 1100 && !mix.scaled) {
      errorSumRoll  = constrain(errorSumRoll  + eRoll  * realDt, -200.0f, 200.0f);
      errorSumPitch = constrain(errorSumPitch + ePitch * realDt, -200.0f, 200.0f);
      if (yawOn) errorSumYaw = constrain(errorSumYaw + eYaw * realDt, -200.0f, 200.0f);
      else errorSumYaw = 0.0f;
    } else if (throttle <= 1100) {
      errorSumRoll = errorSumPitch = errorSumYaw = 0.0f;
    } else if (!yawOn) {
      errorSumYaw = 0.0f;
    }

    // 모터 PWM의 단일 writer는 PID task로 유지한다.
    if (safety_lock) { stopMotors(); wasLocked = true; continue; }
    writeMotor(pinM1, mix.motor[0]);
    writeMotor(pinM2, mix.motor[1]);
    writeMotor(pinM3, mix.motor[2]);
    writeMotor(pinM4, mix.motor[3]);
  }
}

// ==========================================================
// 9. UDP 태스크 (Core 0) — char* 파싱 (String heap 회피)
// ==========================================================
static char *trimCommand(char *s) {
  while (*s == ' ' || *s == '\t' || *s == '\r' || *s == '\n') s++;
  char *end = s + strlen(s);
  while (end > s && (end[-1] == ' ' || end[-1] == '\t' || end[-1] == '\r' || end[-1] == '\n')) end--;
  *end = '\0';
  return s;
}

static bool parseFloatStrict(const char *text, float &out) {
  char *end;
  float value = strtof(text, &end);
  if (end == text || !isfinite(value)) return false;
  while (*end == ' ' || *end == '\t') end++;
  if (*end != '\0') return false;
  out = value;
  return true;
}

static bool parseIntStrict(const char *text, long &out) {
  char *end;
  long value = strtol(text, &end, 10);
  if (end == text) return false;
  while (*end == ' ' || *end == '\t') end++;
  if (*end != '\0') return false;
  out = value;
  return true;
}

static bool setRcTargets(float x, float y, float z, bool hasYaw) {
  if (!isfinite(x) || !isfinite(y) || (hasYaw && !isfinite(z))) return false;
  targetAngleX = constrain(x, -MAX_TARGET_ANGLE_RP, MAX_TARGET_ANGLE_RP);
  targetAngleY = constrain(y, -MAX_TARGET_ANGLE_RP, MAX_TARGET_ANGLE_RP);
  if (hasYaw) targetAngleZ = z;
  lastRcMs = millis();
  return true;
}

// 표준 형식: rc <seq> <roll> <pitch> <yaw>
// 벤치 수동 테스트용으로 rc <roll> <pitch> [yaw]도 허용한다.
static void handleRcCommand(char *buf) {
  char *save = nullptr;
  char *token = strtok_r(buf, " \t", &save); // "rc"
  (void)token;
  char *arg[4] = {nullptr, nullptr, nullptr, nullptr};
  int count = 0;
  while (count < 4 && (arg[count] = strtok_r(nullptr, " \t", &save)) != nullptr) count++;
  if (strtok_r(nullptr, " \t", &save) != nullptr) return; // 여분 필드 거부

  if (count == 4) {
    char *seqEnd;
    unsigned long seqLong = strtoul(arg[0], &seqEnd, 10);
    if (seqEnd == arg[0] || *seqEnd != '\0') return;

    float x, y, z;
    if (!parseFloatStrict(arg[1], x) || !parseFloatStrict(arg[2], y) ||
        !parseFloatStrict(arg[3], z)) return;

    uint32_t seq = (uint32_t)seqLong;
    rcTotalPkts = rcTotalPkts + 1;
    if (lastRcSeq != 0) {
      int32_t advance = (int32_t)(seq - lastRcSeq); // uint32 wrap도 정상 처리
      if (advance <= 0) {
        rcDroppedPkts = rcDroppedPkts + 1;
        return; // 지연 도착/중복 패킷 폐기
      }
      if (advance > 1) rcDroppedPkts += (uint32_t)(advance - 1);
    }
    lastRcSeq = seq;
    setRcTargets(x, y, z, true);
    return;
  }

  if (count == 2 || count == 3) {
    float x, y, z = 0.0f;
    if (!parseFloatStrict(arg[0], x) || !parseFloatStrict(arg[1], y)) return;
    if (count == 3 && !parseFloatStrict(arg[2], z)) return;
    setRcTargets(x, y, z, count == 3);
  }
}

static void handleGainCommand(const char *buf) {
  if (strlen(buf) < 3) return;
  float value;
  if (!parseFloatStrict(buf + 2, value) || value < 0.0f || value > 100.0f) return;

  // Cascade 전용 공통 명령
  if      (strncmp(buf, "rp", 2) == 0) { Kp_Rate_Roll = value; Kp_Rate_Pitch = value; }
  else if (strncmp(buf, "ri", 2) == 0) { Ki_Rate_Roll = value; Ki_Rate_Pitch = value; }
  else if (strncmp(buf, "rd", 2) == 0) { Kd_Rate_Roll = value; Kd_Rate_Pitch = value; }
  else if (strncmp(buf, "ap", 2) == 0) { Kp_Angle_Roll = value; Kp_Angle_Pitch = value; }
  else if (strncmp(buf, "ar", 2) == 0) Kp_Angle_Roll = value;
  else if (strncmp(buf, "at", 2) == 0) Kp_Angle_Pitch = value;
  else if (strncmp(buf, "ay", 2) == 0) Kp_Angle_Yaw = value;
  else if (strncmp(buf, "yp", 2) == 0) Kp_Rate_Yaw = value;
  else if (strncmp(buf, "yi", 2) == 0) Ki_Rate_Yaw = value;
  else if (strncmp(buf, "yd", 2) == 0) Kd_Rate_Yaw = value;

  // scripts/tune_pid.py 명령 호환: P/I/D는 inner rate PID에 대응
  else if (strncmp(buf, "pa", 2) == 0) { Kp_Rate_Roll = value; Kp_Rate_Pitch = value; }
  else if (strncmp(buf, "ia", 2) == 0) { Ki_Rate_Roll = value; Ki_Rate_Pitch = value; }
  else if (strncmp(buf, "da", 2) == 0) { Kd_Rate_Roll = value; Kd_Rate_Pitch = value; }
  else if (strncmp(buf, "pr", 2) == 0) Kp_Rate_Roll = value;
  else if (strncmp(buf, "ir", 2) == 0) Ki_Rate_Roll = value;
  else if (strncmp(buf, "dr", 2) == 0) Kd_Rate_Roll = value;
  else if (strncmp(buf, "pp", 2) == 0) Kp_Rate_Pitch = value;
  else if (strncmp(buf, "ip", 2) == 0) Ki_Rate_Pitch = value;
  else if (strncmp(buf, "dp", 2) == 0) Kd_Rate_Pitch = value;
  else if (strncmp(buf, "py", 2) == 0) Kp_Rate_Yaw = value;
  else if (strncmp(buf, "iy", 2) == 0) Ki_Rate_Yaw = value;
  else if (strncmp(buf, "dy", 2) == 0) Kd_Rate_Yaw = value;
}

// 첫 14개 필드는 기존 PC 스크립트와 호환된다. 뒤 필드는 cascade 진단 확장:
// fault_imu1,fault_imu2,fault_disagree,active_imus,scaled,fault_attitude,calibration_ok
static void sendTelemetry() {
  if (!connectionEstablished) return;
  bool criticalFault = (active_imus == 0) || fault_attitude || !calibration_ok;
  udp.beginPacket(laptopIP, laptopPort);
  udp.printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%d,%d,%d,%lu,%lu,%d,%d,%d,%d,%d,%d,%d",
             angleX, angleY, angleZ,
             gyroX, gyroY, gyroZ,
             accX, accY, accZ,
             base_throttle,
             (int)fault_rc, (int)criticalFault,
             (unsigned long)rcTotalPkts, (unsigned long)rcDroppedPkts,
             (int)fault_imu1, (int)fault_imu2, (int)fault_disagree,
             active_imus, (int)mixer_scaled, (int)fault_attitude,
             (int)calibration_ok);
  udp.endPacket();
}

void udp_task(void *pv) {
  const int CTRL_MARGIN = 150;
  uint32_t lastSend = 0;
  while (true) {
    int packetSize = udp.parsePacket();
    if (packetSize) {
      laptopIP = udp.remoteIP();
      laptopPort = udp.remotePort();
      connectionEstablished = true;

      int len = udp.read(packetBuffer, sizeof(packetBuffer) - 1); // off-by-one 방지
      if (len > 0) {
        packetBuffer[len] = '\0';
        char *buf = trimCommand(packetBuffer);

        if (strncmp(buf, "rc", 2) == 0 && (buf[2] == ' ' || buf[2] == '\t')) {
          handleRcCommand(buf);
        }
        else if (strcmp(buf, "start") == 0) {
          bool overTilt = fabsf(angleX) > SAFETY_ANGLE || fabsf(angleY) > SAFETY_ANGLE;
          bool noUsableImu = imu1_frozen_now && imu2_frozen_now;
          if (!calibration_ok || overTilt || noUsableImu || imu_disagree_now) {
            safety_lock = true;
            Serial.printf(">>> START REFUSED calib=%d tilt=%d imu=%d disagree=%d\n",
                          (int)calibration_ok, (int)overTilt,
                          (int)noUsableImu, (int)imu_disagree_now);
          } else {
            safety_lock = true; // 상태를 모두 초기화한 뒤 마지막에 해제
            fault_rc = false;
            fault_imu1 = imu1_frozen_now;
            fault_imu2 = imu2_frozen_now;
            fault_disagree = false;
            fault_attitude = false;
            lastRcSeq = 0;
            lastRcMs = millis();
            base_throttle = 1100; min_throttle = 1050; max_throttle = 1250;
            targetAngleX = 0.0f;
            targetAngleY = 0.0f;
            targetAngleZ = 0.0f;
            angleZ = 0.0f;
            safety_lock = false;
            Serial.println(">>> START");
          }
        }
        else if (strcmp(buf, "stop") == 0) {
          safety_lock = true;
          base_throttle = 1000;
          Serial.println(">>> STOP");
        }
        else if (strncmp(buf, "th", 2) == 0) {
          long parsed;
          if (parseIntStrict(buf + 2, parsed)) {
            int nb = constrain((int)parsed, 1000, 1900);
            base_throttle = nb;
            min_throttle = max(1050, nb - CTRL_MARGIN);
            max_throttle = min(1900, nb + CTRL_MARGIN);
          }
        }
        else if (strncmp(buf, "yaw", 3) == 0) {
          long enabled;
          if (parseIntStrict(buf + 3, enabled) && (enabled == 0 || enabled == 1)) {
            targetAngleZ = angleZ; // 활성화 순간 setpoint jump 방지
            yaw_enabled = (enabled == 1); // setpoint을 먼저 맞춘 뒤 활성화
            Serial.printf(">>> Yaw %s\n", yaw_enabled ? "ON" : "OFF");
          }
        }
        else {
          handleGainCommand(buf);
        }
      }
    }

    uint32_t now = millis();
    if (now - lastSend >= 50) {
      lastSend = now;
      sendTelemetry();
    }
    vTaskDelay(1);
  }
}

// ==========================================================
// 10. setup / loop
// ==========================================================
void setup() {
  Serial.begin(115200);

  pinMode(SPI_CS1, OUTPUT); pinMode(SPI_CS2, OUTPUT);
  digitalWrite(SPI_CS1, HIGH); digitalWrite(SPI_CS2, HIGH);   // float CS 로 인한 버스 경합 방지
  delay(100);

  WiFi.softAP(WIFI_SSID, WIFI_PASS, WIFI_CHANNEL);
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  udp.begin(UDP_PORT);

  SPI.begin(12, 13, 11, -1);   // SCK, MISO, MOSI; CS는 각 IMU 객체가 관리

  bool esc_ok = ledcAttach(pinM1, ESC_FREQ, ESC_RES)
             && ledcAttach(pinM2, ESC_FREQ, ESC_RES)
             && ledcAttach(pinM3, ESC_FREQ, ESC_RES)
             && ledcAttach(pinM4, ESC_FREQ, ESC_RES);
  if (!esc_ok) while (1) { Serial.println("[FAULT] ESC attach FAIL"); delay(1000); }
  stopMotors();

  if (IMU1.begin() < 0) while (1) { Serial.println("[FAULT] IMU1 FAIL"); delay(1000); }
  if (IMU2.begin() < 0) while (1) { Serial.println("[FAULT] IMU2 FAIL"); delay(1000); }
  int sensorStartStatus = 0;
  sensorStartStatus |= IMU1.startAccel(1600, 16);
  sensorStartStatus |= IMU1.startGyro(1600, 2000);
  sensorStartStatus |= IMU2.startAccel(1600, 16);
  sensorStartStatus |= IMU2.startGyro(1600, 2000);
  if (sensorStartStatus != 0) {
    while (1) { Serial.printf("[FAULT] IMU START FAIL (%d)\n", sensorStartStatus); delay(1000); }
  }
  delay(500);

  calibration_ok = calibrate_bias();

  // pid_task 전용 태스크 워치독. idle task는 감시하지 않고, timeout 시
  // panic 재부팅으로 마지막 PWM 고정 상태에서 벗어난다.
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = PID_WDT_TIMEOUT_MS,
    .idle_core_mask = 0,
    .trigger_panic = true,
  };
  if (esp_task_wdt_reconfigure(&wdt_config) != ESP_OK &&
      esp_task_wdt_init(&wdt_config) != ESP_OK) {
    while (1) { Serial.println("[FAULT] TASK WDT INIT FAIL"); delay(1000); }
  }

  BaseType_t pidTaskOk = xTaskCreatePinnedToCore(pid_task, "PID", 4096, NULL, 2, NULL, 1);
  BaseType_t udpTaskOk = xTaskCreatePinnedToCore(udp_task, "UDP", 4096, NULL, 1, NULL, 0);
  if (pidTaskOk != pdPASS || udpTaskOk != pdPASS) {
    safety_lock = true;
    while (1) { Serial.println("[FAULT] TASK CREATE FAIL"); delay(1000); }
  }

  Serial.println("DUAL_IMU_CASCADE READY");
}

void loop() {
  // UDP 객체는 udp_task 한 곳에서만 접근해 cross-core race를 피한다.
  delay(1000);
}
