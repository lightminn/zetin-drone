#include <Arduino.h>
#include <SPI.h>
#include <ICM42670P.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// ==========================================================
// 1. 튜닝 파라미터
// ==========================================================
const float LPF_ALPHA_ACC  = 0.10f;
const float LPF_ALPHA_GYRO = 0.50f;
const float ALPHA_COMP     = 0.995f;

volatile float Kp_Roll  = 2.5f,  Ki_Roll  = 0.005f, Kd_Roll  = 1.2f;
volatile float Kp_Pitch = 2.5f,  Ki_Pitch = 0.005f, Kd_Pitch = 1.2f;
volatile float Kp_Yaw   = 3.5f,  Ki_Yaw   = 0.0f,   Kd_Yaw   = 0.0f;

volatile int  base_throttle = 1000;
volatile int  min_throttle  = 1050;
volatile int  max_throttle  = 1300;
volatile bool yaw_enabled   = false;

// ==========================================================
// 2. 시스템 상수
// ==========================================================
const char* WIFI_SSID    = "Drone_Tuning";
const char* WIFI_PASS    = "12345678";
const int   UDP_PORT     = 4210;
const int   WIFI_CHANNEL = 6;    // 1/6/11 중 주변 AP와 겹치지 않는 채널로 설정

const int pinM1   = 4;   // FL (앞/왼쪽),   CW
const int pinM2   = 5;   // RR (뒤/오른쪽),  CW
const int pinM3   = 6;   // FR (앞/오른쪽),  CCW
const int pinM4   = 7;   // RL (뒤/왼쪽),   CCW
const int SPI_CS  = 10;
const int LDO_PIN = 9;

const int ESC_FREQ   = 400;
const int ESC_RES    = 14;
const int ESC_PERIOD = 2500;

const float GYRO_SCALE   = 1.0f / 16.4f;
const float ACCEL_SCALE  = 1.0f / 2048.0f;
const float SAFETY_ANGLE = 45.0f;
const float YAW_DEADZONE = 0.3f;

// --- 고장진단 상수 ---
const uint32_t RC_TIMEOUT_MS     = 500;   // RC 패킷 없으면 500ms 후 긴급정지
const float    IMU_FROZEN_THRESH = 0.001f; // 자이로 값이 이만큼 안 변하면 고장
const uint32_t IMU_FROZEN_MS     = 200;   // 200ms 이상 동결되면 고장

// ==========================================================
// 3. 시스템 변수
// ==========================================================
WiFiUDP   udp;
char      packetBuffer[256];
IPAddress laptopIP;
int       laptopPort            = 0;
bool      connectionEstablished = false;

ICM42670 IMU(SPI, SPI_CS);

volatile bool  safety_lock  = true;
volatile float targetAngleX = 0.0f, targetAngleY = 0.0f, targetAngleZ = 0.0f;

float angleX = 0.0f, angleY = 0.0f, angleZ = 0.0f;
float errorSumRoll = 0.0f, errorSumPitch = 0.0f, errorSumYaw = 0.0f;

volatile float raw_gx = 0.0f, raw_gy = 0.0f, raw_gz = 0.0f;
volatile float raw_ax = 0.0f, raw_ay = 0.0f, raw_az = 0.0f;
float lpf_ax = 0.0f, lpf_ay = 0.0f, lpf_az = 0.0f;
float lpf_gx = 0.0f, lpf_gy = 0.0f, lpf_gz = 0.0f;

// --- 고장진단 변수 ---
volatile uint32_t lastRcTimeMs   = 0;     // 마지막 RC 패킷 수신 시각
volatile bool     fault_rc       = false; // RC 타임아웃 고장 플래그
volatile bool     fault_imu      = false; // IMU 동결 고장 플래그
float             prev_gx        = 0.0f;  // IMU 동결 감지용
uint32_t          imuFrozenSince = 0;     // IMU 동결 시작 시각

// --- 통신 품질 변수 ---
volatile uint32_t lastRcSeq      = 0;     // 마지막 수신 RC 시퀀스 번호 (낡은 패킷 폐기용)
volatile uint32_t rcTotalPkts    = 0;     // 수신된 RC 패킷 총 수
volatile uint32_t rcDroppedPkts  = 0;     // 시퀀스 역전으로 폐기된 패킷 수

// ==========================================================
// 4. 모터 제어
// ==========================================================
void writeMotor(int pin, int us) {
  us = constrain(us, 1000, 2000);
  uint32_t duty = ((uint32_t)us * 16383UL) / ESC_PERIOD;
  ledcWrite(pin, duty);
}

void stopMotors() {
  writeMotor(pinM1, 1000);
  writeMotor(pinM2, 1000);
  writeMotor(pinM3, 1000);
  writeMotor(pinM4, 1000);
}

// ==========================================================
// 5. 고장진단
// ==========================================================

// RC 타임아웃 감지: 비행 중 RC 패킷이 일정 시간 이상 안 오면 긴급정지
bool check_rc_timeout() {
  if (safety_lock) return false; // 이미 잠긴 상태면 패스
  uint32_t elapsed = millis() - lastRcTimeMs;
  if (elapsed > RC_TIMEOUT_MS) {
    fault_rc    = true;
    safety_lock = true;
    Serial.printf("[FAULT] RC TIMEOUT (%ums) - 긴급정지\n", elapsed);
    return true;
  }
  return false;
}

// IMU 동결 감지: 자이로 값이 너무 오래 변하지 않으면 센서 고장으로 판단
bool check_imu_frozen() {
  if (fabsf(raw_gx - prev_gx) < IMU_FROZEN_THRESH) {
    if (imuFrozenSince == 0) imuFrozenSince = millis();
    if (millis() - imuFrozenSince > IMU_FROZEN_MS) {
      fault_imu   = true;
      safety_lock = true;
      Serial.printf("[FAULT] IMU FROZEN (%ums) - 긴급정지\n", millis() - imuFrozenSince);
      return true;
    }
  } else {
    imuFrozenSince = 0;
  }
  prev_gx = raw_gx;
  return false;
}

// 과도 기울기 감지
bool check_attitude() {
  if (fabsf(angleX) > SAFETY_ANGLE || fabsf(angleY) > SAFETY_ANGLE) {
    safety_lock = true;
    Serial.printf("[FAULT] OVER-TILT (Roll:%.1f Pitch:%.1f) - 긴급정지\n", angleX, angleY);
    return true;
  }
  return false;
}

// ==========================================================
// 6. PID 태스크 (Core 1, 1kHz)
// ==========================================================
void pid_task(void *pvParameters) {
  IMU.startAccel(1600, 16);
  IMU.startGyro(1600, 2000);

  const unsigned long LOOP_INTERVAL = 1000;
  unsigned long nextLoopTime = micros();
  unsigned long lastTime      = micros();

  inv_imu_sensor_event_t imu_event;

  IMU.getDataFromRegisters(imu_event);
  lpf_ax =  imu_event.accel[0] * ACCEL_SCALE;
  lpf_ay = -imu_event.accel[1] * ACCEL_SCALE;
  lpf_az =  imu_event.accel[2] * ACCEL_SCALE;
  lpf_gx =  imu_event.gyro[0]  * GYRO_SCALE;
  lpf_gy = -imu_event.gyro[1]  * GYRO_SCALE;
  lpf_gz =  imu_event.gyro[2]  * GYRO_SCALE;

  while (true) {
    unsigned long now = micros();
    if (now < nextLoopTime) { vTaskDelay(0); continue; }
    nextLoopTime = now + LOOP_INTERVAL;

    // --- 센서 읽기 ---
    IMU.getDataFromRegisters(imu_event);
    raw_ax =  imu_event.accel[0] * ACCEL_SCALE;
    raw_ay = -imu_event.accel[1] * ACCEL_SCALE;
    raw_az =  imu_event.accel[2] * ACCEL_SCALE;
    raw_gx =  imu_event.gyro[0]  * GYRO_SCALE;
    raw_gy = -imu_event.gyro[1]  * GYRO_SCALE;
    raw_gz =  imu_event.gyro[2]  * GYRO_SCALE;

    lpf_ax = LPF_ALPHA_ACC  * raw_ax + (1.0f - LPF_ALPHA_ACC)  * lpf_ax;
    lpf_ay = LPF_ALPHA_ACC  * raw_ay + (1.0f - LPF_ALPHA_ACC)  * lpf_ay;
    lpf_az = LPF_ALPHA_ACC  * raw_az + (1.0f - LPF_ALPHA_ACC)  * lpf_az;
    lpf_gx = LPF_ALPHA_GYRO * raw_gx + (1.0f - LPF_ALPHA_GYRO) * lpf_gx;
    lpf_gy = LPF_ALPHA_GYRO * raw_gy + (1.0f - LPF_ALPHA_GYRO) * lpf_gy;
    lpf_gz = LPF_ALPHA_GYRO * raw_gz + (1.0f - LPF_ALPHA_GYRO) * lpf_gz;

    float dt = (now - lastTime) / 1000000.0f;
    if (dt > 0.002f) dt = 0.001f;
    lastTime = now;

    // --- 자세 추정 ---
    float accAngleX = atan2f(lpf_ay, sqrtf(lpf_ax*lpf_ax + lpf_az*lpf_az)) * 180.0f / PI;
    float accAngleY = atan2f(-lpf_ax, sqrtf(lpf_ay*lpf_ay + lpf_az*lpf_az)) * 180.0f / PI;
    angleX = ALPHA_COMP * (angleX + lpf_gx * dt) + (1.0f - ALPHA_COMP) * accAngleX;
    angleY = ALPHA_COMP * (angleY + lpf_gy * dt) + (1.0f - ALPHA_COMP) * accAngleY;
    if (fabsf(lpf_gz) > YAW_DEADZONE) angleZ += lpf_gz * dt;

    // --- 고장진단 (순서 중요: 감지 즉시 stopMotors) ---
    if (check_imu_frozen() || check_rc_timeout() || check_attitude() || safety_lock) {
      stopMotors();
      vTaskDelay(0);
      continue;
    }

    // --- PID 계산 ---
    float errorRoll  = targetAngleX - angleX;
    float errorPitch = targetAngleY - angleY;
    float errorYaw   = targetAngleZ - angleZ;

    if (base_throttle < 1100) {
      errorSumRoll = errorSumPitch = errorSumYaw = 0.0f;
    } else {
      if (fabsf(errorRoll)  < 25.0f) errorSumRoll  = constrain(errorSumRoll  + errorRoll  * dt, -15.0f, 15.0f);
      if (fabsf(errorPitch) < 25.0f) errorSumPitch = constrain(errorSumPitch + errorPitch * dt, -15.0f, 15.0f);
      if (fabsf(errorYaw)   < 25.0f) errorSumYaw   = constrain(errorSumYaw   + errorYaw   * dt, -15.0f, 15.0f);
    }

    float pid_roll  = errorRoll  * Kp_Roll  + errorSumRoll  * Ki_Roll  - lpf_gx * Kd_Roll;
    float pid_pitch = errorPitch * Kp_Pitch + errorSumPitch * Ki_Pitch - lpf_gy * Kd_Pitch;
    float pid_yaw   = yaw_enabled
                    ? (errorYaw * Kp_Yaw + errorSumYaw * Ki_Yaw - lpf_gz * Kd_Yaw)
                    : 0.0f;

    // --- 모터 믹싱 (Quad-X) ---
    writeMotor(pinM1, constrain((int)(base_throttle - pid_pitch + pid_roll - pid_yaw), min_throttle, max_throttle));
    writeMotor(pinM2, constrain((int)(base_throttle + pid_pitch - pid_roll - pid_yaw), min_throttle, max_throttle));
    writeMotor(pinM3, constrain((int)(base_throttle - pid_pitch - pid_roll + pid_yaw), min_throttle, max_throttle));
    writeMotor(pinM4, constrain((int)(base_throttle + pid_pitch + pid_roll + pid_yaw), min_throttle, max_throttle));
  }
}

// ==========================================================
// 7. UDP 태스크 (Core 0)
// ==========================================================
void udp_task(void *pvParameters) {
  const int CTRL_MARGIN = 150;

  while (true) {
    int packetSize = udp.parsePacket();
    if (!packetSize) { vTaskDelay(2); continue; }

    laptopIP   = udp.remoteIP();
    laptopPort = udp.remotePort();
    connectionEstablished = true;

    int len = udp.read(packetBuffer, sizeof(packetBuffer) - 1);
    if (len <= 0) { vTaskDelay(2); continue; }
    packetBuffer[len] = '\0';

    String cmd = String(packetBuffer);
    cmd.trim();

    // RC 명령 수신: "rc <seq> <roll> <pitch> <yaw>"
    if (cmd.startsWith("rc ")) {
      // 필드: rc / seq / roll / pitch / yaw
      int s1 = cmd.indexOf(' ');           // "rc" 뒤
      int s2 = cmd.indexOf(' ', s1 + 1);  // seq 뒤
      int s3 = cmd.indexOf(' ', s2 + 1);  // roll 뒤
      int s4 = cmd.indexOf(' ', s3 + 1);  // pitch 뒤

      if (s1 > 0 && s2 > 0 && s3 > 0) {
        uint32_t seq = (uint32_t)cmd.substring(s1 + 1, s2).toInt();
        rcTotalPkts++;

        // 시퀀스 역전 패킷(지연 도착한 낡은 패킷) 폐기
        if (seq <= lastRcSeq && lastRcSeq != 0) {
          rcDroppedPkts++;
          vTaskDelay(1);
          continue;
        }
        lastRcSeq    = seq;
        lastRcTimeMs = millis();

        targetAngleX = cmd.substring(s2 + 1, s3).toFloat();
        targetAngleY = cmd.substring(s3 + 1, s4 > 0 ? s4 : cmd.length()).toFloat();
        if (s4 > 0) targetAngleZ = cmd.substring(s4 + 1).toFloat();
      }
    }
    else if (cmd == "start") {
      fault_rc = fault_imu = false;
      lastRcSeq     = 0;  // 시퀀스 번호 리셋 (재시동 시 낡은 seq 비교 방지)
      lastRcTimeMs  = millis();
      safety_lock   = false;
      base_throttle = 1100; min_throttle = 1050; max_throttle = 1250;
      targetAngleX  = targetAngleY = targetAngleZ = 0.0f;
      angleZ        = errorSumRoll = errorSumPitch = errorSumYaw = 0.0f;
      Serial.println(">>> START");
    }
    else if (cmd == "stop") {
      safety_lock   = true;
      base_throttle = 1000;
      stopMotors();
      Serial.println(">>> STOP");
    }
    else if (cmd.startsWith("yaw ")) {
      yaw_enabled = (cmd.substring(4).toInt() == 1);
      Serial.printf(">>> Yaw: %s\n", yaw_enabled ? "ON" : "OFF");
    }
    else if (cmd.startsWith("th ")) {
      int new_base  = cmd.substring(3).toInt();
      base_throttle = new_base;
      min_throttle  = max(1050, new_base - CTRL_MARGIN);
      max_throttle  = min(1900, new_base + CTRL_MARGIN);
    }
    else {
      float val = cmd.substring(2).toFloat();
      if      (cmd.startsWith("pa")) { Kp_Roll = Kp_Pitch = val; }
      else if (cmd.startsWith("da")) { Kd_Roll = Kd_Pitch = val; }
      else if (cmd.startsWith("ia")) { Ki_Roll = Ki_Pitch = val; }
      else if (cmd.startsWith("pp")) { Kp_Pitch = val; }
      else if (cmd.startsWith("dp")) { Kd_Pitch = val; }
      else if (cmd.startsWith("ip")) { Ki_Pitch = val; }
      else if (cmd.startsWith("pr")) { Kp_Roll  = val; }
      else if (cmd.startsWith("dr")) { Kd_Roll  = val; }
      else if (cmd.startsWith("ir")) { Ki_Roll  = val; }
      else if (cmd.startsWith("py")) { Kp_Yaw   = val; }
      else if (cmd.startsWith("dy")) { Kd_Yaw   = val; }
      else if (cmd.startsWith("iy")) { Ki_Yaw   = val; }
    }

    vTaskDelay(1);
  }
}

// ==========================================================
// 8. setup / loop
// ==========================================================
void setup() {
  pinMode(LDO_PIN, OUTPUT);
  digitalWrite(LDO_PIN, HIGH);
  delay(100);

  Serial.begin(115200);

  WiFi.softAP(WIFI_SSID, WIFI_PASS, WIFI_CHANNEL);
  WiFi.setSleep(false);                          // [핵심] 절전모드 비활성화: 20~200ms 랜덤 지연 제거
  WiFi.setTxPower(WIFI_POWER_19_5dBm);           // TX 출력 최대화
  udp.begin(UDP_PORT);

  SPI.begin(12, 13, 11, SPI_CS);

  bool esc_ok = ledcAttach(pinM1, ESC_FREQ, ESC_RES)
             && ledcAttach(pinM2, ESC_FREQ, ESC_RES)
             && ledcAttach(pinM3, ESC_FREQ, ESC_RES)
             && ledcAttach(pinM4, ESC_FREQ, ESC_RES);
  if (!esc_ok) {
    while (1) { Serial.println("[FAULT] ESC pin attach FAILED"); delay(1000); }
  }
  stopMotors();

  if (IMU.begin() < 0) {
    while (1) { Serial.println("[FAULT] IMU FAILED"); delay(1000); }
  }
  delay(1000);

  xTaskCreatePinnedToCore(pid_task, "PID", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(udp_task, "UDP", 4096, NULL, 0, NULL, 0);

  Serial.println("SYSTEM READY");
}

void loop() {
  static unsigned long lastSendTime = 0;
  if (millis() - lastSendTime < 50) return;
  lastSendTime = millis();

  if (!connectionEstablished) return;

  // 텔레메트리: 자세 + 센서 + 스로틀 + 고장 플래그 + 패킷 드롭률
  udp.beginPacket(laptopIP, laptopPort);
  udp.printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d,%d,%lu,%lu",
             angleX, angleY, angleZ,
             raw_gx, raw_gy, raw_gz,
             raw_ax, raw_ay, raw_az,
             base_throttle,
             (int)fault_rc,
             (int)fault_imu,
             rcTotalPkts,    // 누적 수신 패킷
             rcDroppedPkts); // 누적 폐기 패킷 (통신 품질 지표)
  udp.endPacket();
}
