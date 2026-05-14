#include <Arduino.h>
#include <SPI.h>
#include <ICM42670P.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// ==========================================================
// 1. Tuning parameters (unchanged from single-IMU version)
// ==========================================================
const float LPF_ALPHA_ACC  = 0.10f;
const float LPF_ALPHA_GYRO = 0.50f;

volatile float Kp_Roll  = 2.5f,  Ki_Roll  = 0.005f, Kd_Roll  = 1.2f;
volatile float Kp_Pitch = 2.5f,  Ki_Pitch = 0.005f, Kd_Pitch = 1.2f;
volatile float Kp_Yaw   = 3.5f,  Ki_Yaw   = 0.0f,   Kd_Yaw   = 0.0f;

volatile int  base_throttle = 1000;
volatile int  min_throttle  = 1050;
volatile int  max_throttle  = 1300;
volatile bool yaw_enabled   = false;

// ==========================================================
// 2. System constants
// ==========================================================
const char* WIFI_SSID    = "Drone_Tuning";
const char* WIFI_PASS    = "12345678";
const int   UDP_PORT     = 4210;
const int   WIFI_CHANNEL = 6;

const int pinM1   = 4;   // FL
const int pinM2   = 5;   // RR
const int pinM3   = 6;   // FR
const int pinM4   = 7;   // RL
const int SPI_CS1 = 10;  // IMU1 CS
const int SPI_CS2 = 9;   // IMU2 CS (replaces former LDO_PIN)

const int ESC_FREQ   = 400;
const int ESC_RES    = 14;
const int ESC_PERIOD = 2500;

const float GYRO_SCALE   = 1.0f / 16.4f;
const float ACCEL_SCALE  = 1.0f / 2048.0f;
const float SAFETY_ANGLE = 45.0f;
const float YAW_DEADZONE = 0.3f;

// IMU2 is mounted with x and z axes inverted vs IMU1 (y matches).
// Apply these signs to every IMU2 axis read before averaging or bias use.
const float IMU2_SIGN_X = -1.0f;
const float IMU2_SIGN_Y =  1.0f;
const float IMU2_SIGN_Z = -1.0f;

const uint32_t RC_TIMEOUT_MS     = 500;
const float    IMU_FROZEN_THRESH = 0.001f;
const uint32_t IMU_FROZEN_MS     = 200;

// ==========================================================
// 3. System variables
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

float angleX = 0.0f, angleY = 0.0f, angleZ = 0.0f;
float errorSumRoll = 0.0f, errorSumPitch = 0.0f, errorSumYaw = 0.0f;

volatile float raw_gx = 0.0f, raw_gy = 0.0f, raw_gz = 0.0f;
volatile float raw_ax = 0.0f, raw_ay = 0.0f, raw_az = 0.0f;
float lpf_ax = 0.0f, lpf_ay = 0.0f, lpf_az = 0.0f;
float lpf_gx = 0.0f, lpf_gy = 0.0f, lpf_gz = 0.0f;

volatile uint32_t lastRcTimeMs   = 0;
volatile bool     fault_rc       = false;
volatile bool     fault_imu      = false;   // OR of fault_imu1/2 for telemetry compatibility
float             prev_gx        = 0.0f;
uint32_t          imuFrozenSince = 0;

volatile uint32_t lastRcSeq      = 0;
volatile uint32_t rcTotalPkts    = 0;
volatile uint32_t rcDroppedPkts  = 0;

// ==========================================================
// 4. Motor control
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
// 5. Fault detection (single-IMU version for now; refined in Task 4)
// ==========================================================
bool check_rc_timeout() {
  if (safety_lock) return false;
  uint32_t elapsed = millis() - lastRcTimeMs;
  if (elapsed > RC_TIMEOUT_MS) {
    fault_rc    = true;
    safety_lock = true;
    Serial.printf("[FAULT] RC TIMEOUT (%ums)\n", elapsed);
    return true;
  }
  return false;
}

bool check_imu_frozen() {
  if (fabsf(raw_gx - prev_gx) < IMU_FROZEN_THRESH) {
    if (imuFrozenSince == 0) imuFrozenSince = millis();
    if (millis() - imuFrozenSince > IMU_FROZEN_MS) {
      fault_imu   = true;
      safety_lock = true;
      Serial.printf("[FAULT] IMU FROZEN (%ums)\n", millis() - imuFrozenSince);
      return true;
    }
  } else {
    imuFrozenSince = 0;
  }
  prev_gx = raw_gx;
  return false;
}

bool check_attitude() {
  if (fabsf(angleX) > SAFETY_ANGLE || fabsf(angleY) > SAFETY_ANGLE) {
    safety_lock = true;
    Serial.printf("[FAULT] OVER-TILT (Roll:%.1f Pitch:%.1f)\n", angleX, angleY);
    return true;
  }
  return false;
}

// ==========================================================
// 6. PID task (Core 1, 1kHz) — single IMU read for now
// ==========================================================
const float ALPHA_COMP = 0.995f;  // temporary; replaced in Task 5

void pid_task(void *pvParameters) {
  const unsigned long LOOP_INTERVAL = 1000;
  unsigned long nextLoopTime = micros();
  unsigned long lastTime      = micros();

  inv_imu_sensor_event_t e1, e2;

  IMU1.getDataFromRegisters(e1);
  IMU2.getDataFromRegisters(e2);
  lpf_ax =  ((e1.accel[0] + IMU2_SIGN_X * e2.accel[0]) * 0.5f) * ACCEL_SCALE;
  lpf_ay = -((e1.accel[1] + IMU2_SIGN_Y * e2.accel[1]) * 0.5f) * ACCEL_SCALE;
  lpf_az =  ((e1.accel[2] + IMU2_SIGN_Z * e2.accel[2]) * 0.5f) * ACCEL_SCALE;
  lpf_gx =  ((e1.gyro[0]  + IMU2_SIGN_X * e2.gyro[0])  * 0.5f) * GYRO_SCALE;
  lpf_gy = -((e1.gyro[1]  + IMU2_SIGN_Y * e2.gyro[1])  * 0.5f) * GYRO_SCALE;
  lpf_gz =  ((e1.gyro[2]  + IMU2_SIGN_Z * e2.gyro[2])  * 0.5f) * GYRO_SCALE;

  while (true) {
    unsigned long now = micros();
    if (now < nextLoopTime) { vTaskDelay(0); continue; }
    nextLoopTime = now + LOOP_INTERVAL;

    IMU1.getDataFromRegisters(e1);
    IMU2.getDataFromRegisters(e2);
    raw_ax =  ((e1.accel[0] + IMU2_SIGN_X * e2.accel[0]) * 0.5f) * ACCEL_SCALE;
    raw_ay = -((e1.accel[1] + IMU2_SIGN_Y * e2.accel[1]) * 0.5f) * ACCEL_SCALE;
    raw_az =  ((e1.accel[2] + IMU2_SIGN_Z * e2.accel[2]) * 0.5f) * ACCEL_SCALE;
    raw_gx =  ((e1.gyro[0]  + IMU2_SIGN_X * e2.gyro[0])  * 0.5f) * GYRO_SCALE;
    raw_gy = -((e1.gyro[1]  + IMU2_SIGN_Y * e2.gyro[1])  * 0.5f) * GYRO_SCALE;
    raw_gz =  ((e1.gyro[2]  + IMU2_SIGN_Z * e2.gyro[2])  * 0.5f) * GYRO_SCALE;

    lpf_ax = LPF_ALPHA_ACC  * raw_ax + (1.0f - LPF_ALPHA_ACC)  * lpf_ax;
    lpf_ay = LPF_ALPHA_ACC  * raw_ay + (1.0f - LPF_ALPHA_ACC)  * lpf_ay;
    lpf_az = LPF_ALPHA_ACC  * raw_az + (1.0f - LPF_ALPHA_ACC)  * lpf_az;
    lpf_gx = LPF_ALPHA_GYRO * raw_gx + (1.0f - LPF_ALPHA_GYRO) * lpf_gx;
    lpf_gy = LPF_ALPHA_GYRO * raw_gy + (1.0f - LPF_ALPHA_GYRO) * lpf_gy;
    lpf_gz = LPF_ALPHA_GYRO * raw_gz + (1.0f - LPF_ALPHA_GYRO) * lpf_gz;

    float dt = (now - lastTime) / 1000000.0f;
    if (dt > 0.002f) dt = 0.001f;
    lastTime = now;

    float accAngleX = atan2f(lpf_ay, sqrtf(lpf_ax*lpf_ax + lpf_az*lpf_az)) * 180.0f / PI;
    float accAngleY = atan2f(-lpf_ax, sqrtf(lpf_ay*lpf_ay + lpf_az*lpf_az)) * 180.0f / PI;
    angleX = ALPHA_COMP * (angleX + lpf_gx * dt) + (1.0f - ALPHA_COMP) * accAngleX;
    angleY = ALPHA_COMP * (angleY + lpf_gy * dt) + (1.0f - ALPHA_COMP) * accAngleY;
    if (fabsf(lpf_gz) > YAW_DEADZONE) angleZ += lpf_gz * dt;

    if (check_imu_frozen() || check_rc_timeout() || check_attitude() || safety_lock) {
      stopMotors();
      vTaskDelay(0);
      continue;
    }

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

    writeMotor(pinM1, constrain((int)(base_throttle - pid_pitch + pid_roll - pid_yaw), min_throttle, max_throttle));
    writeMotor(pinM2, constrain((int)(base_throttle + pid_pitch - pid_roll - pid_yaw), min_throttle, max_throttle));
    writeMotor(pinM3, constrain((int)(base_throttle - pid_pitch - pid_roll + pid_yaw), min_throttle, max_throttle));
    writeMotor(pinM4, constrain((int)(base_throttle + pid_pitch + pid_roll + pid_yaw), min_throttle, max_throttle));
  }
}

// ==========================================================
// 7. UDP task (Core 0) — identical to original
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

    if (cmd.startsWith("rc ")) {
      int s1 = cmd.indexOf(' ');
      int s2 = cmd.indexOf(' ', s1 + 1);
      int s3 = cmd.indexOf(' ', s2 + 1);
      int s4 = cmd.indexOf(' ', s3 + 1);

      if (s1 > 0 && s2 > 0 && s3 > 0) {
        uint32_t seq = (uint32_t)cmd.substring(s1 + 1, s2).toInt();
        rcTotalPkts++;

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
      lastRcSeq     = 0;
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
  Serial.begin(115200);

  // Both CS HIGH before SPI init (prevents bus contention from floating CS)
  pinMode(SPI_CS1, OUTPUT);
  pinMode(SPI_CS2, OUTPUT);
  digitalWrite(SPI_CS1, HIGH);
  digitalWrite(SPI_CS2, HIGH);
  delay(100);

  WiFi.softAP(WIFI_SSID, WIFI_PASS, WIFI_CHANNEL);
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  udp.begin(UDP_PORT);

  SPI.begin(12, 13, 11, SPI_CS1);

  bool esc_ok = ledcAttach(pinM1, ESC_FREQ, ESC_RES)
             && ledcAttach(pinM2, ESC_FREQ, ESC_RES)
             && ledcAttach(pinM3, ESC_FREQ, ESC_RES)
             && ledcAttach(pinM4, ESC_FREQ, ESC_RES);
  if (!esc_ok) {
    while (1) { Serial.println("[FAULT] ESC pin attach FAILED"); delay(1000); }
  }
  stopMotors();

  if (IMU1.begin() < 0) {
    while (1) { Serial.println("[FAULT] IMU1 INIT FAILED"); delay(1000); }
  }
  if (IMU2.begin() < 0) {
    while (1) { Serial.println("[FAULT] IMU2 INIT FAILED"); delay(1000); }
  }
  IMU1.startAccel(1600, 16);
  IMU1.startGyro(1600, 2000);
  IMU2.startAccel(1600, 16);
  IMU2.startGyro(1600, 2000);
  delay(500);

  xTaskCreatePinnedToCore(pid_task, "PID", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(udp_task, "UDP", 4096, NULL, 0, NULL, 0);

  Serial.println("SYSTEM READY (Task 2: dual IMU averaged)");
}

void loop() {
  static unsigned long lastSendTime = 0;
  if (millis() - lastSendTime < 50) return;
  lastSendTime = millis();

  if (!connectionEstablished) return;

  udp.beginPacket(laptopIP, laptopPort);
  udp.printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d,%d,%lu,%lu",
             angleX, angleY, angleZ,
             raw_gx, raw_gy, raw_gz,
             raw_ax, raw_ay, raw_az,
             base_throttle,
             (int)fault_rc,
             (int)fault_imu,
             rcTotalPkts,
             rcDroppedPkts);
  udp.endPacket();
}
