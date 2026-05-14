# PWM_TEST_DUAL_IMU_PID Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a new dual-IMU flight controller sketch that eliminates the in-flight drift seen in the single-IMU `PWM_TEST_IMU_PID.ino`, by adding startup gyro bias calibration, runtime bias estimation, dual IMU fusion with fault detection, and an adaptive complementary filter — while keeping the existing UDP protocol with `Drone_Tuning.py` 100% compatible.

**Architecture:** Single Arduino .ino file with FreeRTOS 2-task structure (Core 1: PID 1kHz, Core 0: UDP). Two ICM42670 IMUs on shared SPI (CS=10 and CS=9). Each task built incrementally: scaffold → dual read → bias calibration → fusion + fault detection → adaptive alpha → runtime bias estimation → final integration.

**Tech Stack:** Arduino framework on ESP32-S3, ICM42670P library, FreeRTOS, WiFi/UDP, ledcAttach PWM.

**Spec:** [`docs/superpowers/specs/2026-05-14-dual-imu-pid-design.md`](../specs/2026-05-14-dual-imu-pid-design.md)

## Testing Strategy for Firmware

This is hardware firmware — we cannot run unit tests. Verification at each task is:

1. **Compile check** with `arduino-cli compile` — programmatic, automatic
2. **Hardware verification** by the user — flash, observe serial monitor, manual test described in each task

Compile-failed = block before commit. Hardware verification is documented in each task; user runs it manually and reports back if issues.

## Compile Command (used in every task)

```bash
arduino-cli compile --fqbn esp32:esp32:esp32s3 firmware/examples/PWM_TEST_DUAL_IMU_PID/
```

Expected output: ends with `Used library ... Used platform ...` and exit code 0. Any error → fix before committing.

---

## File Structure

Single sketch file in new folder, matching existing examples/ pattern:

- **Create:** `firmware/examples/PWM_TEST_DUAL_IMU_PID/PWM_TEST_DUAL_IMU_PID.ino`

No other files. No headers or library split — keeps cohesion with existing examples and is short enough to hold in context.

---

## Task 1: Scaffold with dual IMU init + single read

**Goal:** Create the new folder and `.ino` file. Copy the baseline structure from `PWM_TEST_IMU_PID.ino` but with two `ICM42670` instances. Reads only IMU1 (averaging logic deferred to Task 3). This task produces a functionally-equivalent build to the original that already verifies IMU2 can be initialized alongside IMU1.

**Files:**
- Create: `firmware/examples/PWM_TEST_DUAL_IMU_PID/PWM_TEST_DUAL_IMU_PID.ino`

- [ ] **Step 1: Create the folder and file with the scaffold**

Write `firmware/examples/PWM_TEST_DUAL_IMU_PID/PWM_TEST_DUAL_IMU_PID.ino` with this content:

```cpp
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

  inv_imu_sensor_event_t e1;

  IMU1.getDataFromRegisters(e1);
  lpf_ax =  e1.accel[0] * ACCEL_SCALE;
  lpf_ay = -e1.accel[1] * ACCEL_SCALE;
  lpf_az =  e1.accel[2] * ACCEL_SCALE;
  lpf_gx =  e1.gyro[0]  * GYRO_SCALE;
  lpf_gy = -e1.gyro[1]  * GYRO_SCALE;
  lpf_gz =  e1.gyro[2]  * GYRO_SCALE;

  while (true) {
    unsigned long now = micros();
    if (now < nextLoopTime) { vTaskDelay(0); continue; }
    nextLoopTime = now + LOOP_INTERVAL;

    IMU1.getDataFromRegisters(e1);
    raw_ax =  e1.accel[0] * ACCEL_SCALE;
    raw_ay = -e1.accel[1] * ACCEL_SCALE;
    raw_az =  e1.accel[2] * ACCEL_SCALE;
    raw_gx =  e1.gyro[0]  * GYRO_SCALE;
    raw_gy = -e1.gyro[1]  * GYRO_SCALE;
    raw_gz =  e1.gyro[2]  * GYRO_SCALE;

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

  Serial.println("SYSTEM READY (Task 1 scaffold: dual IMU init, single IMU read)");
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
```

- [ ] **Step 2: Compile**

Run:
```bash
arduino-cli compile --fqbn esp32:esp32:esp32s3 firmware/examples/PWM_TEST_DUAL_IMU_PID/
```

Expected: exit 0, no errors. If errors, fix and re-run until clean.

- [ ] **Step 3: Hardware verification (user)**

User flashes and confirms serial shows `SYSTEM READY (Task 1 scaffold...)` and no `IMU1 INIT FAILED` / `IMU2 INIT FAILED`. Drone should not arm.

- [ ] **Step 4: Commit**

```bash
git add firmware/examples/PWM_TEST_DUAL_IMU_PID/PWM_TEST_DUAL_IMU_PID.ino
git commit -m "feat: scaffold PWM_TEST_DUAL_IMU_PID with dual IMU init"
```

---

## Task 2: Read both IMUs and average raw values (no bias yet)

**Goal:** Replace the single IMU1 read in `pid_task` with reads from both IMUs and use the simple average. No bias correction or fault detection yet — that comes in Task 3 and Task 4.

**Files:**
- Modify: `firmware/examples/PWM_TEST_DUAL_IMU_PID/PWM_TEST_DUAL_IMU_PID.ino` (PID task only)

- [ ] **Step 1: Modify the initial read in `pid_task`**

Replace the `inv_imu_sensor_event_t e1;` and the first read block (right after the comment "// 6. PID task...") with:

```cpp
  inv_imu_sensor_event_t e1, e2;

  IMU1.getDataFromRegisters(e1);
  IMU2.getDataFromRegisters(e2);
  lpf_ax =  ((e1.accel[0] + e2.accel[0]) * 0.5f) * ACCEL_SCALE;
  lpf_ay = -((e1.accel[1] + e2.accel[1]) * 0.5f) * ACCEL_SCALE;
  lpf_az =  ((e1.accel[2] + e2.accel[2]) * 0.5f) * ACCEL_SCALE;
  lpf_gx =  ((e1.gyro[0]  + e2.gyro[0])  * 0.5f) * GYRO_SCALE;
  lpf_gy = -((e1.gyro[1]  + e2.gyro[1])  * 0.5f) * GYRO_SCALE;
  lpf_gz =  ((e1.gyro[2]  + e2.gyro[2])  * 0.5f) * GYRO_SCALE;
```

- [ ] **Step 2: Modify the in-loop read in `pid_task`**

Replace the in-loop `IMU1.getDataFromRegisters(e1);` block (six raw_* assignments) with:

```cpp
    IMU1.getDataFromRegisters(e1);
    IMU2.getDataFromRegisters(e2);
    raw_ax =  ((e1.accel[0] + e2.accel[0]) * 0.5f) * ACCEL_SCALE;
    raw_ay = -((e1.accel[1] + e2.accel[1]) * 0.5f) * ACCEL_SCALE;
    raw_az =  ((e1.accel[2] + e2.accel[2]) * 0.5f) * ACCEL_SCALE;
    raw_gx =  ((e1.gyro[0]  + e2.gyro[0])  * 0.5f) * GYRO_SCALE;
    raw_gy = -((e1.gyro[1]  + e2.gyro[1])  * 0.5f) * GYRO_SCALE;
    raw_gz =  ((e1.gyro[2]  + e2.gyro[2])  * 0.5f) * GYRO_SCALE;
```

- [ ] **Step 3: Update ready message**

Replace the `Serial.println("SYSTEM READY (Task 1 ...)");` line with:

```cpp
  Serial.println("SYSTEM READY (Task 2: dual IMU averaged)");
```

- [ ] **Step 4: Compile**

```bash
arduino-cli compile --fqbn esp32:esp32:esp32s3 firmware/examples/PWM_TEST_DUAL_IMU_PID/
```

- [ ] **Step 5: Hardware verification (user)**

Flash, open Drone_Tuning.py. Observe telemetry: with drone stationary, `angleX`/`angleY` should be near 0 and stable. `raw_gx/y/z` should be similar magnitude as previous single-IMU version (averaging two same-direction IMUs gives the same mean, with less noise).

- [ ] **Step 6: Commit**

```bash
git add firmware/examples/PWM_TEST_DUAL_IMU_PID/PWM_TEST_DUAL_IMU_PID.ino
git commit -m "feat: read both IMUs and average raw values"
```

---

## Task 3: Startup gyro bias calibration

**Goal:** Add `calibrate_bias()` that runs in `setup()` after IMU init. Measures 2000 samples per IMU, checks for movement, stores per-IMU bias arrays (in **drone frame**, with IMU2 sign correction applied), retries up to 3 times.

> **Note (Task 2 fix):** IMU2 is mounted with x and z axes inverted vs IMU1; `IMU2_SIGN_X/Y/Z` constants were added in section 2 in the Task 2 fix. Task 3 must apply these signs at bias measurement so `gyro_bias2` is in the same (IMU1/drone) frame as `gyro_bias1`. Otherwise the bias subtraction during averaging would be in the wrong frame.

**Files:**
- Modify: `firmware/examples/PWM_TEST_DUAL_IMU_PID/PWM_TEST_DUAL_IMU_PID.ino`

- [ ] **Step 1: Add bias constants in section 2 (after `YAW_DEADZONE` line)**

```cpp
const int      BIAS_CALIB_SAMPLES         = 2000;
const float    BIAS_CALIB_MOVEMENT_THRESH = 1.0f;   // deg/s std-dev limit per axis
const int      BIAS_CALIB_RETRIES         = 3;
```

- [ ] **Step 2: Add bias state variables in section 3 (after `lpf_gx/y/z` line)**

```cpp
// Per-IMU gyro bias (deg/s), filled by calibrate_bias()
float gyro_bias1[3] = {0.0f, 0.0f, 0.0f};
float gyro_bias2[3] = {0.0f, 0.0f, 0.0f};
```

- [ ] **Step 3: Add `calibrate_bias()` function before `pid_task`**

Insert before the `// 6. PID task` comment:

```cpp
// ==========================================================
// Startup gyro bias calibration
// ==========================================================
// `sign` brings the IMU's gyro into the drone (IMU1) frame.
// For IMU1 pass {+1, +1, +1}; for IMU2 pass {IMU2_SIGN_X, IMU2_SIGN_Y, IMU2_SIGN_Z}.
static void measure_imu_bias(ICM42670 &imu, const float sign[3],
                             float bias_out[3], float stddev_out[3]) {
  double sum[3]    = {0.0, 0.0, 0.0};
  double sum_sq[3] = {0.0, 0.0, 0.0};
  inv_imu_sensor_event_t e;

  for (int i = 0; i < BIAS_CALIB_SAMPLES; i++) {
    imu.getDataFromRegisters(e);
    float g[3] = {
      sign[0] * e.gyro[0] * GYRO_SCALE,
      sign[1] * e.gyro[1] * GYRO_SCALE,
      sign[2] * e.gyro[2] * GYRO_SCALE
    };
    for (int k = 0; k < 3; k++) {
      sum[k]    += g[k];
      sum_sq[k] += (double)g[k] * g[k];
    }
    delayMicroseconds(1000);  // ~1 kHz sampling
  }

  for (int k = 0; k < 3; k++) {
    double mean = sum[k] / BIAS_CALIB_SAMPLES;
    double var  = (sum_sq[k] / BIAS_CALIB_SAMPLES) - (mean * mean);
    if (var < 0) var = 0;
    bias_out[k]   = (float)mean;
    stddev_out[k] = (float)sqrt(var);
  }
}

static const float IMU1_SIGN[3] = { 1.0f, 1.0f, 1.0f };
static const float IMU2_SIGN[3] = { IMU2_SIGN_X, IMU2_SIGN_Y, IMU2_SIGN_Z };

static void calibrate_bias() {
  float sd1[3], sd2[3];

  for (int attempt = 1; attempt <= BIAS_CALIB_RETRIES; attempt++) {
    Serial.printf("[CALIB] attempt %d/%d (hold still)...\n", attempt, BIAS_CALIB_RETRIES);

    measure_imu_bias(IMU1, IMU1_SIGN, gyro_bias1, sd1);
    measure_imu_bias(IMU2, IMU2_SIGN, gyro_bias2, sd2);

    float max_sd = max(max(max(sd1[0], sd1[1]), sd1[2]),
                       max(max(sd2[0], sd2[1]), sd2[2]));

    Serial.printf("[CALIB] IMU1 bias x=%.3f y=%.3f z=%.3f (sd %.3f %.3f %.3f)\n",
                  gyro_bias1[0], gyro_bias1[1], gyro_bias1[2], sd1[0], sd1[1], sd1[2]);
    Serial.printf("[CALIB] IMU2 bias x=%.3f y=%.3f z=%.3f (sd %.3f %.3f %.3f) [drone frame]\n",
                  gyro_bias2[0], gyro_bias2[1], gyro_bias2[2], sd2[0], sd2[1], sd2[2]);

    if (max_sd <= BIAS_CALIB_MOVEMENT_THRESH) {
      Serial.println("[CALIB] OK");
      return;
    }
    Serial.printf("[CALIB] movement detected (sd %.3f > %.3f), retrying\n",
                  max_sd, BIAS_CALIB_MOVEMENT_THRESH);
  }

  Serial.println("[CALIB] WARN: all retries failed, using last measurement");
}
```

After this step, `gyro_bias1` and `gyro_bias2` are both stored in **IMU1/drone frame** (with IMU2's axis sign correction applied during accumulation). Downstream code subtracts bias from sign-corrected gyro values — see Step 5.

- [ ] **Step 4: Call `calibrate_bias()` in setup() after IMU startAccel/startGyro and before xTaskCreatePinnedToCore**

In `setup()`, find the block:

```cpp
  IMU2.startAccel(1600, 16);
  IMU2.startGyro(1600, 2000);
  delay(500);

  xTaskCreatePinnedToCore(pid_task, "PID", 4096, NULL, 1, NULL, 1);
```

Change to:

```cpp
  IMU2.startAccel(1600, 16);
  IMU2.startGyro(1600, 2000);
  delay(500);

  calibrate_bias();

  xTaskCreatePinnedToCore(pid_task, "PID", 4096, NULL, 1, NULL, 1);
```

- [ ] **Step 5: Apply bias correction in `pid_task` reads**

In `pid_task`, change the in-loop dual read block to apply IMU2 sign correction and subtract per-IMU bias (bias is already in drone frame from Step 3). Replace the entire in-loop read block (the one from Task 2 that uses `IMU2_SIGN_X * e2.gyro[0]` etc) with:

```cpp
    IMU1.getDataFromRegisters(e1);
    IMU2.getDataFromRegisters(e2);

    // Sign-corrected (drone-frame) gyro reads, then subtract bias
    float gx1 =                  e1.gyro[0]  * GYRO_SCALE - gyro_bias1[0];
    float gy1 =                  e1.gyro[1]  * GYRO_SCALE - gyro_bias1[1];
    float gz1 =                  e1.gyro[2]  * GYRO_SCALE - gyro_bias1[2];
    float gx2 = IMU2_SIGN_X    * e2.gyro[0]  * GYRO_SCALE - gyro_bias2[0];
    float gy2 = IMU2_SIGN_Y    * e2.gyro[1]  * GYRO_SCALE - gyro_bias2[1];
    float gz2 = IMU2_SIGN_Z    * e2.gyro[2]  * GYRO_SCALE - gyro_bias2[2];

    // Accel: average sign-corrected reads, then apply drone Y flip at the end
    raw_ax =  ((e1.accel[0] + IMU2_SIGN_X * e2.accel[0]) * 0.5f) * ACCEL_SCALE;
    raw_ay = -((e1.accel[1] + IMU2_SIGN_Y * e2.accel[1]) * 0.5f) * ACCEL_SCALE;
    raw_az =  ((e1.accel[2] + IMU2_SIGN_Z * e2.accel[2]) * 0.5f) * ACCEL_SCALE;
    // Gyro: average bias-corrected per-IMU values, then apply drone Y flip
    raw_gx =  (gx1 + gx2) * 0.5f;
    raw_gy = -(gy1 + gy2) * 0.5f;
    raw_gz =  (gz1 + gz2) * 0.5f;
```

Reasoning: `gx1/gy1/gz1` and `gx2/gy2/gz2` are per-IMU gyro readings in each IMU's own frame after bias subtraction (bias was measured pre-Y-flip in Step 3). The outer Y negation on `raw_gy` brings the result to the drone's Y axis convention, matching the original sign behavior used by `atan2f` and PID.

Also replace the pre-loop init similarly:

```cpp
  IMU1.getDataFromRegisters(e1);
  IMU2.getDataFromRegisters(e2);

  float gx1 =                  e1.gyro[0]  * GYRO_SCALE - gyro_bias1[0];
  float gy1 =                  e1.gyro[1]  * GYRO_SCALE - gyro_bias1[1];
  float gz1 =                  e1.gyro[2]  * GYRO_SCALE - gyro_bias1[2];
  float gx2 = IMU2_SIGN_X    * e2.gyro[0]  * GYRO_SCALE - gyro_bias2[0];
  float gy2 = IMU2_SIGN_Y    * e2.gyro[1]  * GYRO_SCALE - gyro_bias2[1];
  float gz2 = IMU2_SIGN_Z    * e2.gyro[2]  * GYRO_SCALE - gyro_bias2[2];

  lpf_ax =  ((e1.accel[0] + IMU2_SIGN_X * e2.accel[0]) * 0.5f) * ACCEL_SCALE;
  lpf_ay = -((e1.accel[1] + IMU2_SIGN_Y * e2.accel[1]) * 0.5f) * ACCEL_SCALE;
  lpf_az =  ((e1.accel[2] + IMU2_SIGN_Z * e2.accel[2]) * 0.5f) * ACCEL_SCALE;
  lpf_gx =  (gx1 + gx2) * 0.5f;
  lpf_gy = -(gy1 + gy2) * 0.5f;
  lpf_gz =  (gz1 + gz2) * 0.5f;
```

Important sign note: bias for IMU1 (`gyro_bias1`) was measured on raw `e1.gyro[k] * GYRO_SCALE` (no Y flip). Same for IMU2 — but with sign correction applied during accumulation. So in pid_task we subtract bias from the same sign-corrected, pre-Y-flip values. The drone-frame Y flip is applied once at the very end to `raw_gy` and `lpf_gy`.

- [ ] **Step 6: Update ready message**

```cpp
  Serial.println("SYSTEM READY (Task 3: bias calibration)");
```

- [ ] **Step 7: Compile**

```bash
arduino-cli compile --fqbn esp32:esp32:esp32s3 firmware/examples/PWM_TEST_DUAL_IMU_PID/
```

- [ ] **Step 8: Hardware verification (user)**

Flash. On boot serial should show `[CALIB] attempt 1/3` then `[CALIB] IMU1 bias x=... y=... z=...` for both IMUs, then `[CALIB] OK`. Bias values should typically be in -2 ~ +2 deg/s range. After boot, with the drone stationary, `raw_gx/y/z` (telemetry) should hover very close to 0.

Then leave the drone sitting for 1-2 minutes (do **not** arm) and observe `angleZ` — it should drift much less than the previous version because yaw integration now uses bias-corrected gyro.

- [ ] **Step 9: Commit**

```bash
git add firmware/examples/PWM_TEST_DUAL_IMU_PID/PWM_TEST_DUAL_IMU_PID.ino
git commit -m "feat: startup gyro bias calibration for both IMUs"
```

---

## Task 4: Per-IMU fault detection with fallback + disagree check

**Goal:** Replace single `check_imu_frozen()` with per-IMU frozen detection that allows fallback, plus a new `check_imu_disagree()` that locks safety if the two IMUs disagree significantly.

**Files:**
- Modify: `firmware/examples/PWM_TEST_DUAL_IMU_PID/PWM_TEST_DUAL_IMU_PID.ino`

- [ ] **Step 1: Add new disagree constants in section 2**

After the `IMU_FROZEN_MS` line:

```cpp
const float    IMU_DISAGREE_GYRO  = 30.0f;  // deg/s
const float    IMU_DISAGREE_ACCEL = 0.5f;   // G
const uint32_t IMU_DISAGREE_MS    = 100;
```

- [ ] **Step 2: Replace single-IMU fault state with per-IMU state in section 3**

Find:

```cpp
volatile bool     fault_imu      = false;   // OR of fault_imu1/2 for telemetry compatibility
float             prev_gx        = 0.0f;
uint32_t          imuFrozenSince = 0;
```

Replace with:

```cpp
volatile bool fault_imu  = false;   // OR of fault_imu1/2 for telemetry compatibility
volatile bool fault_imu1 = false;
volatile bool fault_imu2 = false;

float    prev_gx1 = 0.0f, prev_gx2 = 0.0f;
uint32_t frozenSince1 = 0, frozenSince2 = 0;
uint32_t disagreeSince = 0;

// Latest per-IMU corrected gyro/accel (filled each loop)
float gx1_c = 0, gy1_c = 0, gz1_c = 0, ax1_c = 0, ay1_c = 0, az1_c = 0;
float gx2_c = 0, gy2_c = 0, gz2_c = 0, ax2_c = 0, ay2_c = 0, az2_c = 0;
```

- [ ] **Step 3: Replace `check_imu_frozen()` with per-IMU version**

Replace the entire `check_imu_frozen()` function with:

```cpp
// Returns true if BOTH IMUs frozen (lethal). Sets individual fault flags.
static bool check_imu_frozen() {
  bool f1 = (fabsf(gx1_c - prev_gx1) < IMU_FROZEN_THRESH);
  bool f2 = (fabsf(gx2_c - prev_gx2) < IMU_FROZEN_THRESH);

  if (f1) {
    if (frozenSince1 == 0) frozenSince1 = millis();
    if (millis() - frozenSince1 > IMU_FROZEN_MS) fault_imu1 = true;
  } else {
    frozenSince1 = 0;
    fault_imu1 = false;
  }

  if (f2) {
    if (frozenSince2 == 0) frozenSince2 = millis();
    if (millis() - frozenSince2 > IMU_FROZEN_MS) fault_imu2 = true;
  } else {
    frozenSince2 = 0;
    fault_imu2 = false;
  }

  prev_gx1 = gx1_c;
  prev_gx2 = gx2_c;

  fault_imu = fault_imu1 || fault_imu2;

  if (fault_imu1 && fault_imu2) {
    safety_lock = true;
    Serial.println("[FAULT] BOTH IMUS FROZEN");
    return true;
  }
  return false;
}

// Returns true if IMUs disagree beyond thresholds for >IMU_DISAGREE_MS.
static bool check_imu_disagree() {
  float dgx = fabsf(gx1_c - gx2_c);
  float dgy = fabsf(gy1_c - gy2_c);
  float dgz = fabsf(gz1_c - gz2_c);
  float dax = fabsf(ax1_c - ax2_c);
  float day = fabsf(ay1_c - ay2_c);
  float daz = fabsf(az1_c - az2_c);

  bool gyro_bad  = (dgx > IMU_DISAGREE_GYRO)  || (dgy > IMU_DISAGREE_GYRO)  || (dgz > IMU_DISAGREE_GYRO);
  bool accel_bad = (dax > IMU_DISAGREE_ACCEL) || (day > IMU_DISAGREE_ACCEL) || (daz > IMU_DISAGREE_ACCEL);

  if (gyro_bad || accel_bad) {
    if (disagreeSince == 0) disagreeSince = millis();
    if (millis() - disagreeSince > IMU_DISAGREE_MS) {
      // If one is already known frozen, trust the other one — skip lock
      if (fault_imu1 ^ fault_imu2) {
        return false;
      }
      safety_lock = true;
      Serial.printf("[FAULT] IMU DISAGREE (dg %.2f %.2f %.2f, da %.2f %.2f %.2f)\n",
                    dgx, dgy, dgz, dax, day, daz);
      return true;
    }
  } else {
    disagreeSince = 0;
  }
  return false;
}
```

- [ ] **Step 4: Restructure the in-loop read in `pid_task` to populate per-IMU corrected values and fuse with fallback**

The corrected values `gx1_c..az2_c` are stored in **drone frame** (sign-corrected for IMU2, Y already flipped). The fusion averages them and writes directly to `raw_*`.

Replace the in-loop dual read block from Task 3 with:

```cpp
    IMU1.getDataFromRegisters(e1);
    IMU2.getDataFromRegisters(e2);

    // Per-IMU gyro in drone frame (sign-corrected, Y-flipped, bias-subtracted)
    gx1_c =  (              e1.gyro[0] * GYRO_SCALE - gyro_bias1[0]);
    gy1_c = -(              e1.gyro[1] * GYRO_SCALE - gyro_bias1[1]);
    gz1_c =  (              e1.gyro[2] * GYRO_SCALE - gyro_bias1[2]);
    gx2_c =  (IMU2_SIGN_X * e2.gyro[0] * GYRO_SCALE - gyro_bias2[0]);
    gy2_c = -(IMU2_SIGN_Y * e2.gyro[1] * GYRO_SCALE - gyro_bias2[1]);
    gz2_c =  (IMU2_SIGN_Z * e2.gyro[2] * GYRO_SCALE - gyro_bias2[2]);

    // Per-IMU accel in drone frame (sign-corrected, Y-flipped)
    ax1_c =  (              e1.accel[0]) * ACCEL_SCALE;
    ay1_c = -(              e1.accel[1]) * ACCEL_SCALE;
    az1_c =  (              e1.accel[2]) * ACCEL_SCALE;
    ax2_c =  (IMU2_SIGN_X * e2.accel[0]) * ACCEL_SCALE;
    ay2_c = -(IMU2_SIGN_Y * e2.accel[1]) * ACCEL_SCALE;
    az2_c =  (IMU2_SIGN_Z * e2.accel[2]) * ACCEL_SCALE;

    // Fuse: average if both healthy, else use the healthy one
    if (fault_imu1 && !fault_imu2) {
      raw_gx = gx2_c; raw_gy = gy2_c; raw_gz = gz2_c;
      raw_ax = ax2_c; raw_ay = ay2_c; raw_az = az2_c;
    } else if (fault_imu2 && !fault_imu1) {
      raw_gx = gx1_c; raw_gy = gy1_c; raw_gz = gz1_c;
      raw_ax = ax1_c; raw_ay = ay1_c; raw_az = az1_c;
    } else {
      raw_gx = (gx1_c + gx2_c) * 0.5f;
      raw_gy = (gy1_c + gy2_c) * 0.5f;
      raw_gz = (gz1_c + gz2_c) * 0.5f;
      raw_ax = (ax1_c + ax2_c) * 0.5f;
      raw_ay = (ay1_c + ay2_c) * 0.5f;
      raw_az = (az1_c + az2_c) * 0.5f;
    }
```

Also update the pre-loop initial read similarly to populate `gx1_c..az2_c` and seed `lpf_*` from `raw_*` (which now exists after fusion):

Replace the pre-loop init from Task 3 with:

```cpp
  IMU1.getDataFromRegisters(e1);
  IMU2.getDataFromRegisters(e2);

  gx1_c =  (              e1.gyro[0] * GYRO_SCALE - gyro_bias1[0]);
  gy1_c = -(              e1.gyro[1] * GYRO_SCALE - gyro_bias1[1]);
  gz1_c =  (              e1.gyro[2] * GYRO_SCALE - gyro_bias1[2]);
  gx2_c =  (IMU2_SIGN_X * e2.gyro[0] * GYRO_SCALE - gyro_bias2[0]);
  gy2_c = -(IMU2_SIGN_Y * e2.gyro[1] * GYRO_SCALE - gyro_bias2[1]);
  gz2_c =  (IMU2_SIGN_Z * e2.gyro[2] * GYRO_SCALE - gyro_bias2[2]);
  ax1_c =  (              e1.accel[0]) * ACCEL_SCALE;
  ay1_c = -(              e1.accel[1]) * ACCEL_SCALE;
  az1_c =  (              e1.accel[2]) * ACCEL_SCALE;
  ax2_c =  (IMU2_SIGN_X * e2.accel[0]) * ACCEL_SCALE;
  ay2_c = -(IMU2_SIGN_Y * e2.accel[1]) * ACCEL_SCALE;
  az2_c =  (IMU2_SIGN_Z * e2.accel[2]) * ACCEL_SCALE;

  lpf_ax = (ax1_c + ax2_c) * 0.5f;
  lpf_ay = (ay1_c + ay2_c) * 0.5f;
  lpf_az = (az1_c + az2_c) * 0.5f;
  lpf_gx = (gx1_c + gx2_c) * 0.5f;
  lpf_gy = (gy1_c + gy2_c) * 0.5f;
  lpf_gz = (gz1_c + gz2_c) * 0.5f;
```

- [ ] **Step 5: Update the fault gate in pid loop**

Find the line:

```cpp
    if (check_imu_frozen() || check_rc_timeout() || check_attitude() || safety_lock) {
```

Replace with:

```cpp
    bool fatal = check_imu_frozen();
    fatal      = fatal || check_imu_disagree();
    if (fatal || check_rc_timeout() || check_attitude() || safety_lock) {
```

- [ ] **Step 6: Reset per-IMU faults on `start` command in udp_task**

Find:

```cpp
    else if (cmd == "start") {
      fault_rc = fault_imu = false;
```

Replace with:

```cpp
    else if (cmd == "start") {
      fault_rc = fault_imu = fault_imu1 = fault_imu2 = false;
      frozenSince1 = frozenSince2 = disagreeSince = 0;
```

- [ ] **Step 7: Update ready message**

```cpp
  Serial.println("SYSTEM READY (Task 4: per-IMU fault detect + fusion fallback)");
```

- [ ] **Step 8: Compile**

```bash
arduino-cli compile --fqbn esp32:esp32:esp32s3 firmware/examples/PWM_TEST_DUAL_IMU_PID/
```

- [ ] **Step 9: Hardware verification (user)**

Two tests, no propellers attached:

a) **Both IMUs working**: serial should not show any FAULT messages, telemetry `fault_imu` should be 0.

b) **One IMU disconnect test**: With drone powered off, disconnect IMU2 (CS=9). Power on. Boot will log `[CALIB]` with IMU2 reading zeros or noise; IMU1 should be fine. After boot, serial should eventually log `[FAULT] IMU FROZEN`-style messages for IMU2 (or the disagree check may trigger first). Telemetry `fault_imu` becomes 1 but the loop continues to feed IMU1 data into `raw_*`. Drone should NOT arm (safety_lock from disagree if it fires). Power off, reconnect IMU2, repeat: confirm fault clears on next boot.

- [ ] **Step 10: Commit**

```bash
git add firmware/examples/PWM_TEST_DUAL_IMU_PID/PWM_TEST_DUAL_IMU_PID.ino
git commit -m "feat: per-IMU fault detect with fallback and disagree check"
```

---

## Task 5: Adaptive complementary filter

**Goal:** Replace fixed `ALPHA_COMP = 0.995f` with a function that returns 0.99, 0.995, or 0.999 based on accelerometer deviation from 1G.

**Files:**
- Modify: `firmware/examples/PWM_TEST_DUAL_IMU_PID/PWM_TEST_DUAL_IMU_PID.ino`

- [ ] **Step 1: Replace `ALPHA_COMP` constant with the three-level constants and threshold constants**

Find:

```cpp
const float ALPHA_COMP = 0.995f;  // temporary; replaced in Task 5
```

Replace with:

```cpp
const float ACCEL_DEV_SOFT = 0.1f;   // G
const float ACCEL_DEV_HARD = 0.3f;   // G
const float ALPHA_STATIC   = 0.99f;
const float ALPHA_NORMAL   = 0.995f;
const float ALPHA_DYNAMIC  = 0.999f;

static inline float compute_alpha(float ax, float ay, float az) {
  float mag = sqrtf(ax*ax + ay*ay + az*az);
  float dev = fabsf(mag - 1.0f);
  if      (dev < ACCEL_DEV_SOFT) return ALPHA_STATIC;
  else if (dev < ACCEL_DEV_HARD) return ALPHA_NORMAL;
  else                            return ALPHA_DYNAMIC;
}
```

- [ ] **Step 2: Use `compute_alpha()` in the complementary filter**

In `pid_task`, find the lines that compute `angleX`/`angleY`:

```cpp
    angleX = ALPHA_COMP * (angleX + lpf_gx * dt) + (1.0f - ALPHA_COMP) * accAngleX;
    angleY = ALPHA_COMP * (angleY + lpf_gy * dt) + (1.0f - ALPHA_COMP) * accAngleY;
```

Replace with:

```cpp
    float alpha = compute_alpha(lpf_ax, lpf_ay, lpf_az);
    angleX = alpha * (angleX + lpf_gx * dt) + (1.0f - alpha) * accAngleX;
    angleY = alpha * (angleY + lpf_gy * dt) + (1.0f - alpha) * accAngleY;
```

- [ ] **Step 3: Update ready message**

```cpp
  Serial.println("SYSTEM READY (Task 5: adaptive complementary filter)");
```

- [ ] **Step 4: Compile**

```bash
arduino-cli compile --fqbn esp32:esp32:esp32s3 firmware/examples/PWM_TEST_DUAL_IMU_PID/
```

- [ ] **Step 5: Hardware verification (user)**

Flash. With drone stationary, `angleX`/`angleY` in telemetry should be **less drift** over a minute than Task 3 (because alpha=0.99 boosts accel correction). When you hand-shake the drone vigorously, angles should not go wild (alpha=0.999 makes it gyro-dominated during chaos).

- [ ] **Step 6: Commit**

```bash
git add firmware/examples/PWM_TEST_DUAL_IMU_PID/PWM_TEST_DUAL_IMU_PID.ino
git commit -m "feat: adaptive complementary filter based on accel magnitude"
```

---

## Task 6: Runtime slow bias estimation

**Goal:** While the drone is idle (`base_throttle < 1100`) AND actually stationary (`|gyro| < 2 deg/s`), slowly update `gyro_bias1` and `gyro_bias2` with EMA α=0.0005.

**Files:**
- Modify: `firmware/examples/PWM_TEST_DUAL_IMU_PID/PWM_TEST_DUAL_IMU_PID.ino`

- [ ] **Step 1: Add runtime bias constants in section 2**

After the disagree constants:

```cpp
const float RUNTIME_BIAS_ALPHA      = 0.0005f;
const float RUNTIME_BIAS_GYRO_LIMIT = 2.0f;   // deg/s
```

- [ ] **Step 2: Add helper at top of `pid_task` (just after `inv_imu_sensor_event_t e1, e2;` declaration)**

Add inside `pid_task`, after the local declarations and before the pre-loop initial read:

```cpp
  auto maybe_update_bias = [&](float sgx, float sgy, float sgz,
                               float (&bias)[3]) {
    // sgx/sgy/sgz are sign-corrected pre-Y-flip gyro (drone frame except Y flip),
    // matching how bias was measured in setup. Bias is stored in the same frame.
    if (base_throttle >= 1100) return;
    if (fabsf(sgx - bias[0]) > RUNTIME_BIAS_GYRO_LIMIT) return;
    if (fabsf(sgy - bias[1]) > RUNTIME_BIAS_GYRO_LIMIT) return;
    if (fabsf(sgz - bias[2]) > RUNTIME_BIAS_GYRO_LIMIT) return;
    bias[0] = bias[0] * (1.0f - RUNTIME_BIAS_ALPHA) + sgx * RUNTIME_BIAS_ALPHA;
    bias[1] = bias[1] * (1.0f - RUNTIME_BIAS_ALPHA) + sgy * RUNTIME_BIAS_ALPHA;
    bias[2] = bias[2] * (1.0f - RUNTIME_BIAS_ALPHA) + sgz * RUNTIME_BIAS_ALPHA;
  };
```

- [ ] **Step 3: Call the helper after each in-loop IMU read**

In `pid_task`, immediately after the in-loop reads:

```cpp
    IMU1.getDataFromRegisters(e1);
    IMU2.getDataFromRegisters(e2);
```

Add right below (apply IMU2 sign correction so the values match how bias was measured):

```cpp
    maybe_update_bias(              e1.gyro[0] * GYRO_SCALE,
                                    e1.gyro[1] * GYRO_SCALE,
                                    e1.gyro[2] * GYRO_SCALE,
                                    gyro_bias1);
    maybe_update_bias(IMU2_SIGN_X * e2.gyro[0] * GYRO_SCALE,
                      IMU2_SIGN_Y * e2.gyro[1] * GYRO_SCALE,
                      IMU2_SIGN_Z * e2.gyro[2] * GYRO_SCALE,
                      gyro_bias2);
```

- [ ] **Step 4: Update ready message**

```cpp
  Serial.println("SYSTEM READY (Task 6: runtime bias estimation)");
```

- [ ] **Step 5: Compile**

```bash
arduino-cli compile --fqbn esp32:esp32:esp32s3 firmware/examples/PWM_TEST_DUAL_IMU_PID/
```

- [ ] **Step 6: Hardware verification (user)**

Flash. After boot, leave drone stationary for 10 minutes (idle, do not arm). The startup bias is still in the right ballpark and now the runtime EMA further refines it. To verify by inspection, you can temporarily add `Serial.printf` for bias values every 5 seconds; this is optional. The functional check: `angleZ` drift over 10 minutes should be smaller than Task 5 alone.

- [ ] **Step 7: Commit**

```bash
git add firmware/examples/PWM_TEST_DUAL_IMU_PID/PWM_TEST_DUAL_IMU_PID.ino
git commit -m "feat: runtime slow gyro bias estimation while idle"
```

---

## Task 7: Final hover flight test + tuning verification

**Goal:** Verify end-to-end behavior on the actual drone. Update the ready message to a final form and confirm the integration. No code changes other than the message.

**Files:**
- Modify: `firmware/examples/PWM_TEST_DUAL_IMU_PID/PWM_TEST_DUAL_IMU_PID.ino`

- [ ] **Step 1: Update ready message to final form**

```cpp
  Serial.println("PWM_TEST_DUAL_IMU_PID READY");
```

- [ ] **Step 2: Compile**

```bash
arduino-cli compile --fqbn esp32:esp32:esp32s3 firmware/examples/PWM_TEST_DUAL_IMU_PID/
```

- [ ] **Step 3: Hardware verification (user)** — propeller-OFF tests first

a) Power on, confirm `[CALIB] OK` and `PWM_TEST_DUAL_IMU_PID READY`.
b) Launch `Drone_Tuning.py`. Confirm telemetry arrives, `fault_imu=0`, all 14 fields parse.
c) Send `start` via the tuning script. Confirm motors arm (base_throttle=1100), `fault_imu=0`. Send `stop`. Confirm disarm.
d) Test PID tuning commands: `pa 2.0`, then `pa 2.5`. Confirm the script reports updated gains.

- [ ] **Step 4: Hardware verification (user)** — propellers ON, tethered or in safe area

a) Hover and observe whether the previous drift-to-one-side behavior reproduces. Expected: significantly reduced.
b) If drift remains, capture `angleX`/`angleY` from telemetry log and report — may need to revisit alpha thresholds or PID gains.

- [ ] **Step 5: Commit**

```bash
git add firmware/examples/PWM_TEST_DUAL_IMU_PID/PWM_TEST_DUAL_IMU_PID.ino
git commit -m "chore: finalize PWM_TEST_DUAL_IMU_PID ready message"
```
