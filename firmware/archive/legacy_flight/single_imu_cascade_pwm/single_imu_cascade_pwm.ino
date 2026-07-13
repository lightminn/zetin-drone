#include <Arduino.h>
#include <SPI.h>
#include <ICM42670P.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// ==========================================================
// 🚨 [추가됨] Low Pass Filter 클래스 (D항 노이즈 제거용)
// ==========================================================
class LowPassFilter {
public:
    float alpha;
    float lastOutput = 0;

    // cutoff_freq: 통과시킬 최대 주파수 (보통 30~50Hz 사용)
    // dt: 루프 타임 (1kHz = 0.001s)
    LowPassFilter(float cutoff_freq, float dt) {
        float rc = 1.0f / (2.0f * PI * cutoff_freq);
        alpha = dt / (rc + dt);
    }

    float update(float input) {
        lastOutput = lastOutput + alpha * (input - lastOutput);
        return lastOutput;
    }
};

// ==========================================================
// 1. 튜닝 변수 (이중 루프 구조)
// ==========================================================
volatile float Kp_Angle_Roll = 6.0;
volatile float Kp_Angle_Pitch = 6.0;
volatile float Kp_Angle_Yaw = 3.0;

volatile float Kp_Rate_Roll = 0.5;
volatile float Ki_Rate_Roll = 0.005;
volatile float Kd_Rate_Roll = 0.00; // 이제 0.01 단위로 올리셔도 됩니다!

volatile float Kp_Rate_Pitch = 0.5;
volatile float Ki_Rate_Pitch = 0.005;
volatile float Kd_Rate_Pitch = 0.00; // 이제 0.01 단위로 올리셔도 됩니다!

volatile float Kp_Rate_Yaw = 1.5;
volatile float Ki_Rate_Yaw = 0.05;
volatile float Kd_Rate_Yaw = 0.0;

const float ACC_MARGIN = 0.15;

volatile int base_throttle = 1000;
volatile int min_throttle = 1050;
volatile int max_throttle = 1300;

volatile bool yaw_enabled = true;

// ==========================================================
// 2. 시스템 변수
// ==========================================================
const char *ssid = "Drone_Tuning";
const char *password = "12345678";
WiFiUDP udp;
const int udpPort = 4210;
char packetBuffer[255];
IPAddress laptopIP;
int laptopPort = 0;
bool connectionEstablished = false;

// 모터 레이아웃
const int pinM1 = 4; // FL (앞/왼쪽), CW
const int pinM2 = 5; // RR (뒤/오른쪽), CW
const int pinM3 = 6; // FR (앞/오른쪽), CCW
const int pinM4 = 7; // RL (뒤/왼쪽), CCW

#define SPI_CS 10
ICM42670 IMU(SPI, SPI_CS);

const int ESC_FREQ = 400; const int ESC_RES = 14; const int ESC_PERIOD = 2500;

volatile bool safety_lock = true;
volatile float targetAngleX = 0.0, targetAngleY = 0.0, targetAngleZ = 0.0;

float angleX = 0, angleY = 0, angleZ = 0;
float gyroX = 0, gyroY = 0, gyroZ = 0;

float errorSumRateRoll = 0, errorSumRatePitch = 0, errorSumRateYaw = 0;

const float GYRO_SCALE = 1.0 / 16.4;
const float ACCEL_SCALE = 1.0 / 2048.0;

void writeMotor(int pin, int us) {
    us = constrain(us, 1000, 2000);
    ledcWrite(pin, (us * 16383) / ESC_PERIOD);
}

void stopMotors() {
    writeMotor(pinM1, 1000); writeMotor(pinM2, 1000);
    writeMotor(pinM3, 1000); writeMotor(pinM4, 1000);
}

// ==========================================================
// [Core 1] PID 태스크
// ==========================================================
void pid_task(void *pvParameters) {
    IMU.startAccel(1600, 16);
    IMU.startGyro(1600, 2000);

    inv_imu_sensor_event_t imu_event;
    float prevErrorRateRoll = 0, prevErrorRatePitch = 0;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1);
    const float dt = 0.001;

    // 🚨 [추가됨] D항 필터 객체 생성 (Cutoff 40Hz, Loop 1kHz)
    LowPassFilter lpfD_Roll(40.0, dt);
    LowPassFilter lpfD_Pitch(40.0, dt);

    while (true) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // 1. 센서 읽기 (방향 보정 완료)
        IMU.getDataFromRegisters(imu_event);

        float ax = -(imu_event.accel[0] * ACCEL_SCALE);
        float gyroX = -(imu_event.gyro[0] * GYRO_SCALE);

        float ay = -(imu_event.accel[1] * ACCEL_SCALE);
        float gyroY = -(imu_event.gyro[1] * GYRO_SCALE);

        float az =  (imu_event.accel[2] * ACCEL_SCALE);
        float gyroZ = -(imu_event.gyro[2] * GYRO_SCALE);

        // 2. 자세 추정
        angleX += gyroX * dt;
        angleY += gyroY * dt;
        if (abs(gyroZ) > 0.3) angleZ += gyroZ * dt;

        float accMagnitude = sqrt(ax*ax + ay*ay + az*az);
        if (abs(accMagnitude - 1.0) < ACC_MARGIN) {
            float accAngleX = atan2(ay, az) * 180.0 / PI;
            float accAngleY = atan2(-ax, sqrt(ay*ay + az*az)) * 180.0 / PI;

            angleX = angleX * 0.995 + accAngleX * 0.005;
            angleY = angleY * 0.995 + accAngleY * 0.005;
        }

        // 3. PID 계산
        if (abs(angleX) > 60 || abs(angleY) > 60) safety_lock = true;

        if (!safety_lock) {
            float targetRateRoll  = (targetAngleX - angleX) * Kp_Angle_Roll;
            float targetRatePitch = (targetAngleY - angleY) * Kp_Angle_Pitch;
            float targetRateYaw   = (targetAngleZ - angleZ) * Kp_Angle_Yaw;

            float errorRateRoll  = targetRateRoll  - gyroX;
            float errorRatePitch = targetRatePitch - gyroY;
            float errorRateYaw   = targetRateYaw   - gyroZ;

            if (base_throttle > 1100) {
                errorSumRateRoll  = constrain(errorSumRateRoll  + errorRateRoll  * dt, -200, 200);
                errorSumRatePitch = constrain(errorSumRatePitch + errorRatePitch * dt, -200, 200);
                errorSumRateYaw   = constrain(errorSumRateYaw   + errorRateYaw   * dt, -200, 200);
            } else {
                errorSumRateRoll = 0; errorSumRatePitch = 0; errorSumRateYaw = 0;
            }

            // 🚨 [수정됨] 미분값 계산 후 LPF 통과
            float rawDTermRoll  = (errorRateRoll - prevErrorRateRoll) / dt;
            float rawDTermPitch = (errorRatePitch - prevErrorRatePitch) / dt;

            float dTermRoll  = lpfD_Roll.update(rawDTermRoll);
            float dTermPitch = lpfD_Pitch.update(rawDTermPitch);

            prevErrorRateRoll = errorRateRoll;
            prevErrorRatePitch = errorRatePitch;

            float pidRoll  = (errorRateRoll * Kp_Rate_Roll)   + (errorSumRateRoll * Ki_Rate_Roll)   + (dTermRoll * Kd_Rate_Roll);
            float pidPitch = (errorRatePitch * Kp_Rate_Pitch) + (errorSumRatePitch * Ki_Rate_Pitch) + (dTermPitch * Kd_Rate_Pitch);

            float pidYaw = 0.0;
            if (yaw_enabled) {
                pidYaw = (errorRateYaw * Kp_Rate_Yaw) + (errorSumRateYaw * Ki_Rate_Yaw);
            }

            // 4. 모터 믹싱
            int pwm1 = base_throttle - pidPitch + pidRoll - pidYaw;
            int pwm2 = base_throttle + pidPitch - pidRoll - pidYaw;
            int pwm3 = base_throttle - pidPitch - pidRoll + pidYaw;
            int pwm4 = base_throttle + pidPitch + pidRoll + pidYaw;

            writeMotor(pinM1, constrain(pwm1, min_throttle, max_throttle));
            writeMotor(pinM2, constrain(pwm2, min_throttle, max_throttle));
            writeMotor(pinM3, constrain(pwm3, min_throttle, max_throttle));
            writeMotor(pinM4, constrain(pwm4, min_throttle, max_throttle));
        } else {
            stopMotors();
        }
    }
}

// ==========================================================
// [Core 0] UDP 통신 태스크
// ==========================================================
void udp_task(void *pvParameters) {
    const int CTRL_MARGIN = 150;
    while (true) {
        int packetSize = udp.parsePacket();
        if (packetSize) {
            laptopIP = udp.remoteIP();
            laptopPort = udp.remotePort();
            connectionEstablished = true;
            int len = udp.read(packetBuffer, 255);
            if (len > 0) packetBuffer[len] = 0;
            String cmd = String(packetBuffer);
            cmd.trim();

            if (cmd.startsWith("rc")) {
                int s1 = cmd.indexOf(' ');
                int s2 = cmd.indexOf(' ', s1 + 1);
                if (s1 > 0 && s2 > 0) {
                    targetAngleX = cmd.substring(s1 + 1, s2).toFloat();
                    targetAngleY = cmd.substring(s2 + 1).toFloat();
                }
            }
            else if (cmd.startsWith("yaw")) {
                int state = cmd.substring(4).toInt();
                yaw_enabled = (state == 1);
                Serial.printf(">>> Yaw Control: %s\n", yaw_enabled ? "ON" : "OFF");
            }
            else {
                float val = cmd.substring(2).toFloat();
                if (cmd.startsWith("rp")) { Kp_Rate_Roll = val; Kp_Rate_Pitch = val; }
                else if (cmd.startsWith("rd")) { Kd_Rate_Roll = val; Kd_Rate_Pitch = val; }
                else if (cmd.startsWith("ri")) { Ki_Rate_Roll = val; Ki_Rate_Pitch = val; }
                else if (cmd.startsWith("ap")) { Kp_Angle_Roll = val; Kp_Angle_Pitch = val; }

                else if (cmd.startsWith("start")) {
                    safety_lock = false;
                    base_throttle = 1100; min_throttle = 1050; max_throttle = 1250;
                    angleX=0; angleY=0; angleZ=0;
                    errorSumRateRoll=0; errorSumRatePitch=0; errorSumRateYaw=0;
                }
                else if (cmd.startsWith("stop")) {
                    safety_lock = true; base_throttle = 1000;
                }
                else if (cmd.startsWith("th")) {
                    int new_base = (int)val;
                    base_throttle = new_base;
                    min_throttle = max(1050, new_base - CTRL_MARGIN);
                    max_throttle = min(1900, new_base + CTRL_MARGIN);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void setup() {
    pinMode(9,OUTPUT); digitalWrite(9,HIGH); delay(100);
    Serial.begin(115200);
    WiFi.softAP(ssid, password);
    udp.begin(udpPort);
    SPI.begin(12, 13, 11, 10);

    ledcAttach(pinM1, ESC_FREQ, ESC_RES);
    ledcAttach(pinM2, ESC_FREQ, ESC_RES);
    ledcAttach(pinM3, ESC_FREQ, ESC_RES);
    ledcAttach(pinM4, ESC_FREQ, ESC_RES);
    stopMotors();

    if (IMU.begin() < 0) { while (1) { Serial.println("IMU Fail"); delay(1000); } }
    delay(1000);

    xTaskCreatePinnedToCore(pid_task, "PID", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(udp_task, "UDP", 4096, NULL, 0, NULL, 0);
    Serial.println("SYSTEM READY (Added D-Term LPF)");
}

void loop() {
    static unsigned long lastSendTime = 0;

    if (millis() - lastSendTime > 50) {
        lastSendTime = millis();
        if (connectionEstablished) {
            udp.beginPacket(laptopIP, laptopPort);
            udp.printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d",
                       angleX, angleY, angleZ,
                       gyroX, gyroY, gyroZ,
                       0.0, 0.0, 0.0,
                       base_throttle);
            udp.endPacket();
        }
    }
}