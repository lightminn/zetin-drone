#include <Arduino.h>
#include <SPI.h>
#include <ICM42670P.h> // 설치하신 라이브러리 이름에 맞게 조정 (또는 <ICM42670.h>)

// ==========================================
// 1. 핀 맵핑
// ==========================================
#define SPI_MOSI 11
#define SPI_SCK  12
#define SPI_MISO 13

#define ICM1_CS  10
#define ICM2_CS  9

// 두 개의 독립된 IMU 객체 생성
ICM42670 IMU1(SPI, ICM1_CS);
ICM42670 IMU2(SPI, ICM2_CS);

// ==========================================
// 2. 변환 상수 (2000dps, 16G 기준)
// ==========================================
const float GYRO_SCALE = 1.0 / 16.4;
const float ACCEL_SCALE = 1.0 / 2048.0;

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("\n===========================================");
  Serial.println("🚀 [축] 듀얼 ICM42670 센서 동시 읽기 시작!");
  Serial.println("===========================================");

  // 🚨 [필수] SPI 초기화 전 두 센서 모두 대기 상태(HIGH)로 고정
  pinMode(ICM1_CS, OUTPUT); digitalWrite(ICM1_CS, HIGH);
  pinMode(ICM2_CS, OUTPUT); digitalWrite(ICM2_CS, HIGH);

  // SPI 버스 시작 (CS 핀은 하드웨어가 독점하지 못하게 -1 처리)
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, -1);

  // ---------------------------------------
  // 1번 센서 초기화
  // ---------------------------------------
  Serial.print("IMU1 (CS 10) 부팅 중... ");
  if (IMU1.begin() < 0) {
    Serial.println("실패 ❌");
  } else {
    Serial.println("성공 ✅");
    IMU1.startAccel(1600, 16);  // 1.6kHz, 16G
    IMU1.startGyro(1600, 2000); // 1.6kHz, 2000dps
  }

  // ---------------------------------------
  // 2번 센서 초기화
  // ---------------------------------------
  Serial.print("IMU2 (CS 9)  부팅 중... ");
  if (IMU2.begin() < 0) {
    Serial.println("실패 ❌");
  } else {
    Serial.println("성공 ✅");
    IMU2.startAccel(1600, 16);
    IMU2.startGyro(1600, 2000);
  }

  Serial.println("===========================================\n");
  delay(1000);
}

void loop() {
  inv_imu_sensor_event_t imu_evt1;
  inv_imu_sensor_event_t imu_evt2;

  // 두 센서에서 각각 데이터 읽기
  IMU1.getDataFromRegisters(imu_evt1);
  IMU2.getDataFromRegisters(imu_evt2);

  // 자이로 Z축 (회전 속도) 데이터 스케일링
  float gyroZ_1 = imu_evt1.gyro[2] * GYRO_SCALE;
  float gyroZ_2 = imu_evt2.gyro[2] * GYRO_SCALE;

  // 가속도 Z축 (중력 방향) 데이터 스케일링
  float accelZ_1 = imu_evt1.accel[2] * ACCEL_SCALE;
  float accelZ_2 = imu_evt2.accel[2] * ACCEL_SCALE;

  // 결과 출력 (비교하기 쉽게 나란히 출력)
  Serial.print("[Gyro Z] IMU1: ");
  Serial.print(gyroZ_1, 2);
  Serial.print("  |  IMU2: ");
  Serial.print(gyroZ_2, 2);

  Serial.print("    ///    [Accel Z] IMU1: ");
  Serial.print(accelZ_1, 2);
  Serial.print("  |  IMU2: ");
  Serial.println(accelZ_2, 2);

  delay(50); // 너무 빠르면 보기 힘드니 50ms 대기 (20Hz)
}