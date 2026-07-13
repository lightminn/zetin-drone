#include <Arduino.h>
#include <SPI.h>
#include <ICM42670P.h>

// SPI CS 핀 설정
#define SPI_CS1 10
#define SPI_CS2 9
ICM42670 IMU1(SPI, SPI_CS1);
ICM42670 IMU2(SPI, SPI_CS2);

// FC와 동일한 스케일 변환 상수
const float GYRO_SCALE = 1.0 / 16.4;
const float ACCEL_SCALE = 1.0 / 2048.0;

void setup() {
  Serial.begin(115200);

  // 두 IMU의 CS 핀을 먼저 HIGH로 만들어 SPI 버스 충돌 방지
  pinMode(SPI_CS1, OUTPUT);
  pinMode(SPI_CS2, OUTPUT);
  digitalWrite(SPI_CS1, HIGH);
  digitalWrite(SPI_CS2, HIGH);
  delay(1000);

  // SPI 핀 초기화 (SCK=12, MISO=13, MOSI=11, 기본 SS=10)
  SPI.begin(12, 13, 11, SPI_CS1);

  Serial.println("\n=======================================");
  Serial.println("🔍 Dual ICM42670 방향성(축/부호) 테스트 시작");
  Serial.println("=======================================");

  if (IMU1.begin() < 0) {
    Serial.println("💀 IMU1 초기화 실패! 선 연결을 확인하세요.");
    while (1) { delay(100); }
  }
  if (IMU2.begin() < 0) {
    Serial.println("💀 IMU2 초기화 실패! 선 연결을 확인하세요.");
    while (1) { delay(100); }
  }

  // FC와 동일한 해상도로 세팅
  IMU1.startAccel(1600, 16);
  IMU1.startGyro(1600, 2000);
  IMU2.startAccel(1600, 16);
  IMU2.startGyro(1600, 2000);

  Serial.println("✅ 센서 2개 준비 완료! 2초 뒤 출력을 시작합니다.");
  delay(2000);
}

void loop() {
  inv_imu_sensor_event_t imu1_event;
  inv_imu_sensor_event_t imu2_event;
  IMU1.getDataFromRegisters(imu1_event);
  IMU2.getDataFromRegisters(imu2_event);

  // IMU1 스케일 적용
  float ax1 = imu1_event.accel[0] * ACCEL_SCALE;
  float ay1 = imu1_event.accel[1] * ACCEL_SCALE;
  float az1 = imu1_event.accel[2] * ACCEL_SCALE;
  float gx1 = imu1_event.gyro[0] * GYRO_SCALE;
  float gy1 = imu1_event.gyro[1] * GYRO_SCALE;
  float gz1 = imu1_event.gyro[2] * GYRO_SCALE;

  // IMU2 스케일 적용
  float ax2 = imu2_event.accel[0] * ACCEL_SCALE;
  float ay2 = imu2_event.accel[1] * ACCEL_SCALE;
  float az2 = imu2_event.accel[2] * ACCEL_SCALE;
  float gx2 = imu2_event.gyro[0] * GYRO_SCALE;
  float gy2 = imu2_event.gyro[1] * GYRO_SCALE;
  float gz2 = imu2_event.gyro[2] * GYRO_SCALE;

  // IMU1 출력
  Serial.print("A1x:"); Serial.print(ax1); Serial.print("\t");
  Serial.print("A1y:"); Serial.print(ay1); Serial.print("\t");
  Serial.print("A1z:"); Serial.print(az1); Serial.print("\t");
  Serial.print("G1x:"); Serial.print(gx1); Serial.print("\t");
  Serial.print("G1y:"); Serial.print(gy1); Serial.print("\t");
  Serial.print("G1z:"); Serial.print(gz1); Serial.print("\t");

  // IMU2 출력
  Serial.print("A2x:"); Serial.print(ax2); Serial.print("\t");
  Serial.print("A2y:"); Serial.print(ay2); Serial.print("\t");
  Serial.print("A2z:"); Serial.print(az2); Serial.print("\t");
  Serial.print("G2x:"); Serial.print(gx2); Serial.print("\t");
  Serial.print("G2y:"); Serial.print(gy2); Serial.print("\t");
  Serial.print("G2z:"); Serial.println(gz2);

  delay(50); // 20Hz 출력
}
