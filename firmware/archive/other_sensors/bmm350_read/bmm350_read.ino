#include <Arduino.h>
#include <Wire.h>
#include <DFRobot_BMM350.h>

#define BMM_SDA 35
#define BMM_SCK 36
#define LDO_SHD 38

// 객체 생성 (기본 주소 0x14)
DFRobot_BMM350_I2C bmm(&Wire, 0x14);

void setup() {
  Serial.begin(115200);

  // 1. LDO 켜기 및 전압 안정화
  pinMode(LDO_SHD, OUTPUT);
  digitalWrite(LDO_SHD, HIGH);
  delay(100);

  Serial.println("\n=======================================");
  Serial.println("🧭 BMM350 지자기 데이터 읽기 테스트");
  Serial.println("=======================================");

  // 2. I2C 통신 시작
  Wire.begin(BMM_SDA, BMM_SCK);

  // 3. 센서 연결 확인
  Serial.print("BMM350 연결 중... ");
  while (bmm.begin() != 0) {
    Serial.println("실패 ❌ (I2C 라인이나 전원을 확인하세요)");
    delay(1000);
  }
  Serial.println("성공 ✅");

  // 🚨 4. DFRobot 전용 킹받는 파라미터명으로 수정 완료!
  bmm.setOperationMode(eBmm350NormalMode);

  Serial.println("초기화 완료! 자기장 데이터를 스캔합니다.\n");
  delay(1000);
}

void loop() {
  sBmm350MagData_t magData = bmm.getGeomagneticData();

  // 시리얼 모니터 출력
  Serial.print("자기장 X: ");
  Serial.print(magData.x);
  Serial.print(" uT   |   Y: ");
  Serial.print(magData.y);
  Serial.print(" uT   |   Z: ");
  Serial.print(magData.z);
  Serial.println(" uT");

  delay(100);
}