#include <Arduino.h>
#include <Wire.h>
#include <DFRobot_BMM350.h>

// 핀 맵핑
#define BMM_SDA 35
#define BMM_SCK 36
#define LDO_SHD 38

// BMM350 I2C 객체 생성 (아까 스캐너에서 확인한 기본 주소 0x14 사용)
DFRobot_BMM350_I2C bmm(&Wire, 0x14);

void setup() {
  Serial.begin(115200);

  // 1. LDO 켜기 (센서 밥부터 줌)
  pinMode(LDO_SHD, OUTPUT);
  digitalWrite(LDO_SHD, HIGH);
  delay(100); // 전압 안정화 및 센서 부팅 대기

  Serial.println("\n=======================================");
  Serial.println("🧭 BMM350 지자기 센서 데이터 읽기 테스트");
  Serial.println("=======================================");

  // 2. I2C 통신 시작
  Wire.begin(BMM_SDA, BMM_SCK);

  // 3. 센서 라이브러리 초기화
  Serial.print("BMM350 연결 중... ");
  // begin() 함수가 0을 반환하면 성공, 아니면 에러
  while (bmm.begin() != 0) {
    Serial.println("실패 ❌ (I2C 라인이나 전원을 확인하세요)");
    delay(1000);
  }
  Serial.println("성공 ✅");

  // 4. 동작 모드 설정 (Normal Mode: 연속해서 계속 측정하는 모드)
  bmm.setOperationMode(eBmm350NormalMode);

  // (참고) ODR(출력 데이터 속도) 설정도 가능합니다. 기본값으로 둡니다.
  // bmm.setODRAndOversampling(BMM350_ODR_100HZ);

  Serial.println("초기화 완료! 자기장 데이터를 스캔합니다.\n");
  delay(1000);
}

void loop() {
  // 5. 지자기 데이터 읽어오기 (구조체로 한 번에 받아옴)
  sBmm350MagData_t magData = bmm.getGeomagneticData();

  // 6. 시리얼 모니터에 보기 좋게 출력 (단위: uT - 마이크로테슬라)
  Serial.print("자기장 X: ");
  Serial.print(magData.x);
  Serial.print(" uT   |   Y: ");
  Serial.print(magData.y);
  Serial.print(" uT   |   Z: ");
  Serial.print(magData.z);
  Serial.println(" uT");

  // 보드를 이리저리 빙글빙글 돌려보기 편하게 0.1초 대기
  delay(100);
}
