/*
 * motor_id_single.ino
 *
 * WARNING: 프로펠러 제거(props OFF) 상태에서만 사용한다.
 * docs/power_on_bench_procedure.md의 Stage A에서 GPIO→물리 모터 매핑과
 * 회전 방향을 한 번에 한 모터씩 확인하기 위한 진단 펌웨어다.
 *
 * 회전 방향표(기체를 위에서 내려다본 기준):
 *   M1 = GPIO4, FL(전-좌), expected CW
 *   M2 = GPIO5, RR(후-우), expected CW
 *   M3 = GPIO6, FR(전-우), expected CCW
 *   M4 = GPIO7, RL(후-좌), expected CCW
 */

#include <ESP32Servo.h>

// 모터 객체 생성
Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;

// 핀 번호와 물리 모터 배치(ESP32-S3)
const int pin1 = 4;  // M1 FL, CW
const int pin2 = 5;  // M2 RR, CW
const int pin3 = 6;  // M3 FR, CCW
const int pin4 = 7;  // M4 RL, CCW

// ==========================================
// [안전 설정] PWM 범위와 진단 하드 상한
// ==========================================
const int MIN_THROTTLE = 1000;      // idle
const int MAX_THROTTLE = 2000;      // ESC attach 범위
const int DEFAULT_SETPOINT = 1120;  // 모터 선택 시 저속 기본값
const int CAP = 1250;               // 25% 하드 상한
const int SETPOINT_STEP = 10;
const int MOTOR_COUNT = 4;

static_assert(DEFAULT_SETPOINT >= MIN_THROTTLE && DEFAULT_SETPOINT <= CAP,
              "DEFAULT_SETPOINT must stay inside the safe range");
static_assert(CAP <= MAX_THROTTLE, "CAP must not exceed MAX_THROTTLE");

Servo *const motors[MOTOR_COUNT] = {&motor1, &motor2, &motor3, &motor4};
const int motorPins[MOTOR_COUNT] = {pin1, pin2, pin3, pin4};
const char *const motorNames[MOTOR_COUNT] = {"M1", "M2", "M3", "M4"};
const char *const motorPositions[MOTOR_COUNT] = {"FL", "RR", "FR", "RL"};
const char *const motorDirections[MOTOR_COUNT] = {"CW", "CW", "CCW", "CCW"};

int activeMotor = -1;
int setpointUs = DEFAULT_SETPOINT;

// 모든 출력 갱신은 이 함수를 거쳐 한 모터만 idle을 초과하게 한다.
void writeAllIdle() {
  motor1.writeMicroseconds(MIN_THROTTLE);
  motor2.writeMicroseconds(MIN_THROTTLE);
  motor3.writeMicroseconds(MIN_THROTTLE);
  motor4.writeMicroseconds(MIN_THROTTLE);
}

void applyOutputs() {
  // 상태가 손상되어도 실제 출력은 항상 [1000, CAP] 안으로 제한한다.
  if (setpointUs < MIN_THROTTLE) {
    setpointUs = MIN_THROTTLE;
  } else if (setpointUs > CAP) {
    setpointUs = CAP;
  }

  // 전환 시 이전 모터를 먼저 idle로 만든 뒤 선택 모터만 구동한다.
  writeAllIdle();
  if (activeMotor >= 0 && activeMotor < MOTOR_COUNT) {
    motors[activeMotor]->writeMicroseconds(setpointUs);
  }
}

void printMenu() {
  Serial.println();
  Serial.println("[조작 메뉴]");
  Serial.println("  1 : M1 = GPIO4 FL, expected CW");
  Serial.println("  2 : M2 = GPIO5 RR, expected CW");
  Serial.println("  3 : M3 = GPIO6 FR, expected CCW");
  Serial.println("  4 : M4 = GPIO7 RL, expected CCW");
  Serial.println("  0/s : 전 모터 정지(1000 us)");
  Serial.println("  +/- : 활성 setpoint를 10 us 조정");
  Serial.println("  r : 활성 setpoint를 1120 us로 리셋");
  Serial.println("  그 외 키 : 안전 정지 후 메뉴 재출력");
  Serial.println("  (개행 CR/LF는 무시하므로 줄 끝 설정 무관)");
}

void printStatus() {
  Serial.print("[STATE] active=");
  if (activeMotor >= 0 && activeMotor < MOTOR_COUNT) {
    Serial.print(motorNames[activeMotor]);
    Serial.print("(GPIO");
    Serial.print(motorPins[activeMotor]);
    Serial.print(" ");
    Serial.print(motorPositions[activeMotor]);
    Serial.print(", expected ");
    Serial.print(motorDirections[activeMotor]);
    Serial.print(")");
  } else {
    Serial.print("NONE(all motors idle)");
  }

  Serial.print(", setpoint=");
  Serial.print(setpointUs);
  Serial.println(" us");
}

void stopAllMotors() {
  activeMotor = -1;
  writeAllIdle();
  Serial.println("[STOP] all motors = 1000 us");
}

void selectMotor(int motorIndex) {
  activeMotor = motorIndex;
  setpointUs = DEFAULT_SETPOINT;
  applyOutputs();

  Serial.print("[SELECT] ");
  Serial.print(motorNames[motorIndex]);
  Serial.print(" = GPIO");
  Serial.print(motorPins[motorIndex]);
  Serial.print(" ");
  Serial.print(motorPositions[motorIndex]);
  Serial.print(", expected ");
  Serial.println(motorDirections[motorIndex]);
}

void adjustSetpoint(int deltaUs) {
  if (activeMotor < 0 || activeMotor >= MOTOR_COUNT) {
    writeAllIdle();
    Serial.println("[INFO] 활성 모터가 없습니다. 먼저 1~4를 선택하세요.");
    return;
  }

  setpointUs += deltaUs;
  applyOutputs();
  Serial.print("[SETPOINT] ");
  Serial.print(setpointUs);
  Serial.println(" us");
}

void resetSetpoint() {
  setpointUs = DEFAULT_SETPOINT;
  applyOutputs();
  Serial.println("[SETPOINT] reset to 1120 us");
}

void handleCommand(char command) {
  switch (command) {
    case '1':
    case '2':
    case '3':
    case '4':
      selectMotor(command - '1');
      break;

    case '0':
    case 's':
      stopAllMotors();
      break;

    case '+':
      adjustSetpoint(SETPOINT_STEP);
      break;

    case '-':
      adjustSetpoint(-SETPOINT_STEP);
      break;

    case 'r':
      resetSetpoint();
      break;

    case '\r':
    case '\n':
      // 시리얼 모니터가 붙이는 개행은 무시한다(명령 직후 안전정지 방지).
      break;

    default:
      stopAllMotors();
      Serial.print("[SAFE STOP] unknown key code: 0x");
      if (static_cast<unsigned char>(command) < 0x10) {
        Serial.print('0');
      }
      Serial.println(static_cast<unsigned char>(command), HEX);
      printMenu();
      break;
  }
}

void setup() {
  // ESC 설정(50Hz 표준 PWM). attach 직후 각 채널을 즉시 idle로 내린다.
  motor1.setPeriodHertz(50);
  motor1.attach(pin1, MIN_THROTTLE, MAX_THROTTLE);
  motor1.writeMicroseconds(MIN_THROTTLE);

  motor2.setPeriodHertz(50);
  motor2.attach(pin2, MIN_THROTTLE, MAX_THROTTLE);
  motor2.writeMicroseconds(MIN_THROTTLE);

  motor3.setPeriodHertz(50);
  motor3.attach(pin3, MIN_THROTTLE, MAX_THROTTLE);
  motor3.writeMicroseconds(MIN_THROTTLE);

  motor4.setPeriodHertz(50);
  motor4.attach(pin4, MIN_THROTTLE, MAX_THROTTLE);
  motor4.writeMicroseconds(MIN_THROTTLE);

  writeAllIdle();

  Serial.begin(115200);
  Serial.println();
  Serial.println("=========================================");
  Serial.println(">>> SINGLE MOTOR ID / DIRECTION MODE <<<");
  Serial.println("WARNING: 프로펠러 제거(props OFF)");
  Serial.println("자동 시작 없음: 명령 입력 전까지 전 모터 idle");
  Serial.println("CAP = 1250 us (25% hard limit)");
  Serial.println("=========================================");
  printMenu();
  printStatus();
}

void loop() {
  if (Serial.available() <= 0) {
    return;
  }

  const char command = static_cast<char>(Serial.read());
  handleCommand(command);
  printStatus();
}
