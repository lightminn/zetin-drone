/*
 * motor_id_single.ino
 *
 * WARNING: 프로펠러 제거(props OFF) 상태에서만 사용한다.
 * docs/power_on_bench_procedure.md의 Stage A에서 GPIO→물리 모터 매핑과
 * 회전 방향을 한 번에 한 모터씩 확인하기 위한 진단 펌웨어다.
 *
 * PWM 백엔드는 비행펌웨어(dual_imu_cascade_pwm)와 "완전히 동일한"
 * LEDC(ledcAttach 400Hz/14bit, duty=us*16383/2500)를 쓴다.
 * ⚠️ ESP32Servo(50Hz)는 ESP32-S3에서 LEDC 채널을 잘못 할당해 개별 GPIO가
 *    개별 모터로 매핑되지 않는다(핀이 쌍으로 묶이거나 일부가 죽는다). 그래서
 *    반드시 비행펌웨어가 실제로 쓰는 이 LEDC 경로로 진단해야 한다.
 *
 * 회전 방향표(위에서 내려다본 기준):
 *   M1 = GPIO4, FL(전-좌), expected CW
 *   M2 = GPIO5, RR(후-우), expected CW
 *   M3 = GPIO6, FR(전-우), expected CCW
 *   M4 = GPIO7, RL(후-좌), expected CCW
 */

// 비행펌웨어와 동일한 ESC LEDC 파라미터
const int pinM[4] = {4, 5, 6, 7};                 // M1 FL, M2 RR, M3 FR, M4 RL
const char *const NAMES[4] = {"M1", "M2", "M3", "M4"};
const char *const POS[4]   = {"FL", "RR", "FR", "RL"};
const char *const DIR[4]   = {"CW", "CW", "CCW", "CCW"};

const int ESC_FREQ    = 400;
const int ESC_RES     = 14;
const int ESC_PERIOD  = 2500;                     // us
const int ESC_MAXDUTY = (1 << ESC_RES) - 1;       // 16383

const int MIN_THROTTLE = 1000;
const int DEFAULT_SETPOINT = 1120;
const int CAP = 1250;                             // 25% 하드 상한
const int SETPOINT_STEP = 10;

int activeMotor = -1;
int setpointUs = DEFAULT_SETPOINT;

// 비행펌웨어 writeMotor 와 동일: us를 [1000,2000]로 제한 후 duty 변환.
void writeMotorUs(int pin, int us) {
  if (us < 1000) us = 1000;
  else if (us > 2000) us = 2000;
  uint32_t duty = (uint32_t)us * ESC_MAXDUTY / ESC_PERIOD;
  ledcWrite(pin, duty);
}

void writeAllIdle() {
  for (int i = 0; i < 4; i++) writeMotorUs(pinM[i], MIN_THROTTLE);
}

void applyOutputs() {
  if (setpointUs < MIN_THROTTLE) setpointUs = MIN_THROTTLE;
  else if (setpointUs > CAP) setpointUs = CAP;
  writeAllIdle();
  if (activeMotor >= 0 && activeMotor < 4) writeMotorUs(pinM[activeMotor], setpointUs);
}

void printMenu() {
  Serial.println();
  Serial.println("[조작 메뉴 — LEDC(비행펌웨어와 동일 PWM)]");
  Serial.println("  1 : M1 = GPIO4 FL, expected CW");
  Serial.println("  2 : M2 = GPIO5 RR, expected CW");
  Serial.println("  3 : M3 = GPIO6 FR, expected CCW");
  Serial.println("  4 : M4 = GPIO7 RL, expected CCW");
  Serial.println("  0/s : 전 모터 정지 / +,- : setpoint / r : 1120 리셋");
  Serial.println("  (개행 CR/LF 무시)");
}

void printStatus() {
  Serial.print("[STATE] active=");
  if (activeMotor >= 0 && activeMotor < 4) {
    Serial.print(NAMES[activeMotor]); Serial.print("(GPIO");
    Serial.print(pinM[activeMotor]); Serial.print(" ");
    Serial.print(POS[activeMotor]); Serial.print(", expected ");
    Serial.print(DIR[activeMotor]); Serial.print(")");
  } else {
    Serial.print("NONE(all idle)");
  }
  Serial.print(", setpoint="); Serial.print(setpointUs); Serial.println(" us");
}

void stopAll() { activeMotor = -1; writeAllIdle(); Serial.println("[STOP] all = 1000 us"); }

void selectMotor(int i) {
  activeMotor = i; setpointUs = DEFAULT_SETPOINT; applyOutputs();
  Serial.print("[SELECT] "); Serial.print(NAMES[i]); Serial.print(" = GPIO");
  Serial.print(pinM[i]); Serial.print(" "); Serial.print(POS[i]);
  Serial.print(", expected "); Serial.println(DIR[i]);
}

void adjust(int d) {
  if (activeMotor < 0) { writeAllIdle(); Serial.println("[INFO] 먼저 1~4 선택"); return; }
  setpointUs += d; applyOutputs();
  Serial.print("[SETPOINT] "); Serial.print(setpointUs); Serial.println(" us");
}

void handle(char c) {
  switch (c) {
    case '1': case '2': case '3': case '4': selectMotor(c - '1'); break;
    case '0': case 's': stopAll(); break;
    case '+': adjust(SETPOINT_STEP); break;
    case '-': adjust(-SETPOINT_STEP); break;
    case 'r': setpointUs = DEFAULT_SETPOINT; applyOutputs();
              Serial.println("[SETPOINT] reset 1120 us"); break;
    case '\r': case '\n': break;                 // 개행 무시
    default:
      stopAll();
      Serial.print("[SAFE STOP] unknown key 0x");
      if ((unsigned char)c < 0x10) Serial.print('0');
      Serial.println((unsigned char)c, HEX);
      printMenu();
      break;
  }
}

void setup() {
  bool ok = true;
  for (int i = 0; i < 4; i++) ok &= ledcAttach(pinM[i], ESC_FREQ, ESC_RES);
  writeAllIdle();
  Serial.begin(115200);
  Serial.println();
  Serial.println("=========================================");
  Serial.println(">>> SINGLE MOTOR ID (LEDC = flight PWM) <<<");
  Serial.println("WARNING: 프로펠러 제거(props OFF)");
  Serial.print("ledcAttach 4 pins: "); Serial.println(ok ? "OK" : "FAIL(핀 attach 실패)");
  Serial.println("CAP = 1250 us, 자동시작 없음");
  Serial.println("=========================================");
  printMenu();
  printStatus();
}

void loop() {
  if (Serial.available() <= 0) return;
  handle((char)Serial.read());
  printStatus();
}
