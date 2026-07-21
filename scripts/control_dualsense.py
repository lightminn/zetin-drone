import csv
import datetime
import socket
import threading
import time
from pathlib import Path

import pygame

from telemetry_schema import (
    CSV_FIELDS,
    active_fault_names,
    parse_telemetry_packet,
    sample_to_csv_row,
)

# === 설정 ===
UDP_IP        = "192.168.4.1"
UDP_PORT      = 4210
CTRL_LOOP_HZ  = 20          # 제어 루프 주기 (50ms)
MAX_ANGLE     = 15.0
YAW_RATE      = 1.0
TRIM_STEP     = 0.2
STOP_RETRIES  = 5            # stop/start 재전송 횟수
STOP_INTERVAL = 0.02         # 재전송 간격 (초)

# --- 고장진단 상수 ---
TELEM_TIMEOUT_SEC = 1.5
TILT_WARN_DEG     = 30.0
TILT_WARN_PERIOD  = 1.0      # 과도 기울기 경고 최소 간격 (초)

# 송신/수신 단일 소켓 사용 (드론이 송신자 포트로 텔레메트리를 응답하므로 동일 소켓을 사용해야 함)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("", UDP_PORT))
sock.settimeout(0.1)

# --- 비행 CSV 로그 (receive/monitor와 같은 스키마, logs/에 저장) ---
SCRIPT_DIR = Path(__file__).resolve().parent
LOG_DIR = SCRIPT_DIR.parent / "logs"
LOG_DIR.mkdir(parents=True, exist_ok=True)
log_path = LOG_DIR / f"flight_log_{datetime.datetime.now():%Y-%m-%d_%H%M%S}.csv"
csv_file = log_path.open("w", newline="", encoding="utf-8")
csv_writer = csv.writer(csv_file)
csv_writer.writerow(CSV_FIELDS)
print(f"[LOG] 비행 로그: {log_path}")

# === 상태 변수 ===
current_throttle = 1000
target_yaw       = 0.0
trim_roll        = 0.0
trim_pitch       = 0.0
is_armed         = False

# [FIX] RC 시퀀스 번호 — 지연 도착한 낡은 패킷을 드론 측에서 폐기할 수 있도록
rc_seq = 0

# 텔레메트리 상태
telem_lock        = threading.Lock()
last_telem_time   = 0.0
telem_angle_x     = 0.0
telem_angle_y     = 0.0
telem_throttle    = 0
fault_rc_drone    = False
fault_critical_drone = False   # Fault_Critical: IMU 상실/과도 기울기/캘리브레이션 실패
telem_total_pkts  = 0
telem_dropped_pkts = 0

# 버튼 엣지 감지용
last_btn_start = False
last_btn_R1    = False
last_btn_L1    = False
last_trig_R2   = False
last_trig_L2   = False
last_hat_state = (0, 0)


# ==========================================================
# 송신
# ==========================================================
def send_cmd(cmd: str):
    try:
        sock.sendto(cmd.encode(), (UDP_IP, UDP_PORT))
    except OSError:
        pass


def reliable_send(cmd: str):
    """중요 명령(start/stop)을 패킷 손실에 대비해 여러 번 전송."""
    for _ in range(STOP_RETRIES):
        send_cmd(cmd)
        time.sleep(STOP_INTERVAL)


def arm():
    global is_armed, current_throttle, target_yaw, rc_seq
    is_armed         = True
    current_throttle = 1100
    target_yaw       = 0.0
    rc_seq           = 0   # 재시동 시 시퀀스 번호 리셋
    reliable_send("start")
    print("\n>>> [SYSTEM] ARMED (시동 ON)")


def disarm(reason: str = "수동"):
    global is_armed, current_throttle, target_yaw
    reliable_send("stop")
    is_armed         = False
    current_throttle = 1000
    target_yaw       = 0.0
    print(f"\n>>> [SYSTEM] DISARMED ({reason})")


def deadzone(v: float, dz: float = 0.05) -> float:
    return v if abs(v) > dz else 0.0


# ==========================================================
# 텔레메트리 수신 + 고장진단 스레드
# ==========================================================
def telemetry_thread():
    global last_telem_time
    global telem_angle_x, telem_angle_y, telem_throttle
    global fault_rc_drone, fault_critical_drone
    global telem_total_pkts, telem_dropped_pkts

    print("[TELEM] 수신 스레드 시작")
    prev_fault_rc = False
    prev_fault_critical = False
    last_connect = 0.0
    last_tilt_warn = 0.0
    packet_count = 0

    while True:
        # 시동 전에도 텔레메트리를 받도록 주기적으로 목적지를 등록한다.
        now = time.monotonic()
        if now - last_connect >= 1.0:
            send_cmd("connect")
            last_connect = now

        try:
            data, _ = sock.recvfrom(512)
        except socket.timeout:
            if is_armed and last_telem_time > 0:
                elapsed = time.monotonic() - last_telem_time
                if elapsed > TELEM_TIMEOUT_SEC:
                    print(f"\n[FAULT] 텔레메트리 {elapsed:.1f}s 수신 없음 - 긴급 정지")
                    disarm("연결 끊김")
            continue
        except OSError as e:
            print(f"[TELEM ERR] 소켓 오류: {e}")
            continue

        try:
            sample = parse_telemetry_packet(data.decode("utf-8", errors="strict"))
        except (UnicodeDecodeError, ValueError):
            continue

        with telem_lock:
            telem_angle_x      = sample["Roll"]
            telem_angle_y      = sample["Pitch"]
            telem_throttle     = sample["Throttle"] or 0
            fault_rc_drone     = sample["Fault_RC"] == 1
            fault_critical_drone = sample["Fault_Critical"] == 1
            telem_total_pkts   = sample["RC_Total_Pkts"] or 0
            telem_dropped_pkts = sample["RC_Dropped_Pkts"] or 0
            last_telem_time    = time.monotonic()

        now_str = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        csv_writer.writerow(sample_to_csv_row(now_str, sample))
        packet_count += 1
        if packet_count % 20 == 0:
            csv_file.flush()

        # 드론 측 고장 플래그 — fault는 latch되므로 상승 엣지에서만 알린다.
        if fault_rc_drone and not prev_fault_rc:
            print("\n[FAULT] 드론: RC 타임아웃 감지됨")
            if is_armed:
                disarm("드론 RC 타임아웃")
        prev_fault_rc = fault_rc_drone

        if fault_critical_drone and not prev_fault_critical:
            detail = ", ".join(active_fault_names(sample)) or "원인 미상"
            print(f"\n[FAULT] 드론: 치명 고장 감지됨 ({detail})")
            if is_armed:
                disarm("드론 치명 고장")
        prev_fault_critical = fault_critical_drone

        # 패킷 드롭률 출력 (10% 초과 시 경고)
        if telem_total_pkts > 100 and telem_total_pkts % 50 == 0:
            drop_rate = telem_dropped_pkts / telem_total_pkts * 100
            if drop_rate > 10.0:
                print(f"\n[WARN] 패킷 드롭률 {drop_rate:.1f}% ({telem_dropped_pkts}/{telem_total_pkts}) - 간섭 의심")

        # 과도 기울기 경고 (1초에 한 번만)
        ax, ay = abs(telem_angle_x), abs(telem_angle_y)
        if (is_armed and (ax > TILT_WARN_DEG or ay > TILT_WARN_DEG)
                and now - last_tilt_warn >= TILT_WARN_PERIOD):
            print(f"\n[WARN] 과도 기울기 - Roll:{telem_angle_x:.1f}° Pitch:{telem_angle_y:.1f}°")
            last_tilt_warn = now


# ==========================================================
# 컨트롤러 처리 스레드
# ==========================================================
def controller_thread():
    global current_throttle, target_yaw, trim_roll, trim_pitch, rc_seq
    global last_btn_start, last_btn_R1, last_btn_L1
    global last_trig_R2, last_trig_L2, last_hat_state

    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("\n[ERR] 컨트롤러가 없습니다!")
        return

    joy = pygame.joystick.Joystick(0)
    joy.init()

    print("========== DRONE CONTROLLER ==========")
    print(f" [X]        Arm / Disarm")
    print(f" [R2/L2]    Throttle +10 / -10")
    print(f" [R1/L1]    Throttle  +1 /  -1")
    print(f" [DPAD]     Trim  |  [PS] Trim Reset")
    print(f" [Sticks]   Roll / Pitch / Yaw  (max ±{MAX_ANGLE}°)")
    print("======================================")

    loop_dt = 1.0 / CTRL_LOOP_HZ
    next_loop = time.monotonic()

    while True:
        # [FIX] sleep 누적 오차 제거: monotonic 기반 정밀 타이밍
        now = time.monotonic()
        if now < next_loop:
            time.sleep(next_loop - now)
        next_loop += loop_dt

        pygame.event.pump()

        # --- 시동 토글 ---
        btn_start = joy.get_button(0)
        if btn_start and not last_btn_start:
            if is_armed:
                disarm()
            else:
                arm()
        last_btn_start = btn_start

        if is_armed:
            # --- 스로틀 제어 ---
            curr_R1 = joy.get_button(10)
            curr_L1 = joy.get_button(9)
            curr_R2 = joy.get_axis(5) > 0.0
            curr_L2 = joy.get_axis(4) > 0.0

            delta = 0
            if   curr_R2 and not last_trig_R2: delta = +10
            elif curr_L2 and not last_trig_L2: delta = -10
            elif curr_R1 and not last_btn_R1:  delta = +1
            elif curr_L1 and not last_btn_L1:  delta = -1

            if delta:
                current_throttle = max(1000, min(1900, current_throttle + delta))
                send_cmd(f"th {current_throttle}")
                print(f" [TH] {'+' if delta > 0 else ''}{delta} -> {current_throttle}")

            last_trig_R2 = curr_R2; last_trig_L2 = curr_L2
            last_btn_R1  = curr_R1; last_btn_L1  = curr_L1

            # --- DPAD 트림 ---
            hat = joy.get_hat(0)
            if hat != last_hat_state:
                if   hat == (0,  1): trim_pitch -= TRIM_STEP
                elif hat == (0, -1): trim_pitch += TRIM_STEP
                elif hat == (-1, 0): trim_roll  -= TRIM_STEP
                elif hat == (1,  0): trim_roll  += TRIM_STEP
                if hat != (0, 0):
                    print(f" [TRIM] Roll:{trim_roll:.1f}  Pitch:{trim_pitch:.1f}")
            last_hat_state = hat

            if joy.get_button(12):
                trim_roll = trim_pitch = 0.0
                print(" [TRIM] RESET (0.0, 0.0)")

            # --- RC 명령 전송 (시퀀스 번호 포함) ---
            rc_seq += 1
            final_roll  = deadzone(joy.get_axis(0))  * MAX_ANGLE + trim_roll
            final_pitch = deadzone(-joy.get_axis(1)) * MAX_ANGLE + trim_pitch
            target_yaw += deadzone(joy.get_axis(2))  * YAW_RATE

            send_cmd(f"rc {rc_seq} {final_roll:.2f} {final_pitch:.2f} {target_yaw:.2f}")


# ==========================================================
# 메인
# ==========================================================
t_telem = threading.Thread(target=telemetry_thread, daemon=True)
t_ctrl  = threading.Thread(target=controller_thread, daemon=True)
t_telem.start()
t_ctrl.start()

try:
    while True:
        try:
            msg = input()
            if msg:
                send_cmd(msg)
        except KeyboardInterrupt:
            if is_armed:
                disarm("키보드 인터럽트")
            break
finally:
    csv_file.flush()
    csv_file.close()
    print(f"[LOG] 저장 완료: {log_path}")
