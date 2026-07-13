# Repository Cleanup Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use
> `superpowers:subagent-driven-development` (recommended) or
> `superpowers:executing-plans` to implement this plan task-by-task. Steps use
> checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the supported ESP32-S3 + ICM42670 + PWM path obvious, preserve
all other firmware and tools under a clearly unsupported archive, and ensure
every maintained document points to an existing path.

**Architecture:** Promote the current cascade controller and hardware
diagnostics into `firmware/flight/` and `firmware/diagnostics/`. Move every
other sketch and the stale PlatformIO project into `firmware/archive/` without
repairing them. Rename current Python tools in place, archive legacy network
tools, rebuild documentation around a migration map, and enforce the result
with a local repository-layout validator.

**Tech Stack:** Git, Arduino CLI, ESP32-S3 Arduino core, ICM42670P,
ESP32Servo/LEDC PWM, Python 3 standard library, pandas, matplotlib, pygame.

**Design:**
[`docs/design/2026-07-13-repository-cleanup-design.md`](../design/2026-07-13-repository-cleanup-design.md)

## Global Constraints

- Supported stack: ESP32-S3 + ICM42670 + PWM only.
- `dual_imu_cascade_pwm` is a current flight-controller candidate, not a claim
  of stable flight or hover.
- Preserve validated pin assignments, motor mixer signs, body-axis mapping,
  UDP command grammar, and telemetry field order.
- Preserve compatibility with 10-, 14-, and 21-field telemetry packets.
- Do not repair, modernize, compile, or claim support for anything under
  `firmware/archive/` or `scripts/archive/`.
- Use `git mv` for tracked moves so history remains recoverable.
- Arduino sketch directory names must exactly match their `.ino` basenames.
- Current source and maintained docs use `lower_snake_case` paths.
- Do not commit `.codex`, `.claude`, `.gemini`, `.vscode`, `AGENTS.md`, Python
  caches, editor swap files, or generated CSV logs.
- Do not push intermediate commits. Push only after the completed cleanup has
  been reviewed and the user explicitly requests it.

---

### Task 1: Exclude local agent, editor, and generated files

**Files:**

- Modify: `.gitignore`
- Delete from Git: `.gemini/settings.json`
- Delete from Git: `.vscode/extensions.json`
- Delete from Git: `.vscode/settings.json`

**Interfaces:**

- Consumes: the current repository root.
- Produces: Git ignores local agent/editor state while retaining project files.

- [ ] **Step 1: Run the ignore assertion before changing `.gitignore`**

Run:

```bash
for candidate in \
  .codex/probe \
  .claude/probe \
  .gemini/settings.json \
  .vscode/settings.json \
  AGENTS.md \
  firmware/examples/AGENTS.md \
  scripts/__pycache__/probe.pyc \
  scratch.swp \
  scratch.kate-swp; do
  git check-ignore --no-index -q "$candidate" || {
    echo "NOT_IGNORED $candidate"
    exit 1
  }
done
```

Expected: FAIL on at least `.codex/probe`; this proves the current ignore file
does not meet the cleanup requirement.

- [ ] **Step 2: Extend `.gitignore` with exact local/generated patterns**

Append this block using `apply_patch`:

```gitignore

# Local AI agents and project instructions
.codex/
.claude/
.gemini/
AGENTS.md

# Machine-specific editor state
.vscode/
*.swp
*.kate-swp

# Python generated files
__pycache__/
*.py[cod]
```

Keep the existing `.pio` and `*.csv` rules.

- [ ] **Step 3: Remove tracked local configuration**

```bash
git rm -r .gemini .vscode
```

Expected: `.gemini/settings.json`, `.vscode/extensions.json`, and
`.vscode/settings.json` are staged for deletion.

- [ ] **Step 4: Re-run the Step 1 ignore assertion**

Expected: exit 0 with no `NOT_IGNORED` output. Then run:

```bash
git status --short --ignored | rg '!! (\.codex/|AGENTS\.md|firmware/examples/\.codex/|firmware/examples/AGENTS\.md)'
```

Expected: if those local paths exist in the checkout, they appear only with
`!!`, never `??`. No output is also valid in an isolated worktree where the
local-only paths are absent; the hypothetical-path assertion above remains the
authoritative check.

- [ ] **Step 5: Commit the local-state cleanup**

```bash
git add .gitignore
git commit -m "chore: ignore local agent and editor files"
```

### Task 2: Separate current firmware from the archive

**Files:**

- Move: all 23 directories currently under `firmware/examples/`
- Move: `firmware/src/`, `firmware/lib/`, `firmware/include/`
- Move: `firmware/platformio_config/platformio.ini`
- Move: `test/README`
- Modify: `firmware/README.md`
- Create: `firmware/archive/README.md`
- Create: `firmware/archive/dshot/README.md`
- Create: `firmware/archive/platformio_skeleton/README.md`

**Interfaces:**

- Produces current flight sketch:
  `firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino`.
- Produces five diagnostic sketch roots under `firmware/diagnostics/`.
- Produces unsupported historical material only under `firmware/archive/`.
- Later tasks rely on these exact destinations for source comments and docs.

- [ ] **Step 1: Record the pre-migration firmware inventory**

```bash
git ls-files 'firmware/examples/**/*.ino' | sort > /tmp/zetin-firmware-before.txt
test "$(wc -l < /tmp/zetin-firmware-before.txt)" -eq 23
```

Expected: exit 0 and 23 sketch paths.

- [ ] **Step 2: Create lifecycle/category parents**

```bash
mkdir -p \
  firmware/flight \
  firmware/diagnostics \
  firmware/archive/legacy_flight \
  firmware/archive/filter_experiments \
  firmware/archive/dshot \
  firmware/archive/other_sensors \
  firmware/archive/other_mcus \
  firmware/archive/platformio_skeleton/examples \
  firmware/archive/platformio_skeleton/test
```

- [ ] **Step 3: Move and rename the six current sketches**

```bash
git mv firmware/examples/DUAL_IMU_CASCADE firmware/flight/dual_imu_cascade_pwm
git mv firmware/flight/dual_imu_cascade_pwm/DUAL_IMU_CASCADE.ino \
  firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino

git mv firmware/examples/PWM_TEST firmware/diagnostics/motor_pwm_bench
git mv firmware/diagnostics/motor_pwm_bench/PWM_TEST.ino \
  firmware/diagnostics/motor_pwm_bench/motor_pwm_bench.ino

git mv firmware/examples/IMU_TEST_RAW firmware/diagnostics/icm42670_single_raw
git mv firmware/diagnostics/icm42670_single_raw/IMU_TEST_RAW.ino \
  firmware/diagnostics/icm42670_single_raw/icm42670_single_raw.ino

git mv firmware/examples/DUAL_IMU_RAW_TEST firmware/diagnostics/icm42670_dual_raw
git mv firmware/diagnostics/icm42670_dual_raw/DUAL_IMU_RAW_TEST.ino \
  firmware/diagnostics/icm42670_dual_raw/icm42670_dual_raw.ino

git mv firmware/examples/DUAL_IMU_PID_DEBUG firmware/diagnostics/icm42670_dual_loop_debug
git mv firmware/diagnostics/icm42670_dual_loop_debug/DUAL_IMU_PID_DEBUG.ino \
  firmware/diagnostics/icm42670_dual_loop_debug/icm42670_dual_loop_debug.ino

git mv firmware/examples/PCB_1.5.2_TEST firmware/diagnostics/board_v1_5_2_dual_imu
git mv firmware/diagnostics/board_v1_5_2_dual_imu/PCB_1.5.2_TEST.ino \
  firmware/diagnostics/board_v1_5_2_dual_imu/board_v1_5_2_dual_imu.ino
```

- [ ] **Step 4: Archive the superseded flight/filter sketches**

```bash
git mv firmware/examples/PWM_TEST_IMU_PID \
  firmware/archive/legacy_flight/single_imu_pid_pwm
git mv firmware/archive/legacy_flight/single_imu_pid_pwm/PWM_TEST_IMU_PID.ino \
  firmware/archive/legacy_flight/single_imu_pid_pwm/single_imu_pid_pwm.ino

git mv firmware/examples/PWM_TEST_DUAL_IMU_PID \
  firmware/archive/legacy_flight/dual_imu_pid_pwm
git mv firmware/archive/legacy_flight/dual_imu_pid_pwm/PWM_TEST_DUAL_IMU_PID.ino \
  firmware/archive/legacy_flight/dual_imu_pid_pwm/dual_imu_pid_pwm.ino

git mv firmware/examples/DUAL_LOOP_TEST \
  firmware/archive/legacy_flight/single_imu_cascade_pwm
git mv firmware/archive/legacy_flight/single_imu_cascade_pwm/DUAL_LOOP_TEST.ino \
  firmware/archive/legacy_flight/single_imu_cascade_pwm/single_imu_cascade_pwm.ino

git mv firmware/examples/KALMAN_FLIGHT_TEST \
  firmware/archive/legacy_flight/single_imu_kalman_pid_pwm
git mv firmware/archive/legacy_flight/single_imu_kalman_pid_pwm/KALMAN_FLIGHT_TEST.ino \
  firmware/archive/legacy_flight/single_imu_kalman_pid_pwm/single_imu_kalman_pid_pwm.ino

git mv firmware/examples/KALMAN_TEST \
  firmware/archive/filter_experiments/icm42670_kalman_attitude
git mv firmware/archive/filter_experiments/icm42670_kalman_attitude/KALMAN_TEST.ino \
  firmware/archive/filter_experiments/icm42670_kalman_attitude/icm42670_kalman_attitude.ino
```

- [ ] **Step 5: Archive DShot and motor-allocation sketches**

```bash
git mv firmware/examples/DSHOT_TEST \
  firmware/archive/dshot/single_motor_fixed_throttle
git mv firmware/archive/dshot/single_motor_fixed_throttle/DSHOT_TEST.ino \
  firmware/archive/dshot/single_motor_fixed_throttle/single_motor_fixed_throttle.ino

git mv firmware/examples/DSHOT_TEST_PERCENT \
  firmware/archive/dshot/single_motor_serial_control
git mv firmware/archive/dshot/single_motor_serial_control/DSHOT_TEST_PERCENT.ino \
  firmware/archive/dshot/single_motor_serial_control/single_motor_serial_control.ino

git mv firmware/examples/DSHOT_TEST_IMU \
  firmware/archive/dshot/mpu6500_tilt_throttle
git mv firmware/archive/dshot/mpu6500_tilt_throttle/DSHOT_TEST_IMU.ino \
  firmware/archive/dshot/mpu6500_tilt_throttle/mpu6500_tilt_throttle.ino

git mv firmware/examples/DSHOT_TEST_IMU_PID \
  firmware/archive/dshot/mpu6500_pid_bench
git mv firmware/archive/dshot/mpu6500_pid_bench/DSHOT_TEST_IMU_PID.ino \
  firmware/archive/dshot/mpu6500_pid_bench/mpu6500_pid_bench.ino

git mv firmware/examples/RMT_TEST \
  firmware/archive/dshot/four_motor_full_throttle_unsafe
git mv firmware/archive/dshot/four_motor_full_throttle_unsafe/RMT_TEST.ino \
  firmware/archive/dshot/four_motor_full_throttle_unsafe/four_motor_full_throttle_unsafe.ino

git mv firmware/examples/ZETIN_Drone_Standalone_Test \
  firmware/archive/dshot/motor_allocator_dual_core
git mv firmware/archive/dshot/motor_allocator_dual_core/ZETIN_Drone_Standalone_Test.ino \
  firmware/archive/dshot/motor_allocator_dual_core/motor_allocator_dual_core.ino

git mv firmware/examples/MOTOR_ALGO_TEST \
  firmware/archive/platformio_skeleton/examples/motor_allocator_dshot
git mv firmware/archive/platformio_skeleton/examples/motor_allocator_dshot/MOTOR_ALGO_TEST.ino \
  firmware/archive/platformio_skeleton/examples/motor_allocator_dshot/motor_allocator_dshot.ino
```

- [ ] **Step 6: Archive sensor and alternate-MCU sketches**

```bash
git mv firmware/examples/BMM_TEST \
  firmware/archive/other_sensors/bmm350_read
git mv firmware/archive/other_sensors/bmm350_read/BMM_TEST.ino \
  firmware/archive/other_sensors/bmm350_read/bmm350_read.ino

git mv firmware/examples/PCB_1.5.2_BMM_TEST \
  firmware/archive/other_sensors/board_v1_5_2_bmm350
git mv firmware/archive/other_sensors/board_v1_5_2_bmm350/PCB_1.5.2_BMM_TEST.ino \
  firmware/archive/other_sensors/board_v1_5_2_bmm350/board_v1_5_2_bmm350.ino

git mv firmware/examples/GPS_TEST_RAW \
  firmware/archive/other_sensors/gps_uart_passthrough
git mv firmware/archive/other_sensors/gps_uart_passthrough/GPS_TEST_RAW.ino \
  firmware/archive/other_sensors/gps_uart_passthrough/gps_uart_passthrough.ino

git mv firmware/examples/US100_TEST \
  firmware/archive/other_sensors/us100_distance
git mv firmware/archive/other_sensors/us100_distance/US100_TEST.ino \
  firmware/archive/other_sensors/us100_distance/us100_distance.ino

git mv firmware/examples/F411_test \
  firmware/archive/other_mcus/stm32_f411_uart_rx
git mv firmware/archive/other_mcus/stm32_f411_uart_rx/F411_test.ino \
  firmware/archive/other_mcus/stm32_f411_uart_rx/stm32_f411_uart_rx.ino
```

Do not remove the local `firmware/examples/` directory if it still contains
ignored `.codex/` or `AGENTS.md` files. Git will record no tracked
`firmware/examples/` content after the moves.

- [ ] **Step 7: Move the stale PlatformIO skeleton as one historical unit**

```bash
git mv firmware/src firmware/archive/platformio_skeleton/src
git mv firmware/lib firmware/archive/platformio_skeleton/lib
git mv firmware/include firmware/archive/platformio_skeleton/include
git mv firmware/platformio_config/platformio.ini \
  firmware/archive/platformio_skeleton/platformio.ini
rmdir firmware/platformio_config
git mv test/README firmware/archive/platformio_skeleton/test/README
```

Do not edit its source, includes, or dependency list.

- [ ] **Step 8: Write archive warnings and replace `firmware/README.md`**

Create `firmware/archive/README.md`:

```markdown
# Archived firmware

Everything below this directory is preserved for history only. It is not part
of the supported ESP32-S3 + ICM42670 + PWM build path, is not compile-verified,
and may contain missing dependencies, obsolete pinouts, or unsafe behavior.

Do not flash archived motor-control code without reviewing it completely and
removing propellers. Current firmware is linked from [`../README.md`](../README.md).
```

Create `firmware/archive/dshot/README.md`:

```markdown
# Archived DShot experiments

These sketches depend on old or absent DShot/MPU6500 libraries. They are not
maintained. `four_motor_full_throttle_unsafe` commands all four motors near
full output and must never be treated as a normal bench test.
```

Create `firmware/archive/platformio_skeleton/README.md`:

```markdown
# Archived PlatformIO skeleton

This project contains simulated sensor data, a canceled motor allocator, an
old TCP transport, and stale dependencies. It is retained only for history.
Supported firmware is built with Arduino CLI from `firmware/flight/` and
`firmware/diagnostics/`.
```

Replace `firmware/README.md` with a current-only guide containing links to the
one flight and five diagnostic directories, the `/tmp` Arduino CLI build
pattern, the archive warning, and the existing propeller/polarity safety rules.
List the actual SPI/CS pinout for each diagnostic, especially the single-IMU
raw sketch whose pins differ from the PCB v1.5.2 diagnostic.

- [ ] **Step 9: Verify structure and compile only current firmware**

```bash
test "$(find firmware -type f -name '*.ino' | wc -l)" -eq 23

find firmware -type f -name '*.ino' -print0 | while IFS= read -r -d '' ino; do
  test "$(basename "$(dirname "$ino")")" = "$(basename "$ino" .ino)" || {
    echo "SKETCH_NAME_MISMATCH $ino"
    exit 1
  }
done

for sketch in \
  firmware/flight/dual_imu_cascade_pwm \
  firmware/diagnostics/motor_pwm_bench \
  firmware/diagnostics/icm42670_single_raw \
  firmware/diagnostics/icm42670_dual_raw \
  firmware/diagnostics/icm42670_dual_loop_debug \
  firmware/diagnostics/board_v1_5_2_dual_imu; do
  name=$(basename "$sketch")
  arduino-cli compile --warnings all \
    --fqbn esp32:esp32:esp32s3 \
    --build-path "/tmp/zetin-cleanup-$name" \
    "$sketch" || exit 1
done
```

Expected: 23 name checks and six current ESP32-S3 builds pass. Third-party
library warnings may remain; source compile errors may not. Do not compile
anything under `firmware/archive/`.

- [ ] **Step 10: Commit the firmware lifecycle split**

```bash
git add firmware
git commit -m "refactor: separate current and archived firmware"
```

---

### Task 3: Rename current Python tools and archive legacy network scripts

**Files:**

- Rename: eight current files under `scripts/`
- Move: `scripts/GPS_Reciever.py`
- Move: `test/tcp_test.py`
- Remove after folding content: `test/README.md`
- Create: `scripts/archive/README.md`
- Modify imports in: `scripts/receive_telemetry.py`,
  `scripts/monitor_telemetry.py`
- Modify paired-path docstring in: `scripts/receive_dual_imu_debug.py`
- Modify current flight firmware comments naming ground tools
- Replace: `scripts/README.md`

**Interfaces:**

- Produces module `telemetry_schema` exporting `TELEMETRY_FIELDS`,
  `CSV_FIELDS`, `parse_telemetry_packet`, `sample_to_csv_row`, and
  `active_fault_names` unchanged.
- Produces the executable paths consumed by all maintained docs.

- [ ] **Step 1: Prove the new names do not exist yet**

```bash
test ! -e scripts/control_dualsense.py
test ! -e scripts/telemetry_schema.py
```

Expected: exit 0.

- [ ] **Step 2: Rename current scripts and archive legacy scripts**

```bash
git mv scripts/Drone_Control_Dualsense.py scripts/control_dualsense.py
git mv scripts/Drone_Tuning.py scripts/tune_pid.py
git mv scripts/Drone_Reciever.py scripts/receive_telemetry.py
git mv scripts/Drone_Monitor.py scripts/monitor_telemetry.py
git mv scripts/Drone_Analasys.py scripts/analyze_flight_log.py
git mv scripts/drone_telemetry.py scripts/telemetry_schema.py
git mv scripts/dual_imu_pid_debug_receiver.py scripts/receive_dual_imu_debug.py
git mv scripts/Controller_test.py scripts/test_dualsense_input.py

mkdir -p scripts/archive
git mv scripts/GPS_Reciever.py scripts/archive/receive_gps_udp_legacy.py
git mv test/tcp_test.py scripts/archive/test_tcp_legacy.py
```

- [ ] **Step 3: Update imports and paired paths using `apply_patch`**

In both `scripts/receive_telemetry.py` and
`scripts/monitor_telemetry.py`, replace:

```python
from drone_telemetry import (
```

with:

```python
from telemetry_schema import (
```

In `scripts/receive_dual_imu_debug.py`, replace the paired firmware line with:

```python
Pairs with
firmware/diagnostics/icm42670_dual_loop_debug/icm42670_dual_loop_debug.ino.
```

In `firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino`, replace the
two ground-tool comments with:

```cpp
// scripts/control_dualsense.py 프로토콜 호환용
// scripts/tune_pid.py 명령 호환: P/I/D는 inner rate PID에 대응
```

Do not change socket constants, packet formatting, field order, gains, pinout,
motor mixer, or controller behavior.

- [ ] **Step 4: Create the script archive warning and remove `test/`**

Create `scripts/archive/README.md`:

```markdown
# Archived PC tools

These scripts target deprecated GPS/TCP experiments and are retained only for
history. They are not part of the current UDP telemetry/control workflow and
are not included in current Python verification.
```

Fold any unique factual TCP description from `test/README.md` into the archive
README, then run:

```bash
git rm test/README.md
rmdir test
```

- [ ] **Step 5: Replace `scripts/README.md` with exact current commands**

List these commands from repository root:

```bash
python scripts/control_dualsense.py
python scripts/tune_pid.py
python scripts/receive_telemetry.py
python scripts/monitor_telemetry.py
python scripts/analyze_flight_log.py [optional-log.csv]
python scripts/receive_dual_imu_debug.py
python scripts/test_dualsense_input.py
```

State that the receiver and monitor both write to `logs/`, use UDP 4210, and
share `telemetry_schema.py`. Link legacy scripts only through
`archive/README.md`.

- [ ] **Step 6: Verify imports and syntax**

```bash
/home/light/anaconda3/bin/python -m py_compile scripts/*.py
PYTHONPATH=scripts /home/light/anaconda3/bin/python -c \
  'from telemetry_schema import CSV_FIELDS, parse_telemetry_packet; assert len(CSV_FIELDS) == 22; assert parse_telemetry_packet(",".join(["0"] * 10))["Throttle"] == 0'
```

Expected: both commands exit 0. Do not include `scripts/archive/*.py` in the
current support check.

- [ ] **Step 7: Commit the script rename**

```bash
git add scripts firmware/flight
git commit -m "refactor: rename drone utility scripts"
```

---

### Task 4: Rebuild maintained documentation and migration history

**Files:**

- Modify: `README.md`
- Move: `docs/ONBOARDING.md` → `docs/project_overview.md`
- Replace: `docs/README.md`
- Create: `docs/firmware_catalog.md`
- Create: `docs/udp_protocol.md`
- Create: `docs/migration_map.md`
- Replace: `logs/README.md`
- Move two `docs/superpowers/` files to `docs/history/`
- Archive: `docs/presentation.c`, `docs/commit_message.txt`
- Modify historical document banners and exact paths

**Interfaces:**

- Produces the maintained navigation surface validated by Task 5.
- Produces 44 migration rows consumed by
  `check_repo_layout.py::check_migration_map`.

- [ ] **Step 1: Move documents into maintained, historical, and archive paths**

```bash
mkdir -p docs/history docs/archive
git mv docs/ONBOARDING.md docs/project_overview.md
git mv docs/superpowers/specs/2026-05-14-dual-imu-pid-design.md \
  docs/history/2026-05-14-dual-imu-pid-design.md
git mv docs/superpowers/plans/2026-05-14-dual-imu-pid.md \
  docs/history/2026-05-14-dual-imu-pid-implementation-plan.md
rmdir docs/superpowers/specs docs/superpowers/plans docs/superpowers
git mv docs/presentation.c docs/archive/pid_dshot_presentation_snippet.c
git mv docs/commit_message.txt docs/archive/commit_message_snapshot.txt
```

- [ ] **Step 2: Replace the root README with a current-only entry point**

Use this exact structure and link set:

```markdown
# ZETIN Drone

ESP32-S3, dual ICM42670 IMUs, and PWM ESC control are the current development
stack. Motor PWM and raw IMU acquisition are bench-verified; closed-loop
stabilization remains experimental.

## Start here

- [Project overview](docs/project_overview.md)
- [Current flight-controller candidate](firmware/flight/dual_imu_cascade_pwm/)
- [Firmware and diagnostic guide](firmware/README.md)
- [PC control and telemetry tools](scripts/README.md)
- [UDP protocol and telemetry schema](docs/udp_protocol.md)
- [Firmware lifecycle catalog](docs/firmware_catalog.md)

## Quick verification

```bash
arduino-cli compile --warnings all --fqbn esp32:esp32:esp32s3 \
  --build-path /tmp/zetin-flight-build \
  firmware/flight/dual_imu_cascade_pwm

/home/light/anaconda3/bin/python -m py_compile scripts/*.py
/home/light/anaconda3/bin/python tools/check_repo_layout.py
```

## Safety

Remove propellers for bench tests. Verify power polarity, pin assignments,
motor order, and correction direction before any restrained flight test.
Archived experiments are unsupported and may be unsafe.
```

- [ ] **Step 3: Rewrite `docs/README.md` as the document index**

```markdown
# Documentation

## Maintained
- [Project overview](project_overview.md)
- [Firmware catalog](firmware_catalog.md)
- [UDP protocol](udp_protocol.md)
- [Repository migration map](migration_map.md)
- [Repository cleanup design](design/2026-07-13-repository-cleanup-design.md)
- [Repository cleanup implementation plan](plans/2026-07-13-repository-cleanup.md)

## Historical
- [2026-05-14 dual-IMU PID design](history/2026-05-14-dual-imu-pid-design.md)
- [2026-05-14 dual-IMU PID implementation plan](history/2026-05-14-dual-imu-pid-implementation-plan.md)

## Archived
- [PID/DShot presentation snippet](archive/pid_dshot_presentation_snippet.c)
- [Old commit-message snapshot](archive/commit_message_snapshot.txt)
```

- [ ] **Step 4: Update `docs/project_overview.md` facts and links**

Apply all changes in one `apply_patch` review:

1. Replace the old `firmware/examples/` map with `flight/`, `diagnostics/`,
   and `archive/`.
2. Point the representative controller to
   `firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino`.
3. Replace every Python tool name with the Task 3 name.
4. Describe 21 UDP fields and 22 CSV columns including Timestamp; preserve the
   explicit 10/14-field compatibility note.
5. Replace the old `src/` warning with a link to
   `firmware/archive/platformio_skeleton/README.md`.
6. Place BMM350, GPS, US100, DShot, Kalman, STM32, TCP, and old PID variants in
   an explicit deprecated/archive section.
7. Keep the maturity boundary: PWM and raw IMU are bench-verified; stable
   closed-loop flight is not established.
8. Keep current pin, mixer, and body-axis facts unchanged.
9. Convert every navigational code-span path into a relative Markdown link.

Use this current tool table exactly:

```markdown
| Tool | Role |
|---|---|
| [`control_dualsense.py`](../scripts/control_dualsense.py) | DualSense command sender |
| [`tune_pid.py`](../scripts/tune_pid.py) | Manual UDP command and gain tuning |
| [`receive_telemetry.py`](../scripts/receive_telemetry.py) | Terminal receiver and CSV logger |
| [`monitor_telemetry.py`](../scripts/monitor_telemetry.py) | Live plots and CSV logger |
| [`analyze_flight_log.py`](../scripts/analyze_flight_log.py) | Offline CSV analysis |
| [`receive_dual_imu_debug.py`](../scripts/receive_dual_imu_debug.py) | Paired loop diagnostic receiver |
```

- [ ] **Step 5: Create the authoritative UDP protocol document**

`docs/udp_protocol.md` must define:

```text
Transport: UDP
Drone address: 192.168.4.1
Port: 4210
Registration: any incoming packet identifies the ground-station endpoint;
              current receivers send "connect" periodically.
```

Document these commands exactly:

```text
start
stop
rc <seq> <roll> <pitch> <yaw>
th <microseconds>
yaw <0|1>
pa|ia|da <value>
pr|ir|dr <value>
pp|ip|dp <value>
py|iy|dy <value>
```

Document the 21 telemetry fields in this exact order:

```text
Roll, Pitch, Yaw,
Gyro_X, Gyro_Y, Gyro_Z,
Accel_X, Accel_Y, Accel_Z,
Throttle,
Fault_RC, Fault_Critical,
RC_Total_Pkts, RC_Dropped_Pkts,
Fault_IMU1, Fault_IMU2, Fault_Disagree,
Active_IMUs, Mixer_Scaled, Fault_Attitude, Calibration_OK
```

State that 10-field packets end at `Throttle`, 14-field packets end at
`RC_Dropped_Pkts`, unknown legacy values become blank CSV cells, and Timestamp
is added only by ground tools.

- [ ] **Step 6: Create the firmware catalog and complete migration map**

`docs/firmware_catalog.md` must contain exactly 23 rows with these columns:

```text
Lifecycle | Target MCU | Sensor | Motor protocol | Safety/build note | Link
```

Use the six current destinations from Task 2 Step 3 as `current` and the 17
destinations from Task 2 Steps 4-6 as `archived`. Mark
`four_motor_full_throttle_unsafe` explicitly unsafe. State that archived
sketches are not repaired or build-verified.

`docs/migration_map.md` must use `Old path` and `New path` as its first two
table columns and include exactly:

- 23 sketch moves from Task 2 Steps 3-6;
- five PlatformIO/template moves from Task 2 Step 7;
- 10 Python/test moves from Task 3 Step 2;
- five document moves from Task 4 Step 1 plus the folded
  `test/README.md` → `scripts/archive/README.md` relationship;

Expected total: 44 unique old and 44 unique new paths. List the removed
`.gemini`/`.vscode` files separately without `Old path`/`New path` table syntax.

- [ ] **Step 7: Correct logs and historical docs**

Replace `logs/README.md` with the actual
`flight_log_YYYY-MM-DD_HHMMSS.csv` convention, links to the two writers and
analyzer, Timestamp plus the exact 21 Step 5 fields, and the 10/14-field legacy
behavior. Remove claims of battery, motor-output, and PID-output columns.

At the top of both `docs/history/` files, insert:

```markdown
> Historical document: this describes the superseded dual-IMU PID iteration.
> Current flight-controller candidate:
> [`dual_imu_cascade_pwm`](../../firmware/flight/dual_imu_cascade_pwm/).
> Archived result:
> [`dual_imu_pid_pwm`](../../firmware/archive/legacy_flight/dual_imu_pid_pwm/).
```

Replace all occurrences inside those historical files:

```text
firmware/examples/PWM_TEST_DUAL_IMU_PID/PWM_TEST_DUAL_IMU_PID.ino
→ firmware/archive/legacy_flight/dual_imu_pid_pwm/dual_imu_pid_pwm.ino

firmware/examples/PWM_TEST_IMU_PID/PWM_TEST_IMU_PID.ino
→ firmware/archive/legacy_flight/single_imu_pid_pwm/single_imu_pid_pwm.ino

Drone_Tuning.py → scripts/tune_pid.py
Drone_Control_Dualsense.py → scripts/control_dualsense.py
```

Label historical compile commands unsupported and do not run them.

- [ ] **Step 8: Run a pre-validator stale-reference audit**

```bash
rg -n \
  'firmware/examples/|firmware/src/|firmware/platformio_config/|Drone_(Reciever|Analasys|Monitor|Tuning|Control_Dualsense)\.py|Controller_test\.py|GPS_Reciever\.py|ZETIN_Drone' \
  README.md docs/project_overview.md docs/README.md docs/firmware_catalog.md \
  docs/udp_protocol.md firmware/README.md scripts/README.md logs/README.md \
  firmware/flight firmware/diagnostics scripts \
  --glob '!scripts/archive/**'
```

Expected: no output. Old names remain only in `docs/migration_map.md`,
`docs/design/`, `docs/plans/`, `docs/history/`, and archived source.

- [ ] **Step 9: Commit the documentation migration**

```bash
git add README.md docs firmware/README.md scripts/README.md logs/README.md
git commit -m "docs: align project documentation with repository layout"
```

---

### Task 5: Add deterministic repository-layout and telemetry checks

**Files:**

- Create: `tools/test_repo_layout.py`
- Create: `tools/check_repo_layout.py`
- Create: `tools/test_telemetry_schema.py`

**Interfaces:**

- `check_markdown_links(repo: Path, files: Iterable[Path]) -> list[str]`
- `check_sketch_names(repo: Path) -> list[str]`
- `check_migration_map(repo: Path, map_path: Path) -> list[str]`
- `check_stale_tokens(repo: Path, files: Iterable[Path]) -> list[str]`
- `main() -> int`

- [ ] **Step 1: Write failing tests for the repository checker**

Create `tools/test_repo_layout.py`:

```python
import tempfile
import unittest
from pathlib import Path

from tools.check_repo_layout import (
    check_markdown_links,
    check_migration_map,
    check_sketch_names,
    check_stale_tokens,
)


class RepoLayoutChecksTest(unittest.TestCase):
    def setUp(self):
        self.tempdir = tempfile.TemporaryDirectory()
        self.repo = Path(self.tempdir.name)

    def tearDown(self):
        self.tempdir.cleanup()

    def write(self, relative, content=""):
        path = self.repo / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding="utf-8")
        return path

    def test_markdown_links_report_only_missing_local_target(self):
        self.write("docs/good.md", "ok")
        readme = self.write(
            "README.md",
            "[good](docs/good.md) [bad](docs/missing.md) "
            "[web](https://example.com)",
        )
        errors = check_markdown_links(self.repo, [readme])
        self.assertEqual(1, len(errors))
        self.assertIn("docs/missing.md", errors[0])

    def test_sketch_directory_must_match_ino_basename(self):
        self.write("firmware/flight/right/right.ino")
        self.write("firmware/flight/wrong/not_wrong.ino")
        errors = check_sketch_names(self.repo)
        self.assertEqual(1, len(errors))
        self.assertIn("not_wrong.ino", errors[0])

    def test_migration_requires_missing_old_and_existing_new(self):
        mapping = self.write(
            "docs/migration_map.md",
            "| Old path | New path |\n|---|---|\n"
            "| `old/file.txt` | `new/file.txt` |\n",
        )
        self.write("new/file.txt")
        self.assertEqual([], check_migration_map(self.repo, mapping, 1))
        self.write("old/file.txt")
        errors = check_migration_map(self.repo, mapping, 1)
        self.assertTrue(any("old path still exists" in error for error in errors))

    def test_stale_token_is_rejected(self):
        readme = self.write("README.md", "Run Drone_Reciever.py")
        errors = check_stale_tokens(self.repo, [readme])
        self.assertEqual(1, len(errors))
        self.assertIn("Drone_Reciever.py", errors[0])


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run tests and confirm the expected red state**

```bash
/home/light/anaconda3/bin/python -m unittest tools/test_repo_layout.py -v
```

Expected: FAIL with `ModuleNotFoundError: No module named
'tools.check_repo_layout'`.

- [ ] **Step 3: Implement `tools/check_repo_layout.py`**

```python
#!/usr/bin/env python3
import re
import sys
from pathlib import Path
from urllib.parse import unquote


REPO_ROOT = Path(__file__).resolve().parents[1]
LINK_RE = re.compile(r"!?\[[^\]]*\]\(([^)]+)\)")
MAP_RE = re.compile(r"^\|\s*`([^`]+)`\s*\|\s*`([^`]+)`\s*\|")

LEGACY_TOKENS = (
    "firmware/examples/",
    "firmware/src/",
    "firmware/platformio_config/",
    "Drone_Control_Dualsense.py",
    "Drone_Tuning.py",
    "Drone_Reciever.py",
    "Drone_Monitor.py",
    "Drone_Analasys.py",
    "drone_telemetry.py",
    "dual_imu_pid_debug_receiver.py",
    "Controller_test.py",
    "GPS_Reciever.py",
    "ZETIN_Drone",
)


def _link_target(raw):
    raw = raw.strip()
    if raw.startswith("<") and ">" in raw:
        raw = raw[1 : raw.index(">")]
    else:
        raw = raw.split(maxsplit=1)[0]
    return unquote(raw)


def check_markdown_links(repo, files):
    errors = []
    for source in files:
        text = source.read_text(encoding="utf-8")
        for match in LINK_RE.finditer(text):
            target = _link_target(match.group(1))
            if not target or target.startswith("#"):
                continue
            if re.match(r"^[a-zA-Z][a-zA-Z0-9+.-]*:", target):
                continue
            target_path = target.split("#", 1)[0]
            if not target_path:
                continue
            if target_path.startswith("/"):
                errors.append(f"{source.relative_to(repo)}: absolute link {target}")
                continue
            resolved = (source.parent / target_path).resolve()
            if not resolved.exists():
                errors.append(
                    f"{source.relative_to(repo)}: missing link target {target_path}"
                )
    return errors


def check_sketch_names(repo):
    errors = []
    for ino in sorted((repo / "firmware").rglob("*.ino")):
        if ino.parent.name != ino.stem:
            errors.append(
                f"sketch name mismatch: {ino.relative_to(repo)} "
                f"(directory {ino.parent.name!r}, file {ino.stem!r})"
            )
    return errors


def check_migration_map(repo, map_path, expected_rows=44):
    errors = []
    rows = []
    for line_number, line in enumerate(
        map_path.read_text(encoding="utf-8").splitlines(), 1
    ):
        match = MAP_RE.match(line)
        if match and match.group(1) != "Old path":
            rows.append((line_number, match.group(1), match.group(2)))

    if len(rows) != expected_rows:
        errors.append(
            f"migration map has {len(rows)} rows; expected {expected_rows}"
        )

    old_seen = set()
    new_seen = set()
    for line_number, old, new in rows:
        if old in old_seen:
            errors.append(f"migration map line {line_number}: duplicate old path {old}")
        if new in new_seen:
            errors.append(f"migration map line {line_number}: duplicate new path {new}")
        old_seen.add(old)
        new_seen.add(new)
        if (repo / old).exists():
            errors.append(f"migration map line {line_number}: old path still exists {old}")
        if not (repo / new).exists():
            errors.append(f"migration map line {line_number}: new path missing {new}")
    return errors


def check_stale_tokens(repo, files):
    errors = []
    for source in files:
        text = source.read_text(encoding="utf-8")
        for token in LEGACY_TOKENS:
            if token in text:
                errors.append(f"{source.relative_to(repo)}: stale token {token}")
    return errors


def maintained_markdown_files(repo):
    explicit = [
        repo / "README.md",
        repo / "docs/README.md",
        repo / "docs/project_overview.md",
        repo / "docs/firmware_catalog.md",
        repo / "docs/udp_protocol.md",
        repo / "docs/migration_map.md",
        repo / "firmware/README.md",
        repo / "scripts/README.md",
        repo / "logs/README.md",
        repo / "firmware/archive/README.md",
        repo / "firmware/archive/dshot/README.md",
        repo / "firmware/archive/platformio_skeleton/README.md",
        repo / "scripts/archive/README.md",
    ]
    return [path for path in explicit if path.exists()]


def stale_scan_files(repo):
    # The migration map intentionally contains every legacy token, so it is
    # validated structurally but excluded from stale-reference rejection.
    files = [
        path
        for path in maintained_markdown_files(repo)
        if path != repo / "docs/migration_map.md"
    ]
    files.extend(sorted((repo / "firmware/flight").rglob("*.ino")))
    files.extend(sorted((repo / "firmware/diagnostics").rglob("*.ino")))
    files.extend(sorted((repo / "scripts").glob("*.py")))
    return files


def main():
    errors = []
    markdown = maintained_markdown_files(REPO_ROOT)
    errors.extend(check_markdown_links(REPO_ROOT, markdown))
    errors.extend(check_sketch_names(REPO_ROOT))
    errors.extend(
        check_migration_map(REPO_ROOT, REPO_ROOT / "docs/migration_map.md")
    )
    errors.extend(check_stale_tokens(REPO_ROOT, stale_scan_files(REPO_ROOT)))

    if errors:
        for error in errors:
            print(f"ERROR: {error}")
        return 1
    print("repository layout checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 4: Run checker tests until green**

```bash
/home/light/anaconda3/bin/python -m unittest tools/test_repo_layout.py -v
```

Expected: four tests pass.

- [ ] **Step 5: Add telemetry compatibility characterization tests**

Create `tools/test_telemetry_schema.py`:

```python
import sys
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from telemetry_schema import (  # noqa: E402
    CSV_FIELDS,
    TELEMETRY_FIELDS,
    parse_telemetry_packet,
    sample_to_csv_row,
)


def packet(field_count):
    values = [str(index + 0.5) for index in range(9)]
    values.extend(str(index) for index in range(9, field_count))
    return ",".join(values)


class TelemetryCompatibilityTest(unittest.TestCase):
    def test_10_field_packet_keeps_extended_values_unknown(self):
        sample = parse_telemetry_packet(packet(10))
        self.assertEqual(9, sample["Throttle"])
        self.assertIsNone(sample["Fault_RC"])
        self.assertEqual(22, len(sample_to_csv_row("00:00:00.000", sample)))

    def test_14_field_packet_populates_legacy_fault_and_rc_fields(self):
        sample = parse_telemetry_packet(packet(14))
        self.assertEqual(13, sample["RC_Dropped_Pkts"])
        self.assertIsNone(sample["Fault_IMU1"])

    def test_21_field_packet_populates_cascade_diagnostics(self):
        sample = parse_telemetry_packet(packet(21))
        self.assertEqual(20, sample["Calibration_OK"])
        self.assertEqual(21, len(TELEMETRY_FIELDS))
        self.assertEqual(22, len(CSV_FIELDS))

    def test_short_packet_is_rejected(self):
        with self.assertRaises(ValueError):
            parse_telemetry_packet(packet(9))


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 6: Run all repository and telemetry tests**

```bash
/home/light/anaconda3/bin/python -m unittest \
  tools/test_repo_layout.py \
  tools/test_telemetry_schema.py -v
/home/light/anaconda3/bin/python tools/check_repo_layout.py
```

Expected: eight unit tests pass and the checker prints
`repository layout checks passed`.

- [ ] **Step 7: Commit the validation tooling**

```bash
git add tools
git commit -m "test: validate repository layout and telemetry compatibility"
```

---

### Task 6: Run final cleanup verification and prepare review

**Files:** No new files unless a verification failure reveals a defect in a
file already owned by Tasks 1-5.

**Interfaces:** Produces a clean, locally verified branch. Does not push.

- [ ] **Step 1: Run complete Python and repository checks fresh**

```bash
/home/light/anaconda3/bin/python -m unittest \
  tools/test_repo_layout.py \
  tools/test_telemetry_schema.py -v
/home/light/anaconda3/bin/python tools/check_repo_layout.py
/home/light/anaconda3/bin/python -m py_compile scripts/*.py tools/*.py
```

Expected: eight tests pass, the repository checker passes, and py_compile
exits 0.

- [ ] **Step 2: Compile all six supported firmware sketches fresh**

```bash
for sketch in \
  firmware/flight/dual_imu_cascade_pwm \
  firmware/diagnostics/motor_pwm_bench \
  firmware/diagnostics/icm42670_single_raw \
  firmware/diagnostics/icm42670_dual_raw \
  firmware/diagnostics/icm42670_dual_loop_debug \
  firmware/diagnostics/board_v1_5_2_dual_imu; do
  name=$(basename "$sketch")
  arduino-cli compile --warnings all \
    --fqbn esp32:esp32:esp32s3 \
    --build-path "/tmp/zetin-final-$name" \
    "$sketch" || exit 1
done
```

Expected: six exit-0 builds. Do not substitute archive builds.

- [ ] **Step 3: Verify migration accounting and working-tree quality**

```bash
test "$(find firmware -type f -name '*.ino' | wc -l)" -eq 23
test "$(find firmware/flight firmware/diagnostics -type f -name '*.ino' | wc -l)" -eq 6
test "$(find firmware/archive -type f -name '*.ino' | wc -l)" -eq 17
git diff --check origin/main...HEAD
git status --short --branch
git log --oneline --decorate origin/main..HEAD
```

Expected:

- 23 total, 6 current, and 17 archived sketches;
- no whitespace errors;
- no staged or unstaged project files;
- local agent/editor paths do not appear because they are ignored;
- local branch is ahead only by the cleanup documentation and implementation
  commits.

- [ ] **Step 4: Review the final diff for forbidden behavior changes**

```bash
git diff --find-renames --stat origin/main...HEAD
git diff --find-renames origin/main...HEAD -- \
  firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino \
  scripts/control_dualsense.py \
  scripts/tune_pid.py \
  scripts/telemetry_schema.py
```

Expected: current firmware/script diffs contain path/comment/import-only
changes. There must be no gain, pin, motor mixer, packet order, socket address,
or command parsing change.

- [ ] **Step 5: Hand off for user review without pushing**

Report:

- final commit list;
- 6 current and 17 archived sketch counts;
- firmware compile results;
- Python, unit-test, and checker results;
- exact design and implementation-plan links;
- confirmation that no archive code was repaired or build-claimed;
- confirmation that the branch has not been pushed.
