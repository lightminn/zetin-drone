# Repository Cleanup Design

**Date:** 2026-07-13  
**Status:** Approved for implementation planning

## 1. Goal

Restructure the repository so a reader can immediately distinguish the current
ESP32-S3 + ICM42670 + PWM path from deprecated experiments, while preserving
the latter under an explicit archive. After the migration, every maintained
document and command must point to an existing file or directory.

## 2. Scope and decisions

- The only supported hardware/control stack is ESP32-S3 + ICM42670 + PWM.
- `DUAL_IMU_CASCADE` is the current flight-controller candidate. It remains
  experimental until hardware flight validation proves otherwise.
- Current diagnostics are the PWM motor bench, ICM42670 raw readers, the dual
  IMU loop diagnostic, and the PCB v1.5.2 dual-IMU diagnostic.
- DShot, MPU6500, STM32 F411, Kalman experiments, BMM350, GPS, US100, the old
  PlatformIO skeleton, TCP tests, and superseded PID variants are deprecated.
- Deprecated code is moved with `git mv` and preserved under `archive/`. It is
  not repaired, modernized, or included in the supported build matrix.
- Arduino CLI is the supported firmware build path. The stale PlatformIO
  project is archived as historical material instead of being restored.
- Python tools are renamed to descriptive `lower_snake_case` names without
  changing their command or telemetry behavior.
- The UDP command protocol and 10/14/21-field telemetry compatibility remain
  unchanged.
- No compatibility symlinks or duplicate wrapper scripts are kept under the
  old names. `docs/migration_map.md` is the authoritative old-to-new lookup.
- Local AI-agent and machine-specific editor files are removed from tracking
  and excluded from repository checks.

## 3. Naming rules

1. Directories and source files use `lower_snake_case`.
2. An Arduino sketch directory and its primary `.ino` basename must match.
3. Names describe target and purpose, not chronology: use
   `icm42670_dual_raw`, not `NEW_DUAL_TEST`.
4. Version punctuation becomes underscores: use `board_v1_5_2_dual_imu`.
5. Lifecycle status is represented by directory placement (`flight/`,
   `diagnostics/`, `archive/`), not by vague suffixes such as `_TEST`.
6. Archived unsafe code uses an explicit name and archive warning; it is never
   presented as a runnable quick-start example.

## 4. Target repository structure

```text
zetin-drone/
├── README.md
├── firmware/
│   ├── README.md
│   ├── flight/
│   │   └── dual_imu_cascade_pwm/
│   │       └── dual_imu_cascade_pwm.ino
│   ├── diagnostics/
│   │   ├── motor_pwm_bench/
│   │   ├── icm42670_single_raw/
│   │   ├── icm42670_dual_raw/
│   │   ├── icm42670_dual_loop_debug/
│   │   └── board_v1_5_2_dual_imu/
│   └── archive/
│       ├── README.md
│       ├── legacy_flight/
│       ├── filter_experiments/
│       ├── dshot/
│       ├── other_sensors/
│       ├── other_mcus/
│       └── platformio_skeleton/
├── scripts/
│   ├── README.md
│   ├── control_dualsense.py
│   ├── tune_pid.py
│   ├── receive_telemetry.py
│   ├── monitor_telemetry.py
│   ├── analyze_flight_log.py
│   ├── telemetry_schema.py
│   ├── receive_dual_imu_debug.py
│   ├── test_dualsense_input.py
│   └── archive/
├── logs/
│   └── README.md
├── docs/
│   ├── README.md
│   ├── project_overview.md
│   ├── firmware_catalog.md
│   ├── udp_protocol.md
│   ├── migration_map.md
│   ├── design/
│   ├── history/
│   └── archive/
└── tools/
    └── check_repo_layout.py
```

## 5. Firmware migration map

### 5.1 Current flight and diagnostics

| Old path | New path | Role |
|---|---|---|
| `firmware/examples/DUAL_IMU_CASCADE/DUAL_IMU_CASCADE.ino` | `firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino` | Current flight-controller candidate |
| `firmware/examples/PWM_TEST/PWM_TEST.ino` | `firmware/diagnostics/motor_pwm_bench/motor_pwm_bench.ino` | Four-motor PWM bench test |
| `firmware/examples/IMU_TEST_RAW/IMU_TEST_RAW.ino` | `firmware/diagnostics/icm42670_single_raw/icm42670_single_raw.ino` | Single ICM42670 raw SPI reader; pinout documented explicitly |
| `firmware/examples/DUAL_IMU_RAW_TEST/DUAL_IMU_RAW_TEST.ino` | `firmware/diagnostics/icm42670_dual_raw/icm42670_dual_raw.ino` | Dual ICM42670 axis/sign reader |
| `firmware/examples/DUAL_IMU_PID_DEBUG/DUAL_IMU_PID_DEBUG.ino` | `firmware/diagnostics/icm42670_dual_loop_debug/icm42670_dual_loop_debug.ino` | Dual-IMU loop timing/filter diagnostic |
| `firmware/examples/PCB_1.5.2_TEST/PCB_1.5.2_TEST.ino` | `firmware/diagnostics/board_v1_5_2_dual_imu/board_v1_5_2_dual_imu.ino` | PCB v1.5.2 dual-IMU smoke test |

### 5.2 Deprecated flight and filter experiments

| Old path | Archive path |
|---|---|
| `firmware/examples/PWM_TEST_IMU_PID/PWM_TEST_IMU_PID.ino` | `firmware/archive/legacy_flight/single_imu_pid_pwm/single_imu_pid_pwm.ino` |
| `firmware/examples/PWM_TEST_DUAL_IMU_PID/PWM_TEST_DUAL_IMU_PID.ino` | `firmware/archive/legacy_flight/dual_imu_pid_pwm/dual_imu_pid_pwm.ino` |
| `firmware/examples/DUAL_LOOP_TEST/DUAL_LOOP_TEST.ino` | `firmware/archive/legacy_flight/single_imu_cascade_pwm/single_imu_cascade_pwm.ino` |
| `firmware/examples/KALMAN_FLIGHT_TEST/KALMAN_FLIGHT_TEST.ino` | `firmware/archive/legacy_flight/single_imu_kalman_pid_pwm/single_imu_kalman_pid_pwm.ino` |
| `firmware/examples/KALMAN_TEST/KALMAN_TEST.ino` | `firmware/archive/filter_experiments/icm42670_kalman_attitude/icm42670_kalman_attitude.ino` |

### 5.3 Deprecated DShot and motor-allocation experiments

| Old path | Archive path |
|---|---|
| `firmware/examples/DSHOT_TEST/DSHOT_TEST.ino` | `firmware/archive/dshot/single_motor_fixed_throttle/single_motor_fixed_throttle.ino` |
| `firmware/examples/DSHOT_TEST_PERCENT/DSHOT_TEST_PERCENT.ino` | `firmware/archive/dshot/single_motor_serial_control/single_motor_serial_control.ino` |
| `firmware/examples/DSHOT_TEST_IMU/DSHOT_TEST_IMU.ino` | `firmware/archive/dshot/mpu6500_tilt_throttle/mpu6500_tilt_throttle.ino` |
| `firmware/examples/DSHOT_TEST_IMU_PID/DSHOT_TEST_IMU_PID.ino` | `firmware/archive/dshot/mpu6500_pid_bench/mpu6500_pid_bench.ino` |
| `firmware/examples/RMT_TEST/RMT_TEST.ino` | `firmware/archive/dshot/four_motor_full_throttle_unsafe/four_motor_full_throttle_unsafe.ino` |
| `firmware/examples/ZETIN_Drone_Standalone_Test/ZETIN_Drone_Standalone_Test.ino` | `firmware/archive/dshot/motor_allocator_dual_core/motor_allocator_dual_core.ino` |
| `firmware/examples/MOTOR_ALGO_TEST/MOTOR_ALGO_TEST.ino` | `firmware/archive/platformio_skeleton/examples/motor_allocator_dshot/motor_allocator_dshot.ino` |

The DShot archive README must call out missing dependencies and the unsafe
full-throttle sketch. None of these sketches is part of verification.

### 5.4 Deprecated sensors and MCU targets

| Old path | Archive path |
|---|---|
| `firmware/examples/BMM_TEST/BMM_TEST.ino` | `firmware/archive/other_sensors/bmm350_read/bmm350_read.ino` |
| `firmware/examples/PCB_1.5.2_BMM_TEST/PCB_1.5.2_BMM_TEST.ino` | `firmware/archive/other_sensors/board_v1_5_2_bmm350/board_v1_5_2_bmm350.ino` |
| `firmware/examples/GPS_TEST_RAW/GPS_TEST_RAW.ino` | `firmware/archive/other_sensors/gps_uart_passthrough/gps_uart_passthrough.ino` |
| `firmware/examples/US100_TEST/US100_TEST.ino` | `firmware/archive/other_sensors/us100_distance/us100_distance.ino` |
| `firmware/examples/F411_test/F411_test.ino` | `firmware/archive/other_mcus/stm32_f411_uart_rx/stm32_f411_uart_rx.ino` |

### 5.5 Deprecated PlatformIO skeleton

The following are moved together under
`firmware/archive/platformio_skeleton/` so their historical relationship is
preserved:

- `firmware/src/`
- `firmware/lib/`
- `firmware/include/`
- `firmware/platformio_config/platformio.ini`
- the generated PlatformIO `test/README`

The archive README states that this project contains simulated sensor data,
the canceled motor allocator, old TCP transport, stale dependencies, and no
supported build guarantee.

## 6. Python migration map

| Old path | New path | Required reference update |
|---|---|---|
| `scripts/Drone_Control_Dualsense.py` | `scripts/control_dualsense.py` | README commands and firmware comments |
| `scripts/Drone_Tuning.py` | `scripts/tune_pid.py` | README commands, firmware comments, historical docs |
| `scripts/Drone_Reciever.py` | `scripts/receive_telemetry.py` | README commands and telemetry import |
| `scripts/Drone_Monitor.py` | `scripts/monitor_telemetry.py` | README commands and telemetry import |
| `scripts/Drone_Analasys.py` | `scripts/analyze_flight_log.py` | README and logs documentation |
| `scripts/drone_telemetry.py` | `scripts/telemetry_schema.py` | imports in receiver and monitor |
| `scripts/dual_imu_pid_debug_receiver.py` | `scripts/receive_dual_imu_debug.py` | paired firmware path in docstring and catalog |
| `scripts/Controller_test.py` | `scripts/test_dualsense_input.py` | scripts catalog |
| `scripts/GPS_Reciever.py` | `scripts/archive/receive_gps_udp_legacy.py` | archive catalog only |
| `test/tcp_test.py` | `scripts/archive/test_tcp_legacy.py` | archive catalog only |

Renaming corrects `Reciever` to `receive` and `Analasys` to `analyze`; the
misspellings are not preserved in new names.

## 7. Documentation architecture

### 7.1 Maintained documents

- `README.md`: short current-state entry point and quick start. It links only
  to supported flight, diagnostics, tools, and the document index.
- `docs/README.md`: complete document index grouped as maintained, historical,
  and archived.
- `docs/project_overview.md`: renamed and corrected onboarding document. It
  preserves the verified-versus-experimental boundary.
- `docs/firmware_catalog.md`: one row per current and archived sketch with
  target MCU, sensor, motor protocol, lifecycle, safety note, and exact link.
- `docs/udp_protocol.md`: authoritative command grammar, UDP port, handshake,
  and 10/14/21-field telemetry schema.
- `docs/migration_map.md`: complete old-path to new-path mapping for firmware,
  scripts, tests, and documents.
- `firmware/README.md`: Arduino CLI build instructions and current sketches;
  it must not present the archived PlatformIO skeleton as current.
- `scripts/README.md`: dependencies and exact commands using renamed scripts.
- `logs/README.md`: actual `flight_log_YYYY-MM-DD_HHMMSS.csv` convention and
  the real 22-column CSV schema, including Timestamp.

### 7.2 Historical and archived documents

- Move the 2026-05-14 dual-IMU PID design and implementation plan from
  `docs/superpowers/` to `docs/history/`, update their exact file links, and
  mark the resulting PID firmware as superseded by the current cascade path.
- Move `docs/presentation.c` to
  `docs/archive/pid_dshot_presentation_snippet.c` and label it non-runnable.
- Move `docs/commit_message.txt` to
  `docs/archive/commit_message_snapshot.txt`; it is not a commit convention.
- Fold `test/README.md` into `scripts/archive/README.md`, then remove the empty
  top-level `test/` directory.

### 7.3 Reference rules

- Exact repository paths in maintained docs use resolvable Markdown links.
- Code examples may use backticks, but any path intended for navigation must
  also have a link.
- Old paths are allowed only in `docs/migration_map.md` and clearly labeled
  historical descriptions.
- Firmware source comments naming Python tools are updated to the new paths.
- Python docstrings naming paired firmware are updated to the new paths.
- Machine-local `.codex`, `.claude`, `.gemini`, `.vscode`, and `AGENTS.md`
  content is outside documentation-check scope.

## 8. Local and generated files

Remove tracked `.gemini/` and `.vscode/` configuration. The latter contains
machine-specific Windows paths and the old `ZETIN_Drone` repository name.
Extend `.gitignore` to cover:

- `.codex/`, `.claude/`, `.gemini/`
- `AGENTS.md` at any repository depth
- `.vscode/`
- `__pycache__/`, `*.pyc`
- editor swap files including `*.swp` and `*.kate-swp`
- existing PlatformIO and CSV output patterns

These paths are neither migrated nor checked by the repository validator.

## 9. Repository validation

Create `tools/check_repo_layout.py` with four deterministic checks:

1. Resolve local Markdown links in `README.md`, `docs/`, `firmware/README.md`,
   `scripts/README.md`, and `logs/README.md` relative to their source file.
2. Confirm every Arduino sketch directory matches its `.ino` basename,
   including archived sketches.
3. Confirm every old/new pair in `docs/migration_map.md` has one missing old
   path and one existing new path after migration.
4. Reject legacy path/name tokens in maintained docs and current source while
   allowing them in the migration map and explicitly historical documents.

Verification after all moves:

```bash
/home/light/anaconda3/bin/python tools/check_repo_layout.py
/home/light/anaconda3/bin/python -m py_compile scripts/*.py
arduino-cli compile --warnings all --fqbn esp32:esp32:esp32s3 \
  --build-path /tmp/zetin-flight-build \
  firmware/flight/dual_imu_cascade_pwm
```

Compile each directory under `firmware/diagnostics/` with a separate `/tmp`
build path. Run telemetry parser cases for exactly 10, 14, and 21 input fields
and verify a 22-column CSV row including Timestamp. Do not compile anything
under `firmware/archive/`.

Final repository checks:

- `git status --short` shows no untracked AI/editor/generated files.
- `git diff --check` reports no whitespace errors.
- local `main` and `origin/main` are compared only after the cleanup commits
  are complete and explicitly approved for push.

## 10. Commit strategy

1. `chore: ignore local agent and editor files`
   - remove tracked `.gemini/` and `.vscode/`; update `.gitignore`.
2. `refactor: separate current and archived firmware`
   - move all sketches and the PlatformIO skeleton; add firmware archive and
     catalog documentation in the same commit so paths remain explainable.
3. `refactor: rename drone utility scripts`
   - rename current tools, update Python imports and paired firmware comments,
     and archive GPS/TCP tools.
4. `docs: align project documentation with repository layout`
   - update every maintained document, move historical material, and add the
     complete migration map.
5. `test: add repository layout validation`
   - add the link/layout checker and parser cases, then run the full supported
     verification set.

No cleanup commit changes controller gains, pin assignments, motor mixing,
telemetry field order, UDP commands, or archived implementation logic.

## 11. Acceptance criteria

- From the root README, the current flight sketch is reachable within two
  links and is identified as experimental.
- Every supported script command and firmware build command names an existing
  path.
- Every maintained local Markdown link resolves.
- All 23 original sketches appear exactly once in the migration map and at
  exactly one destination.
- Current firmware lives only in `firmware/flight/` and
  `firmware/diagnostics/`; deprecated firmware lives only in
  `firmware/archive/`.
- Archive READMEs explicitly state that archived code is unsupported and not
  build-verified.
- The current cascade sketch and all current diagnostics compile for the
  intended ESP32-S3 target.
- Current Python scripts compile and their renamed imports resolve.
- Telemetry parsing remains compatible with 10-, 14-, and 21-field packets.
- The CSV logger still writes Timestamp plus the 21 telemetry fields.
- No old path remains in maintained docs outside the migration map.
- AI-agent, editor, cache, swap, and generated log files stay out of Git.
