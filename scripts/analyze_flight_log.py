import sys
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd


SCRIPT_DIR = Path(__file__).resolve().parent
LOG_DIR = SCRIPT_DIR.parent / "logs"


def select_log_file():
    if len(sys.argv) >= 2:
        selected = Path(sys.argv[1]).expanduser().resolve()
        if not selected.is_file():
            raise FileNotFoundError(f"지정한 CSV 파일이 없습니다: {selected}")
        return selected

    candidates = list(LOG_DIR.glob("*.csv"))
    if not candidates:
        raise FileNotFoundError(f"CSV 파일이 없습니다: {LOG_DIR}")
    return max(candidates, key=lambda path: path.stat().st_mtime)


try:
    file_path = select_log_file()
    print(f"📂 분석 대상: {file_path}")
    df = pd.read_csv(file_path, skipinitialspace=True)
    df.columns = df.columns.str.strip()
    if df.empty:
        raise ValueError("파일이 비어 있습니다")
except (OSError, ValueError, pd.errors.ParserError) as exc:
    print(f"❌ 로그 로딩 실패: {exc}")
    sys.exit(1)

# 과거 실험 로그에서 사용했을 가능성이 있는 이름을 현재 스키마로 정규화한다.
column_aliases = {
    "Fault_IMU": "Fault_Critical",
    "Scaled": "Mixer_Scaled",
    "scaled": "Mixer_Scaled",
    "Active_Imus": "Active_IMUs",
    "active_imus": "Active_IMUs",
}
for old_name, new_name in column_aliases.items():
    if old_name in df.columns and new_name not in df.columns:
        df.rename(columns={old_name: new_name}, inplace=True)

for column in df.columns:
    if column != "Timestamp":
        df[column] = pd.to_numeric(df[column], errors="coerce")

print(f"✅ 데이터 로딩 완료! ({len(df)}개 샘플)")

cols_attitude = ["Roll", "Pitch", "Yaw"]
cols_gyro = ["Gyro_X", "Gyro_Y", "Gyro_Z"]
cols_accel = ["Accel_X", "Accel_Y", "Accel_Z"]
cols_control = ["Throttle", "Active_IMUs", "Mixer_Scaled"]
cols_fault = [
    "Fault_RC",
    "Fault_Critical",
    "Fault_IMU1",
    "Fault_IMU2",
    "Fault_Disagree",
    "Fault_Attitude",
    "Calibration_OK",
]

summary_columns = [
    column
    for column in cols_attitude + cols_gyro + cols_accel + cols_control
    if column in df.columns and df[column].notna().any()
]

print("\n" + "=" * 72)
print("📊 비행 데이터 요약 통계")
print("=" * 72)
if summary_columns:
    print(df[summary_columns].describe().round(3))
else:
    print("분석할 수치 열이 없습니다.")


def asserted_samples(column, invert=False):
    if column not in df.columns:
        return None
    valid = df[column].dropna()
    if valid.empty:
        return None
    asserted = valid <= 0 if invert else valid > 0
    return int(asserted.sum()), len(valid)


print("\n🔎 제어·고장 이벤트")
scaled_result = asserted_samples("Mixer_Scaled")
if scaled_result is not None:
    count, total = scaled_result
    print(f"  Mixer scaled: {count}/{total} 샘플 ({count / total * 100:.2f}%)")

if "Active_IMUs" in df.columns and df["Active_IMUs"].notna().any():
    active = df["Active_IMUs"].dropna()
    degraded = int((active < 2).sum())
    unavailable = int((active <= 0).sum())
    print(
        f"  Active IMUs: 최소 {active.min():.0f}, "
        f"degraded {degraded}샘플, unavailable {unavailable}샘플"
    )

for column in cols_fault:
    result = asserted_samples(column, invert=(column == "Calibration_OK"))
    if result is None:
        continue
    count, total = result
    label = "Calibration_Fail" if column == "Calibration_OK" else column
    print(f"  {label}: {count}/{total} 샘플")

if all(column in df.columns for column in ("RC_Total_Pkts", "RC_Dropped_Pkts")):
    total_series = df["RC_Total_Pkts"].dropna()
    dropped_series = df["RC_Dropped_Pkts"].dropna()
    if not total_series.empty and not dropped_series.empty:
        total = int(total_series.iloc[-1])
        dropped = int(dropped_series.iloc[-1])
        rate = dropped / total * 100 if total > 0 else 0.0
        print(f"  RC packet drops: {dropped}/{total} ({rate:.2f}%)")
print("=" * 72)

plt.style.use(
    "seaborn-v0_8-darkgrid"
    if "seaborn-v0_8-darkgrid" in plt.style.available
    else "default"
)

fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1, figsize=(13, 15), sharex=True)
fig.suptitle(f"Flight Analysis: {file_path.name}", fontsize=16, fontweight="bold")
x_axis = pd.RangeIndex(len(df))

for column, color, style in (
    ("Roll", "red", "-"),
    ("Pitch", "blue", "-"),
    ("Yaw", "green", "--"),
):
    if column in df.columns:
        ax1.plot(x_axis, df[column], label=column, color=color, linestyle=style, linewidth=1.3)
ax1.axhline(0, color="black", linestyle=":", alpha=0.5)
ax1.set_ylabel("Angle (deg)")
ax1.set_title("1. Attitude Response")
ax1.set_ylim(-60, 60)
ax1.legend(loc="upper right")

for column, color in (
    ("Gyro_X", "red"),
    ("Gyro_Y", "blue"),
    ("Gyro_Z", "green"),
):
    if column in df.columns:
        ax2.plot(x_axis, df[column], label=column, color=color, alpha=0.65, linewidth=0.9)
ax2.set_ylabel("Angular rate (dps)")
ax2.set_title("2. Gyroscope (Vibration Check)")
ax2.legend(loc="upper right")

for column, color in (
    ("Accel_X", "red"),
    ("Accel_Y", "blue"),
    ("Accel_Z", "green"),
):
    if column in df.columns:
        ax3.plot(x_axis, df[column], label=column, color=color, alpha=0.55, linewidth=0.9)
ax3.set_ylabel("Acceleration (g)")
ax3.set_ylim(-2.0, 2.0)
ax3.set_title("3. Accelerometer & Throttle")
ax3.legend(loc="upper left")

ax3_right = ax3.twinx()
if "Throttle" in df.columns:
    ax3_right.plot(x_axis, df["Throttle"], label="Throttle", color="orange", linewidth=1.5)
    throttle_valid = df["Throttle"].dropna()
    if not throttle_valid.empty and throttle_valid.max() <= 100:
        throttle_label = "Throttle (%)"
        ax3_right.set_ylim(0, 100)
    else:
        throttle_label = "Throttle (PWM)"
        ax3_right.set_ylim(1000, 2000)
else:
    throttle_label = "Throttle"
ax3_right.set_ylabel(throttle_label, color="orange")
ax3_right.tick_params(axis="y", labelcolor="orange")
ax3_right.legend(loc="upper right")

event_rows = (
    ("Mixer_Scaled", "Mixer scaled", False, "tab:orange"),
    ("Fault_RC", "RC fault", False, "tab:red"),
    ("Fault_Critical", "Critical fault", False, "darkred"),
    ("Fault_IMU1", "IMU1 fault", False, "tab:purple"),
    ("Fault_IMU2", "IMU2 fault", False, "tab:pink"),
    ("Fault_Disagree", "IMU disagree", False, "tab:brown"),
    ("Fault_Attitude", "Attitude fault", False, "black"),
    ("Calibration_OK", "Calibration fail", True, "tab:gray"),
)

event_labels = []
event_positions = []
for column, label, invert, color in event_rows:
    if column not in df.columns or not df[column].notna().any():
        continue
    asserted = df[column].le(0) if invert else df[column].gt(0)
    position = len(event_labels)
    event_labels.append(label)
    event_positions.append(position)
    event_x = df.index[asserted.fillna(False)]
    if len(event_x):
        ax4.scatter(event_x, [position] * len(event_x), marker="|", s=90, color=color)

if event_labels:
    ax4.set_yticks(event_positions, event_labels)
    ax4.set_ylim(-0.7, len(event_labels) - 0.3)
else:
    ax4.text(
        0.5,
        0.5,
        "Legacy log: no extended fault/status fields",
        ha="center",
        va="center",
        transform=ax4.transAxes,
    )
    ax4.set_yticks([])

ax4_active = ax4.twinx()
if "Active_IMUs" in df.columns and df["Active_IMUs"].notna().any():
    ax4_active.step(
        x_axis,
        df["Active_IMUs"],
        where="post",
        color="tab:blue",
        linewidth=1.5,
        label="Active IMUs",
    )
    ax4_active.legend(loc="upper right")
ax4_active.set_ylabel("Active IMUs", color="tab:blue")
ax4_active.set_ylim(-0.1, 2.2)
ax4_active.set_yticks([0, 1, 2])
ax4.set_title("4. Saturation, Redundancy & Fault Events")
ax4.set_xlabel("Sample count")
ax4.grid(True, axis="x", alpha=0.4)

fig.tight_layout(rect=(0, 0, 1, 0.97))
plt.show()
