import shutil
import signal
import subprocess
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[1]
NATIVE_TEST_DIR = REPO_ROOT / "tools" / "native_tests"
SKETCH_DIR = (
    REPO_ROOT / "firmware" / "flight" / "dual_imu_cascade_pwm"
)


def test_native_control_math(tmp_path):
    compiler = shutil.which("g++")
    if compiler is None:
        pytest.skip("g++ is unavailable; skipping native firmware logic tests")

    m32_probe = tmp_path / "m32_probe"
    probe_command = [
        compiler,
        "-std=c++17",
        "-m32",
        "-x",
        "c++",
        "-",
        "-o",
        str(m32_probe),
    ]
    try:
        probe = subprocess.run(
            probe_command,
            input="static_assert(sizeof(long) == 4); int main() { return 0; }\n",
            capture_output=True,
            text=True,
            check=False,
            timeout=60,
        )
    except subprocess.TimeoutExpired as error:
        pytest.fail(
            "32-bit g++ multilib toolchain probe timed out after 60 seconds\n"
            f"command: {' '.join(probe_command)}\n"
            f"stdout:\n{error.stdout or ''}\n"
            f"stderr:\n{error.stderr or ''}"
        )
    if probe.returncode != 0:
        pytest.skip(
            "32-bit g++ multilib toolchain is unavailable; "
            "native ESP32-width tests require -m32\n"
            f"command: {' '.join(probe_command)}\n"
            f"stdout:\n{probe.stdout}\n"
            f"stderr:\n{probe.stderr}"
        )

    executable = tmp_path / "test_control_math"
    compile_command = [
        compiler,
        "-std=c++17",
        "-O0",
        "-g",
        "-m32",
        "-I",
        str(NATIVE_TEST_DIR / "shims"),
        "-I",
        str(SKETCH_DIR),
        "-x",
        "c++",
        str(NATIVE_TEST_DIR / "test_control_math.cpp"),
        "-o",
        str(executable),
    ]
    try:
        compiled = subprocess.run(
            compile_command,
            capture_output=True,
            text=True,
            check=False,
            timeout=60,
        )
    except subprocess.TimeoutExpired as error:
        pytest.fail(
            "native firmware test compilation timed out after 60 seconds\n"
            f"command: {' '.join(compile_command)}\n"
            f"stdout:\n{error.stdout or ''}\n"
            f"stderr:\n{error.stderr or ''}"
        )
    if compiled.returncode != 0:
        pytest.fail(
            "native firmware test compilation failed\n"
            f"command: {' '.join(compile_command)}\n"
            f"stdout:\n{compiled.stdout}\n"
            f"stderr:\n{compiled.stderr}"
        )

    try:
        completed = subprocess.run(
            [str(executable)],
            capture_output=True,
            text=True,
            check=False,
            timeout=10,
        )
    except subprocess.TimeoutExpired as error:
        pytest.fail(
            "native firmware logic tests timed out after 10 seconds\n"
            f"stdout:\n{error.stdout or ''}\n"
            f"stderr:\n{error.stderr or ''}"
        )
    if completed.returncode == -signal.SIGSYS:
        qemu_i386 = shutil.which("qemu-i386-static")
        if qemu_i386 is None:
            pytest.fail(
                "32-bit native test execution was blocked by SIGSYS and "
                "qemu-i386-static is unavailable"
            )
        try:
            completed = subprocess.run(
                [qemu_i386, str(executable)],
                capture_output=True,
                text=True,
                check=False,
                timeout=10,
            )
        except subprocess.TimeoutExpired as error:
            pytest.fail(
                "native firmware logic tests under qemu-i386-static timed out "
                "after 10 seconds\n"
                f"stdout:\n{error.stdout or ''}\n"
                f"stderr:\n{error.stderr or ''}"
            )
    if completed.returncode != 0:
        pytest.fail(
            "native firmware logic tests failed\n"
            f"stdout:\n{completed.stdout}\n"
            f"stderr:\n{completed.stderr}"
        )
