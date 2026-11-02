"""Static guards for the ESP32-CAM stream sketch and the laptop camera tools.

These do not build anything. They pin the spec's anti-goals that a compile
cannot see: the camera path must never touch the flight UDP port, the capture
timestamp must not be printed with a 32-bit format, and the static IP must
stay outside the flight SoftAP's DHCP pool.
"""

import re
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SKETCH_DIR = REPO_ROOT / "firmware" / "camera" / "esp32cam_stream"
SKETCH = SKETCH_DIR / "esp32cam_stream.ino"
CAMERA_SCRIPTS = (
    REPO_ROOT / "scripts" / "camera_stream.py",
    REPO_ROOT / "scripts" / "record_camera.py",
    REPO_ROOT / "scripts" / "camera_to_mp4.py",
)
CI = REPO_ROOT / ".github" / "workflows" / "ci.yml"

# ESP-IDF dhcpserver 의 기본 풀: SoftAP 주소 + 1 부터 DHCPS_MAX_LEASE(0x64) 개.
DHCP_POOL = range(2, 102)


class CameraSketchStaticTest(unittest.TestCase):
    def test_never_touches_flight_udp_port(self):
        self.assertTrue(SKETCH.exists(), f"missing {SKETCH}")
        sources = [p for p in sorted(SKETCH_DIR.iterdir()) if p.is_file()]
        sources += [p for p in CAMERA_SCRIPTS if p.exists()]
        for path in sources:
            text = path.read_text(encoding="utf-8")
            with self.subTest(path=path.relative_to(REPO_ROOT)):
                self.assertNotIn("4210", text)
                self.assertNotIn("WiFiUDP", text)

    def test_timestamp_not_printed_with_int_format(self):
        self.assertNotIn('"%d.%06d"', SKETCH.read_text(encoding="utf-8"))

    def test_static_ip_outside_dhcp_pool(self):
        text = SKETCH.read_text(encoding="utf-8")
        hosts = [int(n) for n in re.findall(r"IPAddress\(\s*192\s*,\s*168\s*,\s*4\s*,\s*(\d+)\s*\)", text)]
        self.assertIn(200, hosts, "camera static IP 192.168.4.200 not found")
        self.assertNotIn(200, DHCP_POOL)

    def test_ci_compiles_camera_with_esp32cam_fqbn(self):
        text = CI.read_text(encoding="utf-8")
        self.assertIn("esp32:esp32:esp32cam", text)
        self.assertIn("firmware/camera/esp32cam_stream", text)


if __name__ == "__main__":
    unittest.main()
