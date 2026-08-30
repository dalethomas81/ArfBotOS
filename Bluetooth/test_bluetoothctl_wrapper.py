from __future__ import annotations

import unittest
from unittest.mock import MagicMock, patch

from bluetoothctl_wrapper import (
    BluetoothctlWrapper,
    looks_like_controller,
    validate_mac_address,
)


SHOW_OUTPUT = """
Controller AA:BB:CC:DD:EE:FF arfbot-pi [default]
        Name: BlueZ 5.66
        Alias: arfbot-pi
        Powered: yes
        Discoverable: no
        Pairable: yes
        Discovering: no
        PowerState: on
"""

DEVICES_OUTPUT = """
Device 11:22:33:44:55:66 DualSense Wireless Controller
Device 77:88:99:AA:BB:CC Dale's iPhone
"""

INFO_CONTROLLER = """
Device 11:22:33:44:55:66 (public)
        Name: DualSense Wireless Controller
        Alias: DualSense Wireless Controller
        Icon: input-gaming
        Paired: yes
        Trusted: yes
        Blocked: no
        Connected: yes
        RSSI: -52
"""

INFO_PHONE = """
Device 77:88:99:AA:BB:CC (public)
        Name: Dale's iPhone
        Alias: Dale's iPhone
        Icon: phone
        Paired: no
        Trusted: no
        Blocked: no
        Connected: no
"""


class HelperTests(unittest.TestCase):
    def test_validate_mac(self):
        self.assertTrue(validate_mac_address("AA:BB:CC:DD:EE:FF"))
        self.assertFalse(validate_mac_address("not-a-mac"))

    def test_controller_hints(self):
        self.assertTrue(looks_like_controller("DualSense Wireless Controller", "input-gaming"))
        self.assertTrue(looks_like_controller("Wireless Controller"))
        self.assertFalse(looks_like_controller("Dale's iPhone", "phone"))


class WrapperTests(unittest.TestCase):
    def setUp(self):
        self.bt = BluetoothctlWrapper(executable="bluetoothctl-fake")

    def _completed(self, stdout: str, returncode: int = 0):
        return {
            "ok": returncode == 0,
            "message": "Command completed",
            "stdout": stdout,
            "stderr": "",
            "returncode": returncode,
        }

    def test_get_status_parses_adapter(self):
        with patch.object(self.bt, "_run_session", return_value=self._completed(SHOW_OUTPUT)):
            result = self.bt.get_status()
        adapter = result["adapter"]
        self.assertEqual(adapter["controller"], "AA:BB:CC:DD:EE:FF")
        self.assertTrue(adapter["powered"])
        self.assertTrue(adapter["pairable"])
        self.assertFalse(adapter["discovering"])

    def test_discovered_devices_sort_controllers_first_without_enrich(self):
        with patch.object(self.bt, "_run_session", return_value=self._completed(DEVICES_OUTPUT)):
            result = self.bt.get_discovered_devices(enrich=False)
        names = [d["name"] for d in result["devices"]]
        self.assertEqual(names[0], "DualSense Wireless Controller")
        self.assertTrue(result["devices"][0]["is_controller"])
        self.assertFalse(result["devices"][1]["is_controller"])

    def test_setup_device_reports_connected(self):
        def run_session(commands, timeout_s=None):
            joined = " ".join(commands)
            if "power on" in joined:
                return self._completed("Changing power on succeeded")
            if "pair" in joined and "trust" in joined and "connect" in joined:
                return self._completed(
                    "Pairing successful\nTrust succeeded\nConnection successful"
                )
            if commands == [f"info 11:22:33:44:55:66"]:
                return self._completed(INFO_CONTROLLER)
            if commands == ["show"]:
                return self._completed(SHOW_OUTPUT)
            return self._completed("")

        with patch.object(self.bt, "_run_session", side_effect=run_session):
            result = self.bt.setup_device("11:22:33:44:55:66")

        self.assertTrue(result["ok"])
        self.assertIn("connected", result["message"].lower())
        self.assertTrue(result["device"]["connected"])

    def test_start_scan_keeps_process_alive(self):
        fake_proc = MagicMock()
        fake_proc.poll.return_value = None
        fake_proc.stdin = MagicMock()

        with patch.object(self.bt, "ensure_ready", return_value={"ok": True, "message": "ready"}):
            with patch("bluetoothctl_wrapper.subprocess.Popen", return_value=fake_proc):
                with patch("bluetoothctl_wrapper.time.sleep", return_value=None):
                    result = self.bt.start_scan()

        self.assertTrue(result["ok"])
        self.assertTrue(result["scanning"])
        fake_proc.stdin.write.assert_called()
        self.assertTrue(self.bt.is_scanning())

        fake_proc.poll.return_value = None
        with patch.object(self.bt, "_run_session", return_value=self._completed("")):
            stop = self.bt.stop_scan()
        self.assertFalse(stop["scanning"])
        self.assertFalse(self.bt.is_scanning())


if __name__ == "__main__":
    unittest.main()
