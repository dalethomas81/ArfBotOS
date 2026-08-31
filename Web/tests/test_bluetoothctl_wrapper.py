from __future__ import annotations

import sys
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from app.bluetoothctl_wrapper import (  # noqa: E402
    BluetoothctlWrapper,
    looks_like_controller,
    looks_like_mac_name,
    parse_rssi_value,
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
Device AA:AA:AA:AA:AA:AA AA-AA-AA-AA-AA-AA
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


class HelperTests(unittest.TestCase):
    def test_validate_mac(self):
        self.assertTrue(validate_mac_address("AA:BB:CC:DD:EE:FF"))
        self.assertFalse(validate_mac_address("not-a-mac"))

    def test_controller_hints(self):
        self.assertTrue(looks_like_controller("DualSense Wireless Controller", "input-gaming"))
        self.assertTrue(looks_like_controller("Wireless Controller"))
        self.assertFalse(looks_like_controller("Dale's iPhone", "phone"))

    def test_mac_like_names(self):
        self.assertTrue(looks_like_mac_name("AA-AA-AA-AA-AA-AA", "AA:AA:AA:AA:AA:AA"))
        self.assertTrue(looks_like_mac_name("aa:aa:aa:aa:aa:aa"))
        self.assertTrue(looks_like_mac_name("", "AA:AA:AA:AA:AA:AA"))
        self.assertFalse(looks_like_mac_name("Wireless Controller", "D0:BC:C1:BE:5C:FA"))

    def test_parse_rssi(self):
        self.assertEqual(parse_rssi_value("-72"), -72)
        self.assertEqual(parse_rssi_value("0xffffffb8 (-72)"), -72)
        self.assertIsNone(parse_rssi_value(None))


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
        with patch.object(self.bt, "_run_cli", return_value=self._completed(SHOW_OUTPUT)):
            result = self.bt.get_status()
        adapter = result["adapter"]
        self.assertEqual(adapter["controller"], "AA:BB:CC:DD:EE:FF")
        self.assertTrue(adapter["powered"])
        self.assertTrue(adapter["pairable"])
        self.assertFalse(adapter["discovering"])

    def test_discovered_devices_sort_controllers_first_without_enrich(self):
        with patch.object(self.bt, "_run_cli", return_value=self._completed(DEVICES_OUTPUT)):
            result = self.bt.get_discovered_devices(enrich=False)
        names = [d["name"] for d in result["devices"]]
        self.assertEqual(names[0], "DualSense Wireless Controller")
        self.assertTrue(result["devices"][0]["is_controller"])
        self.assertIn("Dale's iPhone", names)
        self.assertFalse(any(d["name"].startswith("AA-") for d in result["devices"]))

    def test_setup_device_pairs_and_trusts_in_session_without_connect(self):
        commands = []

        def session_command(command, wait_for, timeout_s=20):
            commands.append(command)
            if command.startswith("pair"):
                return {"ok": True, "stdout": "Pairing successful"}
            return {"ok": True, "stdout": "Trust succeeded"}

        with patch.object(self.bt, "start_scan", return_value={"ok": True, "scanning": True}):
            with patch.object(self.bt, "is_scanning", return_value=True):
                with patch.object(self.bt, "_session_command", side_effect=session_command):
                    with patch.object(self.bt, "get_device_info", return_value={
                        "device": {
                            "mac": "11:22:33:44:55:66",
                            "display_name": "DualSense Wireless Controller",
                            "paired": True,
                            "trusted": True,
                            "connected": True,
                        }
                    }):
                        with patch("app.bluetoothctl_wrapper.time.sleep", return_value=None):
                            result = self.bt.setup_device("11:22:33:44:55:66")

        self.assertTrue(result["ok"])
        self.assertIn("connected", result["message"].lower())
        self.assertEqual(commands[0], "pair 11:22:33:44:55:66")
        self.assertEqual(commands[1], "trust 11:22:33:44:55:66")
        self.assertFalse(any(cmd.startswith("connect") for cmd in commands))

    def test_ensure_ready_unblocks_rfkill_then_powers_on(self):
        with patch.object(self.bt, "_rfkill_unblock", return_value={"ok": True, "message": "unblocked"}) as rfkill:
            with patch.object(self.bt, "_run_cli", return_value=self._completed("Changing power on succeeded")):
                with patch.object(self.bt, "get_status", return_value={
                    "adapter": {"powered": True, "power_state": "on"}
                }):
                    result = self.bt.ensure_ready()
        rfkill.assert_called_once()
        self.assertTrue(result["ok"])
        self.assertIn("powered on", result["message"].lower())

    def test_remove_succeeds_when_unpaired(self):
        with patch.object(self.bt, "ensure_ready", return_value={"ok": True}):
            with patch.object(self.bt, "_run_cli", return_value=self._completed("Device has been removed")):
                with patch.object(self.bt, "_is_bonded", return_value=False):
                    with patch.object(self.bt, "get_device_info", return_value={
                        "device": {"mac": "11:22:33:44:55:66", "paired": False}
                    }):
                        result = self.bt.remove("11:22:33:44:55:66")
        self.assertTrue(result["ok"])
        self.assertIn("removed", result["message"].lower())

    def test_remove_falls_back_to_bond_files_when_cli_not_ready(self):
        with patch.object(self.bt, "ensure_ready", return_value={"ok": True}):
            with patch.object(self.bt, "_run_cli", return_value=self._completed(
                "Failed to remove device: org.bluez.Error.NotReady"
            )):
                with patch.object(self.bt, "_is_bonded", side_effect=[True, False]):
                    with patch.object(self.bt, "_remove_bond_files", return_value={"ok": True, "removed": ["/tmp/x"]}) as disk:
                        with patch.object(self.bt, "_restart_bluetooth", return_value={"ok": True}):
                            with patch.object(self.bt, "get_status", return_value={"adapter": {"powered": False}}):
                                with patch.object(self.bt, "get_device_info", return_value={
                                    "device": {"mac": "11:22:33:44:55:66", "paired": False}
                                }):
                                    with patch("app.bluetoothctl_wrapper.time.sleep", return_value=None):
                                        result = self.bt.remove("11:22:33:44:55:66")
        disk.assert_called_once()
        self.assertTrue(result["ok"])
        self.assertIn("removed", result["message"].lower())

    def test_start_scan_keeps_process_alive(self):
        fake_proc = MagicMock()
        fake_proc.poll.return_value = None
        fake_proc.stdin = MagicMock()
        fake_proc.stdout = None

        with patch.object(self.bt, "ensure_ready", return_value={"ok": True, "message": "ready"}):
            with patch("app.bluetoothctl_wrapper.subprocess.Popen", return_value=fake_proc):
                with patch("app.bluetoothctl_wrapper.time.sleep", return_value=None):
                    result = self.bt.start_scan()

        self.assertTrue(result["ok"])
        self.assertTrue(result["scanning"])
        self.assertTrue(self.bt.is_scanning())

        fake_proc.poll.return_value = None
        with patch.object(self.bt, "_run_cli", return_value=self._completed("")):
            stop = self.bt.stop_scan()
        self.assertFalse(stop["scanning"])
        self.assertFalse(self.bt.is_scanning())


if __name__ == "__main__":
    unittest.main()
