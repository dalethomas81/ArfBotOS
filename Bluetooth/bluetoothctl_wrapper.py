from __future__ import annotations

import re
import subprocess
from typing import Dict, List


MAC_PATTERN = re.compile(r"^([0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}$")
ANSI_ESCAPE_PATTERN = re.compile(r"\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])")


def validate_mac_address(mac: str) -> bool:
    return bool(MAC_PATTERN.match(mac.strip()))


class BluetoothctlWrapper:
    def __init__(self, executable: str = "bluetoothctl", timeout_s: int = 15):
        self.executable = executable
        self.timeout_s = timeout_s

    def _run_session(self, commands: List[str], timeout_s: int | None = None) -> Dict:
        session = list(commands) + ["quit"]
        timeout = timeout_s if timeout_s is not None else self.timeout_s
        try:
            process = subprocess.run(
                [self.executable],
                input="\n".join(session) + "\n",
                capture_output=True,
                text=True,
                timeout=timeout,
                check=False,
            )
        except FileNotFoundError:
            return {
                "ok": False,
                "message": "bluetoothctl was not found on this system",
                "stdout": "",
                "stderr": "",
                "returncode": None,
            }
        except subprocess.TimeoutExpired as exc:
            return {
                "ok": False,
                "message": f"bluetoothctl timed out after {timeout} seconds",
                "stdout": exc.stdout or "",
                "stderr": exc.stderr or "",
                "returncode": None,
            }

        stdout = self._clean_output(process.stdout or "")
        stderr = self._clean_output(process.stderr or "")
        return {
            "ok": process.returncode == 0,
            "message": "Command completed" if process.returncode == 0 else "bluetoothctl returned a non-zero exit code",
            "stdout": stdout,
            "stderr": stderr,
            "returncode": process.returncode,
        }

    def _clean_output(self, output: str) -> str:
        cleaned = ANSI_ESCAPE_PATTERN.sub("", output)
        cleaned = cleaned.replace("\r", "")
        return cleaned

    def _extract_devices(self, output: str) -> List[Dict]:
        devices = []
        seen = set()
        for line in output.splitlines():
            line = line.strip()
            if not line:
                continue
            match = re.match(r"^Device\s+([0-9A-Fa-f:]{17})\s+(.+)$", line)
            if not match:
                continue
            mac = match.group(1)
            name = match.group(2).strip()
            if mac in seen:
                continue
            seen.add(mac)
            devices.append({"mac": mac, "name": name})
        return devices

    def _extract_info_field(self, output: str, field_name: str):
        pattern = re.compile(rf"^\s*{re.escape(field_name)}:\s*(.+?)\s*$", re.MULTILINE)
        match = pattern.search(output)
        return match.group(1).strip() if match else None

    def _extract_controller(self, output: str):
        match = re.search(r"^Controller\s+([0-9A-Fa-f:]{17})\b", output, re.MULTILINE)
        return match.group(1) if match else None

    def _to_bool(self, value):
        if value is None:
            return None
        return value.lower() == "yes"

    def get_device_info(self, mac: str) -> Dict:
        if not validate_mac_address(mac):
            return {"ok": False, "message": "Invalid MAC address format", "mac": mac}

        result = self._run_session([f"info {mac}"])
        info = {
            "mac": mac,
            "name": self._extract_info_field(result["stdout"], "Name"),
            "alias": self._extract_info_field(result["stdout"], "Alias"),
            "icon": self._extract_info_field(result["stdout"], "Icon"),
            "paired": self._to_bool(self._extract_info_field(result["stdout"], "Paired")),
            "trusted": self._to_bool(self._extract_info_field(result["stdout"], "Trusted")),
            "connected": self._to_bool(self._extract_info_field(result["stdout"], "Connected")),
            "blocked": self._to_bool(self._extract_info_field(result["stdout"], "Blocked")),
            "rssi": self._extract_info_field(result["stdout"], "RSSI"),
        }
        result["device"] = info
        return result

    def get_status(self) -> Dict:
        result = self._run_session(["show"])
        result["adapter"] = {
            "controller": self._extract_controller(result["stdout"]),
            "name": self._extract_info_field(result["stdout"], "Name"),
            "alias": self._extract_info_field(result["stdout"], "Alias"),
            "powered": self._to_bool(self._extract_info_field(result["stdout"], "Powered")),
            "discoverable": self._to_bool(self._extract_info_field(result["stdout"], "Discoverable")),
            "pairable": self._to_bool(self._extract_info_field(result["stdout"], "Pairable")),
            "discovering": self._to_bool(self._extract_info_field(result["stdout"], "Discovering")),
            "power_state": self._extract_info_field(result["stdout"], "PowerState"),
        }
        return result

    def get_discovered_devices(self) -> Dict:
        result = self._run_session(["devices"])
        devices = self._extract_devices(result["stdout"])
        enriched = []
        for device in devices:
            info = self.get_device_info(device["mac"])
            merged = dict(device)
            merged.update(info.get("device", {}))
            enriched.append(merged)
        result["devices"] = enriched
        return result

    def get_paired_devices(self) -> Dict:
        result = self._run_session(["devices Paired"])
        devices = self._extract_devices(result["stdout"])
        enriched = []
        for device in devices:
            info = self.get_device_info(device["mac"])
            merged = dict(device)
            merged.update(info.get("device", {}))
            enriched.append(merged)
        result["devices"] = enriched
        return result

    def start_scan(self) -> Dict:
        result = self._run_session(["scan on"], timeout_s=10)
        result["scanning"] = True
        result["message"] = "Scan requested"
        return result

    def stop_scan(self) -> Dict:
        result = self._run_session(["scan off"])
        result["scanning"] = False
        result["message"] = "Scan stop requested"
        return result

    def _simple_action(self, command: str, mac: str, action_name: str) -> Dict:
        if not validate_mac_address(mac):
            return {"ok": False, "message": "Invalid MAC address format", "mac": mac}

        commands = [f"{command} {mac}"]
        timeout_s = 30

        if command == "pair":
            commands = ["agent on", "default-agent", f"{command} {mac}"]
            timeout_s = 90

        result = self._run_session(commands, timeout_s=timeout_s)
        result["mac"] = mac
        result["action"] = action_name
        result["message"] = self._build_action_message(result, action_name, mac)
        result["device"] = self.get_device_info(mac).get("device", {"mac": mac})
        return result

    def _build_action_message(self, result: Dict, action_name: str, mac: str) -> str:
        stdout = (result.get("stdout") or "").lower()
        stderr = (result.get("stderr") or "").lower()

        if "pairing successful" in stdout:
            return f"{action_name.capitalize()} succeeded for {mac}"
        if "trust succeeded" in stdout:
            return f"{action_name.capitalize()} succeeded for {mac}"
        if "connection successful" in stdout:
            return f"{action_name.capitalize()} succeeded for {mac}"
        if "failed" in stdout or "failed" in stderr:
            return f"{action_name.capitalize()} failed for {mac}"
        if "successful" in stdout or "succeeded" in stdout:
            return f"{action_name.capitalize()} succeeded for {mac}"
        if result.get("ok"):
            return f"{action_name.capitalize()} requested for {mac}"
        return f"{action_name.capitalize()} failed for {mac}"

    def pair(self, mac: str) -> Dict:
        return self._simple_action("pair", mac, "pair")

    def trust(self, mac: str) -> Dict:
        return self._simple_action("trust", mac, "trust")

    def connect(self, mac: str) -> Dict:
        return self._simple_action("connect", mac, "connect")

    def disconnect(self, mac: str) -> Dict:
        return self._simple_action("disconnect", mac, "disconnect")

    def remove(self, mac: str) -> Dict:
        return self._simple_action("remove", mac, "remove")
