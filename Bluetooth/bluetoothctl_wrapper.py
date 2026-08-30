from __future__ import annotations

import re
import subprocess
import threading
import time
from typing import Dict, List, Optional


MAC_PATTERN = re.compile(r"^([0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}$")
ANSI_ESCAPE_PATTERN = re.compile(r"\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])")

# Names commonly advertised by DualSense / DualShock / generic gamepads.
CONTROLLER_NAME_HINTS = (
    "dualsense",
    "dualshock",
    "wireless controller",
    "sony interactive",
    "gamepad",
    "xbox",
    "8bitdo",
)


def validate_mac_address(mac: str) -> bool:
    return bool(MAC_PATTERN.match(mac.strip()))


def looks_like_controller(name: Optional[str], icon: Optional[str] = None) -> bool:
    haystack = f"{name or ''} {icon or ''}".lower()
    return any(hint in haystack for hint in CONTROLLER_NAME_HINTS)


class BluetoothctlWrapper:
    """Thin wrapper around bluetoothctl with a persistent scan session."""

    def __init__(self, executable: str = "bluetoothctl", timeout_s: int = 15):
        self.executable = executable
        self.timeout_s = timeout_s
        self._scan_lock = threading.Lock()
        self._scan_process: Optional[subprocess.Popen] = None
        self._scan_started_at: Optional[float] = None

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
                "stdout": self._clean_output(exc.stdout or ""),
                "stderr": self._clean_output(exc.stderr or ""),
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

    def _annotate_device(self, device: Dict) -> Dict:
        annotated = dict(device)
        display_name = annotated.get("alias") or annotated.get("name") or "Unknown device"
        annotated["display_name"] = display_name
        annotated["is_controller"] = looks_like_controller(
            display_name,
            annotated.get("icon"),
        ) or looks_like_controller(annotated.get("name"), annotated.get("icon"))
        return annotated

    def is_scanning(self) -> bool:
        with self._scan_lock:
            process = self._scan_process
            if process is None:
                return False
            if process.poll() is not None:
                self._scan_process = None
                self._scan_started_at = None
                return False
            return True

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
        result["device"] = self._annotate_device(info)
        return result

    def get_status(self) -> Dict:
        result = self._run_session(["show"])
        discovering = self._to_bool(self._extract_info_field(result["stdout"], "Discovering"))
        scanning = self.is_scanning() or bool(discovering)
        result["adapter"] = {
            "controller": self._extract_controller(result["stdout"]),
            "name": self._extract_info_field(result["stdout"], "Name"),
            "alias": self._extract_info_field(result["stdout"], "Alias"),
            "powered": self._to_bool(self._extract_info_field(result["stdout"], "Powered")),
            "discoverable": self._to_bool(self._extract_info_field(result["stdout"], "Discoverable")),
            "pairable": self._to_bool(self._extract_info_field(result["stdout"], "Pairable")),
            "discovering": discovering,
            "scanning": scanning,
            "power_state": self._extract_info_field(result["stdout"], "PowerState"),
            "scan_started_at": self._scan_started_at,
        }
        result["scanning"] = scanning
        return result

    def _list_devices(self, command: str, enrich: bool) -> Dict:
        result = self._run_session([command])
        devices = self._extract_devices(result["stdout"])
        enriched = []
        for device in devices:
            if enrich:
                info = self.get_device_info(device["mac"])
                merged = dict(device)
                merged.update(info.get("device", {}))
                enriched.append(self._annotate_device(merged))
            else:
                enriched.append(self._annotate_device(device))

        # Controllers first, then alphabetical by display name.
        enriched.sort(
            key=lambda item: (
                not item.get("is_controller", False),
                (item.get("display_name") or "").lower(),
            )
        )
        result["devices"] = enriched
        result["scanning"] = self.is_scanning()
        return result

    def get_discovered_devices(self, enrich: bool = False) -> Dict:
        # Discovery list refreshes often; skip per-device info by default.
        return self._list_devices("devices", enrich=enrich)

    def get_paired_devices(self, enrich: bool = True) -> Dict:
        return self._list_devices("devices Paired", enrich=enrich)

    def ensure_ready(self) -> Dict:
        """Power on the adapter and make it pairable for incoming controllers."""
        result = self._run_session(
            ["power on", "agent on", "default-agent", "pairable on"],
            timeout_s=20,
        )
        status = self.get_status()
        adapter = status.get("adapter", {})
        powered = bool(adapter.get("powered"))
        result["adapter"] = adapter
        result["ok"] = powered
        if powered:
            result["message"] = "Bluetooth adapter is powered on and pairable"
        else:
            result["message"] = "Could not power on the Bluetooth adapter"
        return result

    def power_on(self) -> Dict:
        return self.ensure_ready()

    def power_off(self) -> Dict:
        self.stop_scan()
        result = self._run_session(["power off"])
        status = self.get_status()
        result["adapter"] = status.get("adapter", {})
        result["message"] = "Bluetooth adapter power off requested"
        return result

    def start_scan(self, prepare: bool = True) -> Dict:
        """Start a long-lived bluetoothctl scan session.

        The old MVP sent `scan on` then immediately `quit`, which stopped discovery
        before any useful devices appeared. This keeps a dedicated process alive.
        """
        with self._scan_lock:
            if self._scan_process is not None and self._scan_process.poll() is None:
                return {
                    "ok": True,
                    "message": "Scan already running",
                    "scanning": True,
                    "scan_started_at": self._scan_started_at,
                }

            if prepare:
                prep = self.ensure_ready()
                if not prep.get("ok"):
                    prep["scanning"] = False
                    return prep

            try:
                process = subprocess.Popen(
                    [self.executable],
                    stdin=subprocess.PIPE,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    bufsize=1,
                )
            except FileNotFoundError:
                return {
                    "ok": False,
                    "message": "bluetoothctl was not found on this system",
                    "scanning": False,
                }

            try:
                assert process.stdin is not None
                process.stdin.write("agent on\ndefault-agent\npairable on\nscan on\n")
                process.stdin.flush()
            except BrokenPipeError:
                process.kill()
                return {
                    "ok": False,
                    "message": "Failed to start bluetoothctl scan session",
                    "scanning": False,
                }

            # Give bluez a moment to flip Discovering=yes.
            time.sleep(0.4)
            if process.poll() is not None:
                output = ""
                if process.stdout is not None:
                    output = self._clean_output(process.stdout.read() or "")
                return {
                    "ok": False,
                    "message": "Scan process exited unexpectedly",
                    "scanning": False,
                    "stdout": output,
                }

            self._scan_process = process
            self._scan_started_at = time.time()
            return {
                "ok": True,
                "message": "Scanning for nearby Bluetooth devices",
                "scanning": True,
                "scan_started_at": self._scan_started_at,
            }

    def stop_scan(self) -> Dict:
        with self._scan_lock:
            process = self._scan_process
            self._scan_process = None
            started_at = self._scan_started_at
            self._scan_started_at = None

        if process is None:
            # Still ask bluez to stop in case another client started discovery.
            result = self._run_session(["scan off"])
            result["scanning"] = False
            result["message"] = "Scan stop requested"
            return result

        try:
            if process.poll() is None and process.stdin is not None:
                process.stdin.write("scan off\nquit\n")
                process.stdin.flush()
                process.wait(timeout=5)
        except (BrokenPipeError, subprocess.TimeoutExpired):
            process.kill()
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                pass

        return {
            "ok": True,
            "message": "Scan stopped",
            "scanning": False,
            "scan_started_at": started_at,
        }

    def _simple_action(self, command: str, mac: str, action_name: str) -> Dict:
        if not validate_mac_address(mac):
            return {"ok": False, "message": "Invalid MAC address format", "mac": mac}

        commands = [f"{command} {mac}"]
        timeout_s = 30

        if command == "pair":
            commands = ["agent on", "default-agent", "pairable on", f"{command} {mac}"]
            timeout_s = 90

        result = self._run_session(commands, timeout_s=timeout_s)
        result["mac"] = mac
        result["action"] = action_name
        result["message"] = self._build_action_message(result, action_name, mac)
        result["device"] = self.get_device_info(mac).get("device", {"mac": mac})
        if "failed" in result["message"].lower():
            result["ok"] = False
        elif any(
            token in (result.get("stdout") or "").lower()
            for token in ("successful", "succeeded")
        ):
            result["ok"] = True
        return result

    def _build_action_message(self, result: Dict, action_name: str, mac: str) -> str:
        stdout = (result.get("stdout") or "").lower()
        stderr = (result.get("stderr") or "").lower()

        if "already exists" in stdout and action_name == "pair":
            return f"Already paired with {mac}"
        if "pairing successful" in stdout:
            return f"Paired with {mac}"
        if "trust succeeded" in stdout:
            return f"Trusted {mac}"
        if "connection successful" in stdout:
            return f"Connected to {mac}"
        if "successful disconnected" in stdout or "disconnected" in stdout and action_name == "disconnect":
            return f"Disconnected {mac}"
        if "device has been removed" in stdout or ("removed" in stdout and action_name == "remove"):
            return f"Removed {mac}"
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

    def setup_device(self, mac: str) -> Dict:
        """One-shot pair + trust + connect flow for controllers."""
        if not validate_mac_address(mac):
            return {"ok": False, "message": "Invalid MAC address format", "mac": mac}

        prep = self.ensure_ready()
        if not prep.get("ok"):
            return {
                "ok": False,
                "message": prep.get("message", "Bluetooth adapter is not ready"),
                "mac": mac,
                "action": "setup",
                "steps": [prep],
            }

        commands = [
            "agent on",
            "default-agent",
            "pairable on",
            f"pair {mac}",
            f"trust {mac}",
            f"connect {mac}",
        ]
        result = self._run_session(commands, timeout_s=120)
        device = self.get_device_info(mac).get("device", {"mac": mac})
        connected = bool(device.get("connected"))
        paired = bool(device.get("paired"))
        trusted = bool(device.get("trusted"))

        result["mac"] = mac
        result["action"] = "setup"
        result["device"] = device
        result["ok"] = connected or (paired and trusted)
        if connected:
            result["message"] = f"Controller ready: connected to {device.get('display_name') or mac}"
        elif paired and trusted:
            result["message"] = (
                f"Paired and trusted {device.get('display_name') or mac}, "
                "but it is not connected yet. Press the PS button and try Connect."
            )
        else:
            stdout = (result.get("stdout") or "").lower()
            if "failed" in stdout:
                result["message"] = (
                    f"Setup failed for {mac}. Put the controller in pairing mode "
                    "(Create + PS until the light bar blinks) and try again."
                )
            else:
                result["message"] = f"Setup did not finish for {mac}"
        return result
