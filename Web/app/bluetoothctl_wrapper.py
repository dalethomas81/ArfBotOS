from __future__ import annotations

import re
import shutil
import subprocess
import threading
import time
from pathlib import Path
from typing import Dict, List, Optional


MAC_PATTERN = re.compile(r"^([0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}$")
MAC_NAME_PATTERN = re.compile(r"^([0-9A-Fa-f]{2}[:\-]){5}[0-9A-Fa-f]{2}$")
RSSI_LINE_PATTERN = re.compile(
    r"Device\s+([0-9A-Fa-f:]{17}).*RSSI:(?:.*\((-?\d+)\)|\s*(-?\d+))",
    re.IGNORECASE,
)
ANSI_ESCAPE_PATTERN = re.compile(r"\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])")
MIN_RSSI_DBM = -80

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


def looks_like_mac_name(name: Optional[str], mac: Optional[str] = None) -> bool:
    """True when the advertised name is missing or is just the address."""
    if not name or not str(name).strip():
        return True
    cleaned = str(name).strip()
    if MAC_NAME_PATTERN.match(cleaned):
        return True
    if mac and cleaned.replace("-", ":").upper() == mac.replace("-", ":").upper():
        return True
    return False


def parse_rssi_value(value) -> Optional[int]:
    if value is None or value == "":
        return None
    text = str(value).strip()
    match = re.search(r"\((-?\d+)\)\s*$", text)
    if match:
        return int(match.group(1))
    try:
        parsed = int(text)
    except (TypeError, ValueError):
        return None
    if parsed > 0:
        return parsed - 256
    return parsed


class BluetoothctlWrapper:
    """Thin wrapper around bluetoothctl with a persistent scan session."""

    def __init__(self, executable: str = "bluetoothctl", timeout_s: int = 15):
        self.executable = executable
        self.timeout_s = timeout_s
        self._scan_lock = threading.Lock()
        self._scan_process: Optional[subprocess.Popen] = None
        self._scan_started_at: Optional[float] = None
        self._output_lock = threading.Lock()
        self._output_lines: List[str] = []
        self._rssi_by_mac: Dict[str, int] = {}

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

    def _run_cli(
        self,
        args: List[str],
        timeout_s: int | None = None,
        agent: str | None = None,
    ) -> Dict:
        """Run a one-shot `bluetoothctl -- <args>` command.

        Interactive stdin sessions often drop the first command while waiting
        to connect to bluetoothd, so Forget/pair/info must not use that path.
        """
        # bluetoothctl -- <cmd> without --timeout stays attached to the
        # event stream and never exits, which wedges the Flask thread.
        bt_timeout = timeout_s if timeout_s is not None else 2
        cmd = [self.executable, "--timeout", str(max(1, bt_timeout))]
        if agent:
            cmd.append("--agent=" + agent)
        cmd += ["--", *args]
        try:
            process = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=bt_timeout + 5,
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
                "message": f"bluetoothctl timed out after {bt_timeout} seconds",
                "stdout": self._clean_output(exc.stdout or ""),
                "stderr": self._clean_output(exc.stderr or ""),
                "returncode": None,
            }

        stdout = self._clean_output(process.stdout or "")
        stderr = self._clean_output(process.stderr or "")
        combined = f"{stdout}\n{stderr}".lower()
        failed = "failed" in combined or "not ready" in combined or "notavailable" in combined.replace(" ", "")
        return {
            "ok": process.returncode == 0 and not failed,
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
        rssi = parse_rssi_value(annotated.get("rssi"))
        if rssi is None:
            with self._output_lock:
                rssi = self._rssi_by_mac.get((annotated.get("mac") or "").upper())
        annotated["rssi"] = rssi
        return annotated

    def _keep_nearby(self, item: Dict, named_only: bool, min_rssi: Optional[int]) -> bool:
        if item.get("is_controller"):
            return True
        name = item.get("display_name") or item.get("alias") or item.get("name")
        if named_only and looks_like_mac_name(name, item.get("mac")):
            return False
        rssi = parse_rssi_value(item.get("rssi"))
        if min_rssi is not None and rssi is not None and rssi < min_rssi:
            return False
        return True

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

    def _start_reader(self, process: subprocess.Popen) -> None:
        stream = process.stdout
        if stream is None:
            return

        def loop() -> None:
            try:
                while True:
                    line = stream.readline()
                    if not line or not isinstance(line, str):
                        break
                    cleaned = self._clean_output(line).rstrip("\n")
                    if not cleaned:
                        continue
                    rssi_match = RSSI_LINE_PATTERN.search(cleaned)
                    with self._output_lock:
                        self._output_lines.append(cleaned)
                        if len(self._output_lines) > 400:
                            del self._output_lines[:-250]
                        if rssi_match:
                            mac = rssi_match.group(1).upper()
                            rssi = parse_rssi_value(rssi_match.group(2) or rssi_match.group(3))
                            if rssi is not None:
                                self._rssi_by_mac[mac] = rssi
            except Exception:
                return

        threading.Thread(target=loop, name="bluetoothctl-reader", daemon=True).start()

    def _session_command(self, command: str, wait_for: List[str], timeout_s: float = 20) -> Dict:
        """Send a command to the live bluetoothctl session and wait for output."""
        with self._scan_lock:
            process = self._scan_process
            if process is None or process.poll() is not None or process.stdin is None:
                return {
                    "ok": False,
                    "message": "Bluetooth session is not running. Start a scan first.",
                    "stdout": "",
                    "stderr": "",
                }
            with self._output_lock:
                start_idx = len(self._output_lines)
            try:
                process.stdin.write(command + "\n")
                process.stdin.flush()
            except BrokenPipeError:
                return {
                    "ok": False,
                    "message": "Bluetooth session ended",
                    "stdout": "",
                    "stderr": "",
                }

        deadline = time.time() + timeout_s
        blob = ""
        while time.time() < deadline:
            with self._output_lock:
                chunk = self._output_lines[start_idx:]
            blob = "\n".join(chunk)
            lower = blob.lower()
            if any(token.lower() in lower for token in wait_for):
                failed = "failed" in lower and "successful" not in lower and "succeeded" not in lower
                return {
                    "ok": not failed,
                    "message": "Command completed",
                    "stdout": blob,
                    "stderr": "",
                }
            time.sleep(0.15)
        return {
            "ok": False,
            "message": f"Timed out waiting for {command}",
            "stdout": blob,
            "stderr": "",
        }

    def get_device_info(self, mac: str) -> Dict:
        if not validate_mac_address(mac):
            return {"ok": False, "message": "Invalid MAC address format", "mac": mac}

        result = self._run_cli(["info", mac])
        info = {
            "mac": mac,
            "name": self._extract_info_field(result["stdout"], "Name"),
            "alias": self._extract_info_field(result["stdout"], "Alias"),
            "icon": self._extract_info_field(result["stdout"], "Icon"),
            "paired": self._to_bool(self._extract_info_field(result["stdout"], "Paired")),
            "bonded": self._to_bool(self._extract_info_field(result["stdout"], "Bonded")),
            "trusted": self._to_bool(self._extract_info_field(result["stdout"], "Trusted")),
            "connected": self._to_bool(self._extract_info_field(result["stdout"], "Connected")),
            "blocked": self._to_bool(self._extract_info_field(result["stdout"], "Blocked")),
            "rssi": self._extract_info_field(result["stdout"], "RSSI"),
        }
        if "not available" in (result.get("stdout") or "").lower():
            result["ok"] = False
            info["paired"] = False
            info["bonded"] = False
            info["trusted"] = False
            info["connected"] = False
        result["device"] = self._annotate_device(info)
        return result

    def get_status(self) -> Dict:
        result = self._run_cli(["show"])
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

    def _list_devices(self, command: str, enrich: bool, nearby_filter: bool = False) -> Dict:
        result = self._run_cli(command.split())
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

        if nearby_filter:
            before = len(enriched)
            enriched = [
                item for item in enriched
                if self._keep_nearby(item, named_only=True, min_rssi=MIN_RSSI_DBM)
            ]
            result["filtered"] = before - len(enriched)

        def sort_key(item: Dict):
            rssi = parse_rssi_value(item.get("rssi"))
            rssi_rank = -(rssi if rssi is not None else -999)
            return (
                not item.get("is_controller", False),
                rssi_rank,
                (item.get("display_name") or "").lower(),
            )

        enriched.sort(key=sort_key)
        result["devices"] = enriched
        result["scanning"] = self.is_scanning()
        return result

    def get_discovered_devices(self, enrich: bool = False, nearby_filter: bool = True) -> Dict:
        return self._list_devices("devices", enrich=enrich, nearby_filter=nearby_filter)

    def get_paired_devices(self, enrich: bool = True) -> Dict:
        return self._list_devices("devices Paired", enrich=enrich, nearby_filter=False)

    def _rfkill_unblock(self) -> Dict:
        """Lite images often soft-block hci0; power on then fails until unblocked."""
        try:
            process = subprocess.run(
                ["rfkill", "unblock", "bluetooth"],
                capture_output=True,
                text=True,
                timeout=10,
                check=False,
            )
        except FileNotFoundError:
            return {
                "ok": False,
                "message": "rfkill was not found on this system",
                "stdout": "",
                "stderr": "",
            }
        except subprocess.TimeoutExpired:
            return {
                "ok": False,
                "message": "rfkill timed out",
                "stdout": "",
                "stderr": "",
            }
        return {
            "ok": process.returncode == 0,
            "message": (
                "rfkill unblock bluetooth completed"
                if process.returncode == 0
                else "rfkill unblock bluetooth failed"
            ),
            "stdout": process.stdout or "",
            "stderr": process.stderr or "",
        }

    def ensure_ready(self) -> Dict:
        """Unblock rfkill, power on the adapter, and make it pairable."""
        rfkill = self._rfkill_unblock()
        # Do not run `agent on` / `default-agent` here: those commands keep
        # bluetoothctl attached even with --timeout, and they wedge Flask.
        result = self._run_cli(["power", "on"], timeout_s=3)
        status = self.get_status()
        adapter = status.get("adapter", {})
        powered = bool(adapter.get("powered"))
        result["adapter"] = adapter
        result["rfkill"] = rfkill
        result["ok"] = powered
        if powered:
            result["message"] = "Bluetooth adapter is powered on and pairable"
        else:
            power_state = adapter.get("power_state") or "unknown"
            hint = (
                " If PowerState is off-blocked, run: sudo rfkill unblock bluetooth"
            )
            if not rfkill.get("ok"):
                hint = f" ({rfkill.get('message')})" + hint
            result["message"] = (
                f"Could not power on the Bluetooth adapter (PowerState: {power_state})."
                + hint
            )
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
        """Start a long-lived interactive bluetoothctl session (like a manual shell).

        Manual pairing works because agent/scan/pair/trust all happen in one
        process that is not quit after pair. We keep that process open here.
        A reader thread drains stdout so the pipe cannot deadlock Flask.
        """
        with self._scan_lock:
            if self._scan_process is not None and self._scan_process.poll() is None:
                return {
                    "ok": True,
                    "message": "Scan already running",
                    "scanning": True,
                    "scan_started_at": self._scan_started_at,
                }

            try:
                process = subprocess.Popen(
                    [self.executable, "--agent=NoInputNoOutput"],
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

            self._start_reader(process)
            try:
                if process.stdin is not None:
                    process.stdin.write("agent NoInputNoOutput\ndefault-agent\npairable on\nscan on\n")
                    process.stdin.flush()
            except BrokenPipeError:
                process.kill()
                return {
                    "ok": False,
                    "message": "Failed to start bluetoothctl session",
                    "scanning": False,
                }

            time.sleep(0.4)
            if process.poll() is not None:
                return {
                    "ok": False,
                    "message": "Scan process exited unexpectedly",
                    "scanning": False,
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

        if process is not None:
            try:
                if process.poll() is None and process.stdin is not None:
                    try:
                        process.stdin.write("scan off\nquit\n")
                        process.stdin.flush()
                    except BrokenPipeError:
                        pass
                    process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    pass

        result = self._run_cli(["scan", "off"], timeout_s=2)
        result["scanning"] = False
        result["scan_started_at"] = started_at
        result["ok"] = True
        result["message"] = "Scan stopped"
        return result

    def _simple_action(self, command: str, mac: str, action_name: str) -> Dict:
        if not validate_mac_address(mac):
            return {"ok": False, "message": "Invalid MAC address format", "mac": mac}

        timeout_s = 30

        if command in {"pair", "connect", "trust"}:
            self.ensure_ready()

        args = [command, mac]
        extra_stdout = ""
        agent = None
        if command == "pair":
            self._run_cli(["pairable", "on"], timeout_s=2)
            timeout_s = 15
            agent = "NoInputNoOutput"

        result = self._run_cli(args, timeout_s=timeout_s, agent=agent)
        result["stdout"] = extra_stdout + (result.get("stdout") or "")
        result["mac"] = mac
        result["action"] = action_name
        result["message"] = self._build_action_message(result, action_name, mac)
        result["device"] = self.get_device_info(mac).get("device", {"mac": mac})
        if "failed" in result["message"].lower() or "not ready" in (result.get("stdout") or "").lower():
            result["ok"] = False
        elif any(
            token in (result.get("stdout") or "").lower()
            for token in ("successful", "succeeded", "device has been removed")
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
        if "successful disconnected" in stdout or (
            "disconnected" in stdout and action_name == "disconnect"
        ):
            return f"Disconnected {mac}"
        if "device has been removed" in stdout or (
            "removed" in stdout and action_name == "remove"
        ):
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

    def _is_bonded(self, mac: str) -> bool:
        mac_u = mac.strip().upper()
        paired = self.get_paired_devices(enrich=False)
        for device in paired.get("devices") or []:
            if (device.get("mac") or "").upper() == mac_u:
                return True
        info = self.get_device_info(mac).get("device") or {}
        return bool(info.get("paired") or info.get("bonded"))

    def _bluez_adapter_dir(self) -> Optional[Path]:
        adapter = (self.get_status().get("adapter") or {}).get("controller")
        if not adapter:
            return None
        path = Path("/var/lib/bluetooth") / adapter
        return path if path.is_dir() else None

    def _remove_bond_files(self, mac: str) -> Dict:
        base = self._bluez_adapter_dir()
        if base is None:
            return {"ok": False, "message": "Bluetooth adapter store not found", "removed": []}
        removed = []
        try:
            device_dir = base / mac
            cache_file = base / "cache" / mac
            if device_dir.is_dir():
                shutil.rmtree(device_dir)
                removed.append(str(device_dir))
            if cache_file.is_file():
                cache_file.unlink()
                removed.append(str(cache_file))
        except OSError as exc:
            return {"ok": False, "message": str(exc), "removed": removed}
        return {
            "ok": bool(removed),
            "message": "Bond files removed" if removed else "No bond files found",
            "removed": removed,
        }

    def _restart_bluetooth(self) -> Dict:
        try:
            process = subprocess.run(
                ["systemctl", "restart", "bluetooth"],
                capture_output=True,
                text=True,
                timeout=20,
                check=False,
            )
        except FileNotFoundError:
            return {"ok": False, "message": "systemctl was not found on this system"}
        except subprocess.TimeoutExpired:
            return {"ok": False, "message": "systemctl restart bluetooth timed out"}
        return {
            "ok": process.returncode == 0,
            "message": "bluetooth service restarted" if process.returncode == 0 else "Failed to restart bluetooth",
            "stdout": process.stdout or "",
            "stderr": process.stderr or "",
        }

    def remove(self, mac: str) -> Dict:
        """Forget a device. Verifies the bond is gone; does not trust bluetoothctl's exit code."""
        if not validate_mac_address(mac):
            return {"ok": False, "message": "Invalid MAC address format", "mac": mac}

        self.ensure_ready()
        cli = self._run_cli(["remove", mac])
        if not self._is_bonded(mac):
            return {
                "ok": True,
                "message": f"Removed {mac}",
                "mac": mac,
                "action": "remove",
                "stdout": cli.get("stdout", ""),
                "device": self.get_device_info(mac).get("device", {"mac": mac}),
            }

        disk = self._remove_bond_files(mac)
        restarted = self._restart_bluetooth()
        time.sleep(1.5)
        gone = not self._is_bonded(mac)
        adapter = (self.get_status().get("adapter") or {})
        message = f"Removed {mac}" if gone else (
            f"Could not forget {mac}. The adapter is not ready "
            f"(Powered: {adapter.get('powered')}, PowerState: {adapter.get('power_state')}). "
            "Reboot the Pi, tap Power On, then try Forget again."
        )
        if gone and not adapter.get("powered"):
            message = (
                f"Removed {mac}. Bluetooth is still powered off "
                "(chip may need a reboot before you can pair again)."
            )
        return {
            "ok": gone,
            "message": message,
            "mac": mac,
            "action": "remove",
            "stdout": cli.get("stdout", ""),
            "fallback": {"disk": disk, "restart": restarted},
            "device": self.get_device_info(mac).get("device", {"mac": mac}),
        }

    def setup_device(self, mac: str) -> Dict:
        """Pair + trust in the live bluetoothctl session, matching manual pairing.

        Manual flow is: agent on, default-agent, pairable on, scan on, pair, trust.
        It does not spawn a new bluetoothctl, quit after pair, or call connect.
        """
        if not validate_mac_address(mac):
            return {"ok": False, "message": "Invalid MAC address format", "mac": mac}

        session = self.start_scan()
        if not session.get("ok") and not self.is_scanning():
            return {
                "ok": False,
                "message": session.get("message", "Could not start bluetoothctl session"),
                "mac": mac,
                "action": "setup",
            }

        pair = self._session_command(
            f"pair {mac}",
            ["pairing successful", "already exists", "failed"],
            timeout_s=25,
        )
        trust = self._session_command(
            f"trust {mac}",
            ["trust succeeded", "failed"],
            timeout_s=8,
        )
        time.sleep(0.5)
        device = self.get_device_info(mac).get("device") or {"mac": mac}
        connected = bool(device.get("connected"))
        paired = bool(device.get("paired") or device.get("bonded"))
        trusted = bool(device.get("trusted"))

        result = {
            "stdout": "\n".join([pair.get("stdout") or "", trust.get("stdout") or ""]),
            "stderr": "",
            "mac": mac,
            "action": "setup",
            "device": device,
            "ok": connected or (paired and trusted),
        }
        if connected:
            result["message"] = f"Controller ready: connected to {device.get('display_name') or mac}"
        elif paired and trusted:
            result["message"] = (
                f"Paired and trusted {device.get('display_name') or mac}. "
                "If it is not connected, wake the device and use Connect."
            )
        else:
            stdout = (result.get("stdout") or "").lower()
            if "failed" in stdout:
                result["message"] = (
                    f"Setup failed for {mac}. Put the device in pairing mode and try again."
                )
            else:
                result["message"] = f"Setup did not finish for {mac}"
        return result
