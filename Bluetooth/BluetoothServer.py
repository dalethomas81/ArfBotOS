from __future__ import annotations

import atexit
import os

from flask import Flask, jsonify, render_template, request

from bluetoothctl_wrapper import BluetoothctlWrapper, validate_mac_address


BASE_DIR = os.path.abspath(os.path.dirname(__file__))
TEMPLATES_DIR = os.path.join(BASE_DIR, "templates")

app = Flask(__name__, template_folder=TEMPLATES_DIR)
app.config["JSON_SORT_KEYS"] = False

bt = BluetoothctlWrapper()
atexit.register(bt.stop_scan)


def json_error(message: str, status_code: int = 400):
    return jsonify({"ok": False, "message": message}), status_code


def get_mac_from_request():
    payload = request.get_json(silent=True) or {}
    mac = payload.get("mac") or request.form.get("mac") or request.args.get("mac")
    if not mac:
        return None, json_error("Missing 'mac' parameter")
    if not validate_mac_address(mac):
        return None, json_error("Invalid MAC address format")
    return mac, None


def truthy(value, default: bool = False) -> bool:
    if value is None:
        return default
    return str(value).lower() in {"1", "true", "yes", "on"}


@app.route("/")
def index():
    return render_template("index.html", title="Connect Controller")


@app.route("/api/status")
def api_status():
    return jsonify(bt.get_status())


@app.route("/api/devices")
def api_devices():
    enrich = truthy(request.args.get("enrich"), default=False)
    return jsonify(bt.get_discovered_devices(enrich=enrich))


@app.route("/api/paired")
def api_paired():
    enrich = truthy(request.args.get("enrich"), default=True)
    return jsonify(bt.get_paired_devices(enrich=enrich))


@app.route("/api/scan/start", methods=["POST"])
def api_scan_start():
    return jsonify(bt.start_scan())


@app.route("/api/scan/stop", methods=["POST"])
def api_scan_stop():
    return jsonify(bt.stop_scan())


@app.route("/api/power/on", methods=["POST"])
def api_power_on():
    return jsonify(bt.power_on())


@app.route("/api/power/off", methods=["POST"])
def api_power_off():
    return jsonify(bt.power_off())


@app.route("/api/pair", methods=["POST"])
def api_pair():
    mac, error = get_mac_from_request()
    if error:
        return error
    return jsonify(bt.pair(mac))


@app.route("/api/trust", methods=["POST"])
def api_trust():
    mac, error = get_mac_from_request()
    if error:
        return error
    return jsonify(bt.trust(mac))


@app.route("/api/connect", methods=["POST"])
def api_connect():
    mac, error = get_mac_from_request()
    if error:
        return error
    return jsonify(bt.connect(mac))


@app.route("/api/disconnect", methods=["POST"])
def api_disconnect():
    mac, error = get_mac_from_request()
    if error:
        return error
    return jsonify(bt.disconnect(mac))


@app.route("/api/remove", methods=["POST"])
def api_remove():
    mac, error = get_mac_from_request()
    if error:
        return error
    return jsonify(bt.remove(mac))


@app.route("/api/setup", methods=["POST"])
def api_setup():
    """One-click pair + trust + connect for a DualSense / gamepad."""
    mac, error = get_mac_from_request()
    if error:
        return error
    return jsonify(bt.setup_device(mac))


if __name__ == "__main__":
    host = os.environ.get("ARFBOTOS_BT_HOST", "0.0.0.0")
    port = int(os.environ.get("ARFBOTOS_BT_PORT", "50014"))
    debug = os.environ.get("ARFBOTOS_BT_DEBUG", "false").lower() in {"1", "true", "yes", "on"}
    app.run(host=host, port=port, debug=debug)
