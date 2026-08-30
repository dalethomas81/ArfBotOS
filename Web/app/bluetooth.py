from __future__ import annotations

import atexit

from flask import Blueprint, jsonify, render_template, request

from .bluetoothctl_wrapper import BluetoothctlWrapper, validate_mac_address

bp = Blueprint("bluetooth", __name__, url_prefix="/bluetooth")

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


@bp.route("")
@bp.route("/")
def index():
    return render_template("bluetooth/index.html", title="Bluetooth")


@bp.route("/api/status")
def api_status():
    return jsonify(bt.get_status())


@bp.route("/api/devices")
def api_devices():
    enrich = truthy(request.args.get("enrich"), default=False)
    show_all = truthy(request.args.get("all"), default=False)
    return jsonify(bt.get_discovered_devices(enrich=enrich, nearby_filter=not show_all))


@bp.route("/api/paired")
def api_paired():
    enrich = truthy(request.args.get("enrich"), default=True)
    return jsonify(bt.get_paired_devices(enrich=enrich))


@bp.route("/api/scan/start", methods=["POST"])
def api_scan_start():
    return jsonify(bt.start_scan())


@bp.route("/api/scan/stop", methods=["POST"])
def api_scan_stop():
    return jsonify(bt.stop_scan())


@bp.route("/api/power/on", methods=["POST"])
def api_power_on():
    return jsonify(bt.power_on())


@bp.route("/api/power/off", methods=["POST"])
def api_power_off():
    return jsonify(bt.power_off())


@bp.route("/api/pair", methods=["POST"])
def api_pair():
    mac, error = get_mac_from_request()
    if error:
        return error
    return jsonify(bt.pair(mac))


@bp.route("/api/trust", methods=["POST"])
def api_trust():
    mac, error = get_mac_from_request()
    if error:
        return error
    return jsonify(bt.trust(mac))


@bp.route("/api/connect", methods=["POST"])
def api_connect():
    mac, error = get_mac_from_request()
    if error:
        return error
    return jsonify(bt.connect(mac))


@bp.route("/api/disconnect", methods=["POST"])
def api_disconnect():
    mac, error = get_mac_from_request()
    if error:
        return error
    return jsonify(bt.disconnect(mac))


@bp.route("/api/remove", methods=["POST"])
def api_remove():
    mac, error = get_mac_from_request()
    if error:
        return error
    return jsonify(bt.remove(mac))


@bp.route("/api/setup", methods=["POST"])
def api_setup():
    mac, error = get_mac_from_request()
    if error:
        return error
    return jsonify(bt.setup_device(mac))
