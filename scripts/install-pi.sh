#!/usr/bin/env bash
# ArfBotOS Raspberry Pi installer.
#
# Automates the Linux-side Software Installation steps from the wiki:
# camera overlay, OpenCV, vision services, DualSense service, and
# CODESYS SysProcess AllowAll when the runtime is already present.
#
# Does NOT install: Raspberry Pi OS, the CODESYS Windows IDE/runtime,
# CODESYS licenses, or Arduino/Teensy firmware.
#
# From SSH on the Pi (OpenKiln2-style, no prior clone required):
#   curl -sSL https://raw.githubusercontent.com/dalethomas81/ArfBotOS/feature/pi-installer/scripts/install-pi.sh | bash
#   curl -sSL https://raw.githubusercontent.com/dalethomas81/ArfBotOS/feature/pi-installer/scripts/install-pi.sh | bash -s -- --plc-only
#   curl -sSL https://raw.githubusercontent.com/dalethomas81/ArfBotOS/feature/pi-installer/scripts/install-pi.sh | bash -s -- --vision-only
#
# From an existing checkout:
#   sudo ./scripts/install-pi.sh
#   sudo ./scripts/install-pi.sh --plc-only
#   sudo ./scripts/install-pi.sh --vision-only
#
# Safe to re-run. Existing cal.yaml / roi.yaml are left alone.

set -euo pipefail

DEFAULT_REPO_URL="https://github.com/dalethomas81/ArfBotOS.git"
# Keep this matching the branch this file is published from so
# `curl | bash` clones the same tree it just downloaded.
DEFAULT_REPO_REF="feature/pi-installer"

REPO_URL="${ARFBOT_REPO:-${DEFAULT_REPO_URL}}"
REPO_REF="${ARFBOT_REF:-${DEFAULT_REPO_REF}}"

VISION_DST="/var/opt/codesys/PlcLogic/Application/Vision"
CONTROLLER_DST="/var/opt/codesys/PlcLogic/Application/Controller"
VENV_DIR="/opt/arfbot/venv"
UDEV_RULES_DST="/etc/udev/rules.d/70-ps5-controller.rules"

SCRIPT_FILE=""
SCRIPT_DIR=""
REPO_ROOT=""
UDEV_RULES_SRC=""

VISION_ONLY=0
PLC_ONLY=0
SKIP_VISION=0
SKIP_CAMERA=0
SKIP_CONTROLLER=0
SKIP_CODESYS=0
NO_START=0
DRY_RUN=0
FORCE=0
CHECK_ONLY=0
RECREATE_VENV=0
NEED_REBOOT=0

log()  { printf '==> %s\n' "$*"; }
warn() { printf 'WARN: %s\n' "$*" >&2; }
die()  { printf 'ERROR: %s\n' "$*" >&2; exit 1; }

usage() {
    cat <<EOF
ArfBotOS Raspberry Pi installer

Usage:
  curl -sSL https://raw.githubusercontent.com/dalethomas81/ArfBotOS/${DEFAULT_REPO_REF}/scripts/install-pi.sh | bash
  curl -sSL https://raw.githubusercontent.com/dalethomas81/ArfBotOS/${DEFAULT_REPO_REF}/scripts/install-pi.sh | bash -s -- [options]
  sudo ./scripts/install-pi.sh [options]

Options:
  --plc-only          PLC Pi without vision. Installs DualSense and CODESYS
                      SysProcess config. Skips camera, OpenCV, and vision
                      services.
  --vision-only       Dedicated vision Pi. Installs camera, OpenCV, PyServer,
                      and VisionWeb. Skips DualSense and CODESYS config.
  --skip-camera       Do not patch config.txt or install picamera2.
  --skip-controller   Do not install the DualSense controller service.
  --skip-codesys      Do not patch CODESYSControl.cfg.
  --repo URL          Git remote to clone when not run from a checkout
                      (default: ${DEFAULT_REPO_URL}).
  --ref REF           Branch or tag to clone (default: ${DEFAULT_REPO_REF}).
  --no-start          Enable systemd units but do not start them.
  --recreate-venv     Delete /opt/arfbot/venv and create it again.
  --force             Allow running on a machine that is not a Raspberry Pi.
  --dry-run           Print actions without changing the system.
  --check             Verify this git checkout has the files the installer
                      needs, then exit. Does not need root or a Pi.
  -h, --help          Show this help.

Environment:
  ARFBOT_REPO         Same as --repo
  ARFBOT_REF          Same as --ref

Typical PLC Pi with vision (32-bit Raspberry Pi OS, after SSH):
  curl -sSL https://raw.githubusercontent.com/dalethomas81/ArfBotOS/${DEFAULT_REPO_REF}/scripts/install-pi.sh | bash

PLC Pi without vision:
  curl -sSL https://raw.githubusercontent.com/dalethomas81/ArfBotOS/${DEFAULT_REPO_REF}/scripts/install-pi.sh | bash -s -- --plc-only

Dedicated vision Pi (64-bit Lite recommended):
  curl -sSL https://raw.githubusercontent.com/dalethomas81/ArfBotOS/${DEFAULT_REPO_REF}/scripts/install-pi.sh | bash -s -- --vision-only

Re-run after CODESYS "Update Raspberry Pi" to apply SysProcess AllowAll.
EOF
}

is_true() { [[ "${1}" -eq 1 ]]; }

run() {
    if is_true "${DRY_RUN}"; then
        printf 'DRY-RUN: %s\n' "$*"
        return 0
    fi
    printf '+ %s\n' "$*"
    "$@"
}

as_root() {
    if [[ "${EUID}" -eq 0 ]]; then
        "$@"
    else
        sudo "$@"
    fi
}

need_arg() {
    local opt="$1"
    local value="${2:-}"
    if [[ -z "${value}" || "${value}" == --* ]]; then
        die "${opt} requires a value"
    fi
}

parse_args() {
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --plc-only)        PLC_ONLY=1; SKIP_VISION=1; SKIP_CAMERA=1 ;;
            --vision-only)     VISION_ONLY=1; SKIP_CONTROLLER=1; SKIP_CODESYS=1 ;;
            --skip-camera)     SKIP_CAMERA=1 ;;
            --skip-controller) SKIP_CONTROLLER=1 ;;
            --skip-codesys)    SKIP_CODESYS=1 ;;
            --repo)
                need_arg "$1" "${2:-}"
                REPO_URL="$2"
                shift
                ;;
            --ref)
                need_arg "$1" "${2:-}"
                REPO_REF="$2"
                shift
                ;;
            --no-start)        NO_START=1 ;;
            --recreate-venv)   RECREATE_VENV=1 ;;
            --force)           FORCE=1 ;;
            --dry-run)         DRY_RUN=1 ;;
            --check)           CHECK_ONLY=1 ;;
            -h|--help)         usage; exit 0 ;;
            *)                 die "unknown option: $1 (try --help)" ;;
        esac
        shift
    done
    if is_true "${PLC_ONLY}" && is_true "${VISION_ONLY}"; then
        die "--plc-only and --vision-only cannot be used together"
    fi
}

resolve_script_paths() {
    local source="${BASH_SOURCE[0]:-$0}"
    SCRIPT_FILE=""
    SCRIPT_DIR=""
    REPO_ROOT=""
    UDEV_RULES_SRC=""
    if [[ -n "${source}" && "${source}" != "-" && "${source}" != "bash" && -f "${source}" ]]; then
        SCRIPT_DIR="$(cd "$(dirname "${source}")" && pwd)"
        SCRIPT_FILE="${SCRIPT_DIR}/$(basename "${source}")"
        REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
        UDEV_RULES_SRC="${SCRIPT_DIR}/pi/70-ps5-controller.rules"
    fi
}

is_repo_checkout() {
    [[ -n "${REPO_ROOT}" ]] \
        && [[ -f "${REPO_ROOT}/Controller/DualSenseServer.py" ]] \
        && [[ -f "${REPO_ROOT}/OpenCV/SocketServer/PyServer.py" ]] \
        && [[ -f "${UDEV_RULES_SRC}" ]]
}

install_mode() {
    if is_true "${VISION_ONLY}"; then
        printf '%s\n' "vision-only"
    elif is_true "${PLC_ONLY}"; then
        printf '%s\n' "plc-only"
    else
        printf '%s\n' "full"
    fi
}

require_repo_files() {
    local missing=0
    local required=()
    if ! is_true "${SKIP_VISION}"; then
        required+=(
            "${REPO_ROOT}/OpenCV/SocketServer/PyServer.py"
            "${REPO_ROOT}/OpenCV/ImageCapture/CaptureImage.py"
            "${REPO_ROOT}/OpenCV/FastTemplateMatching/FastTemplateMatching.py"
            "${REPO_ROOT}/OpenCV/FastTemplateMatching/roi.yaml"
            "${REPO_ROOT}/OpenCV/CameraCalibration/CalibrateCamera.py"
            "${REPO_ROOT}/OpenCV/CameraCalibration/cal.yaml"
            "${REPO_ROOT}/OpenCV/VisionWebServer/vision.py"
            "${REPO_ROOT}/OpenCV/VisionWebServer/config.py"
            "${REPO_ROOT}/OpenCV/VisionWebServer/app/__init__.py"
            "${REPO_ROOT}/OpenCV/VisionWebServer/app/routes.py"
            "${REPO_ROOT}/OpenCV/VisionWebServer/app/forms.py"
        )
    fi
    if ! is_true "${SKIP_CONTROLLER}"; then
        required+=(
            "${UDEV_RULES_SRC}"
            "${REPO_ROOT}/Controller/DualSenseServer.py"
            "${REPO_ROOT}/Controller/DualSenseClient.py"
        )
    fi
    if [[ ${#required[@]} -eq 0 ]]; then
        return 0
    fi
    local f
    for f in "${required[@]}"; do
        if [[ ! -f "${f}" ]]; then
            warn "missing ${f}"
            missing=1
        fi
    done
    if [[ "${missing}" -ne 0 ]]; then
        die "this does not look like a complete ArfBotOS checkout (REPO_ROOT=${REPO_ROOT})"
    fi
}

user_home() {
    local user="${SUDO_USER:-${USER:-}}"
    local home=""
    if [[ -n "${user}" && "${user}" != "root" ]] && command -v getent >/dev/null 2>&1; then
        home="$(getent passwd "${user}" | awk -F: '{print $6}')"
    fi
    if [[ -n "${home}" ]]; then
        printf '%s\n' "${home}"
        return 0
    fi
    printf '%s\n' "${HOME}"
}

ensure_git() {
    if command -v git >/dev/null 2>&1; then
        return 0
    fi
    log "installing git"
    if is_true "${DRY_RUN}"; then
        printf 'DRY-RUN: apt-get install -y git\n'
        return 0
    fi
    as_root apt-get update
    as_root apt-get install -y git
}

bootstrap_from_git() {
    local dest
    dest="$(user_home)/ArfBotOS"
    log "not running from an ArfBotOS checkout; cloning ${REPO_URL} (${REPO_REF}) into ${dest}"
    ensure_git
    if is_true "${DRY_RUN}"; then
        printf 'DRY-RUN: git clone --depth 1 --branch %s %s %s\n' "${REPO_REF}" "${REPO_URL}" "${dest}"
        printf 'DRY-RUN: exec %s/scripts/install-pi.sh\n' "${dest}"
        die "dry-run from a pipe stops after the clone plan; omit --dry-run or run from a checkout"
    fi
    if is_true "${CHECK_ONLY}"; then
        die "--check needs a local checkout; clone the repo first or omit --check"
    fi
    if [[ -d "${dest}/.git" ]]; then
        log "updating existing clone ${dest}"
        git -C "${dest}" fetch --depth 1 origin "${REPO_REF}"
        git -C "${dest}" checkout -B "${REPO_REF}" FETCH_HEAD
    elif [[ -e "${dest}" ]]; then
        die "${dest} exists and is not an ArfBotOS git clone"
    else
        git clone --depth 1 --branch "${REPO_REF}" "${REPO_URL}" "${dest}"
    fi
    local next="${dest}/scripts/install-pi.sh"
    if [[ ! -f "${next}" ]]; then
        die "clone is missing ${next} (check --ref ${REPO_REF})"
    fi
    chmod +x "${next}" || true
    log "re-executing ${next}"
    exec bash "${next}" "$@"
}

is_raspberry_pi() {
    local model=""
    if [[ -r /proc/device-tree/model ]]; then
        model="$(tr -d '\0' </proc/device-tree/model)"
    fi
    [[ "${model}" == *Raspberry* ]] || [[ -f /etc/rpi-issue ]]
}

require_root() {
    if is_true "${DRY_RUN}" || is_true "${CHECK_ONLY}"; then
        return 0
    fi
    if [[ "${EUID}" -eq 0 ]]; then
        return 0
    fi
    if [[ -z "${SCRIPT_FILE}" || ! -f "${SCRIPT_FILE}" ]]; then
        die "run as root: sudo $0 $*"
    fi
    log "re-executing with sudo"
    exec sudo --preserve-env=ARFBOT_REPO,ARFBOT_REF bash "${SCRIPT_FILE}" "$@"
}

require_pi() {
    if is_raspberry_pi; then
        return 0
    fi
    if is_true "${FORCE}" || is_true "${DRY_RUN}"; then
        warn "this machine does not look like a Raspberry Pi; continuing because --force/--dry-run was set"
        return 0
    fi
    die "this installer is meant to run on a Raspberry Pi (use --force to override)"
}

copy_file() {
    local src="$1"
    local dst="$2"
    if is_true "${DRY_RUN}"; then
        printf 'DRY-RUN: cp %s %s\n' "${src}" "${dst}"
        return 0
    fi
    mkdir -p "$(dirname "${dst}")"
    cp -f "${src}" "${dst}"
}

copy_unless_exists() {
    local src="$1"
    local dst="$2"
    if [[ -e "${dst}" ]]; then
        log "keeping existing ${dst}"
        return 0
    fi
    copy_file "${src}" "${dst}"
}

copy_dir_contents() {
    local src="$1"
    local dst="$2"
    if is_true "${DRY_RUN}"; then
        printf 'DRY-RUN: copy dir %s -> %s\n' "${src}" "${dst}"
        return 0
    fi
    mkdir -p "${dst}"
    local item
    local name
    for item in "${src}"/*; do
        [[ -e "${item}" ]] || continue
        name="$(basename "${item}")"
        case "${name}" in
            __pycache__|*.pyc|app.db|README.md) continue ;;
        esac
        if [[ -d "${item}" ]]; then
            rm -rf "${dst}/${name}"
            cp -R "${item}" "${dst}/${name}"
        else
            cp -f "${item}" "${dst}/"
        fi
    done
}

backup_once() {
    local file="$1"
    [[ -f "${file}" ]] || return 0
    if [[ -f "${file}.arfbot.bak" ]]; then
        return 0
    fi
    run cp -f "${file}" "${file}.arfbot.bak"
}

boot_config_path() {
    if [[ -f /boot/firmware/config.txt ]]; then
        printf '%s\n' /boot/firmware/config.txt
    elif [[ -f /boot/config.txt ]]; then
        printf '%s\n' /boot/config.txt
    fi
}

configure_camera() {
    if is_true "${SKIP_CAMERA}"; then
        log "skipping camera overlay (--skip-camera/--plc-only)"
        return 0
    fi

    local cfg
    cfg="$(boot_config_path || true)"
    if [[ -z "${cfg}" ]]; then
        warn "no /boot/firmware/config.txt or /boot/config.txt found; skipping camera overlay"
        return 0
    fi

    log "configuring IMX477 overlay in ${cfg}"
    backup_once "${cfg}"
    if is_true "${DRY_RUN}"; then
        printf 'DRY-RUN: set camera_auto_detect=0 and dtoverlay=imx477 in %s\n' "${cfg}"
        NEED_REBOOT=1
        return 0
    fi

    local result
    result="$(python3 - "${cfg}" <<'PY'
import pathlib, sys
path = pathlib.Path(sys.argv[1])
text = path.read_text()
original = text
lines = text.splitlines()
out = []
in_all = False
saw_all = False
auto_written = False
overlay_present = any(l.strip().startswith("dtoverlay=imx477") for l in lines)

def is_section(line: str) -> bool:
    s = line.strip()
    return s.startswith("[") and s.endswith("]")

for line in lines:
    stripped = line.strip()
    if is_section(stripped):
        if in_all and not auto_written:
            out.append("camera_auto_detect=0")
            auto_written = True
        if not overlay_present and in_all:
            out.append("dtoverlay=imx477")
            overlay_present = True
        in_all = stripped.lower() == "[all]"
        if in_all:
            saw_all = True
        out.append(line)
        continue
    if stripped.startswith("camera_auto_detect="):
        out.append("camera_auto_detect=0")
        auto_written = True
        continue
    out.append(line)

if in_all and not auto_written:
    out.append("camera_auto_detect=0")
    auto_written = True
if in_all and not overlay_present:
    out.append("dtoverlay=imx477")
    overlay_present = True

if not saw_all:
    if out and out[-1].strip() != "":
        out.append("")
    out.append("[all]")
    out.append("camera_auto_detect=0")
    if not overlay_present:
        out.append("dtoverlay=imx477")

new = "\n".join(out) + "\n"
if new != original:
    path.write_text(new)
    print("updated")
else:
    print("unchanged")
PY
)"
    log "camera overlay ${result}"
    if [[ "${result}" == "updated" ]]; then
        NEED_REBOOT=1
    fi
}

enable_i2c() {
    if is_true "${SKIP_CAMERA}"; then
        return 0
    fi
    if command -v raspi-config >/dev/null 2>&1; then
        log "enabling I2C"
        run raspi-config nonint do_i2c 0 || warn "raspi-config failed to enable I2C"
    else
        warn "raspi-config not found; not enabling I2C via raspi-config"
    fi
}

apt_install_available() {
    local pkg
    local to_install=()
    if is_true "${DRY_RUN}"; then
        printf 'DRY-RUN: apt-get install -y %s\n' "$*"
        return 0
    fi
    for pkg in "$@"; do
        if apt-cache show "${pkg}" >/dev/null 2>&1; then
            to_install+=("${pkg}")
        else
            warn "apt package not available: ${pkg}"
        fi
    done
    if [[ ${#to_install[@]} -eq 0 ]]; then
        warn "no apt packages from this group were available"
        return 0
    fi
    run apt-get install -y "${to_install[@]}"
}

install_packages() {
    log "installing apt packages"
    export DEBIAN_FRONTEND=noninteractive
    run apt-get update
    apt_install_available python3 python3-venv python3-pip git
    if ! is_true "${SKIP_VISION}"; then
        apt_install_available python3-opencv python3-numpy
        if ! is_true "${SKIP_CAMERA}"; then
            apt_install_available python3-picamera2 python3-libcamera i2c-tools
        fi
    fi
    if ! is_true "${SKIP_CONTROLLER}"; then
        apt_install_available libhidapi-dev libhidapi-hidraw0 libhidapi-libusb0
    fi
}

needs_venv() {
    ! is_true "${SKIP_VISION}" || ! is_true "${SKIP_CONTROLLER}"
}

ensure_venv() {
    if ! needs_venv; then
        log "skipping Python venv (no vision or controller components requested)"
        return 0
    fi
    if is_true "${RECREATE_VENV}" && [[ -d "${VENV_DIR}" ]]; then
        log "removing existing venv (${VENV_DIR})"
        run rm -rf "${VENV_DIR}"
    fi
    if [[ -d "${VENV_DIR}" ]]; then
        if [[ -f "${VENV_DIR}/pyvenv.cfg" ]] && grep -q 'include-system-site-packages = false' "${VENV_DIR}/pyvenv.cfg"; then
            warn "${VENV_DIR} was created without system site packages; cv2/picamera2 may fail. Re-run with --recreate-venv"
        else
            log "using existing venv ${VENV_DIR}"
        fi
    else
        log "creating venv with system site packages at ${VENV_DIR}"
        run python3 -m venv --system-site-packages "${VENV_DIR}"
    fi

    local pip="${VENV_DIR}/bin/pip"
    if is_true "${DRY_RUN}"; then
        if ! is_true "${SKIP_VISION}"; then
            printf 'DRY-RUN: %s install flask flask-wtf\n' "${pip}"
        fi
        if ! is_true "${SKIP_CONTROLLER}"; then
            printf 'DRY-RUN: %s install git+https://github.com/flok/pydualsense.git\n' "${pip}"
        fi
        return 0
    fi
    if ! is_true "${SKIP_VISION}"; then
        "${pip}" install flask flask-wtf
    fi
    if ! is_true "${SKIP_CONTROLLER}"; then
        if ! "${pip}" install "git+https://github.com/flok/pydualsense.git"; then
            warn "pydualsense from GitHub failed; falling back to PyPI"
            "${pip}" install pydualsense
        fi
    fi
}

deploy_vision_files() {
    if is_true "${SKIP_VISION}"; then
        log "skipping vision files (--plc-only)"
        return 0
    fi
    log "deploying vision files to ${VISION_DST}"
    if ! is_true "${DRY_RUN}"; then
        mkdir -p "${VISION_DST}/Templates" "${VISION_DST}/VisionWebServer"
    else
        printf 'DRY-RUN: mkdir -p %s/Templates %s/VisionWebServer\n' "${VISION_DST}" "${VISION_DST}"
    fi

    copy_file "${REPO_ROOT}/OpenCV/SocketServer/PyServer.py" "${VISION_DST}/PyServer.py"
    copy_file "${REPO_ROOT}/OpenCV/ImageCapture/CaptureImage.py" "${VISION_DST}/CaptureImage.py"
    copy_file "${REPO_ROOT}/OpenCV/FastTemplateMatching/FastTemplateMatching.py" "${VISION_DST}/FastTemplateMatching.py"
    copy_file "${REPO_ROOT}/OpenCV/CameraCalibration/CalibrateCamera.py" "${VISION_DST}/CalibrateCamera.py"
    copy_unless_exists "${REPO_ROOT}/OpenCV/CameraCalibration/cal.yaml" "${VISION_DST}/cal.yaml"
    copy_unless_exists "${REPO_ROOT}/OpenCV/FastTemplateMatching/roi.yaml" "${VISION_DST}/roi.yaml"
    copy_dir_contents "${REPO_ROOT}/OpenCV/VisionWebServer" "${VISION_DST}/VisionWebServer"

    if ! is_true "${DRY_RUN}"; then
        : > "${VISION_DST}/VisionLog.txt"
        : > "${VISION_DST}/VisionErrorLog.txt"
        chmod 644 "${VISION_DST}/"*".py" "${VISION_DST}/cal.yaml" "${VISION_DST}/roi.yaml" 2>/dev/null || true
    fi
}

deploy_controller_files() {
    if is_true "${SKIP_CONTROLLER}"; then
        log "skipping DualSense controller (--skip-controller/--vision-only)"
        return 0
    fi
    log "deploying controller files to ${CONTROLLER_DST}"
    if ! is_true "${DRY_RUN}"; then
        mkdir -p "${CONTROLLER_DST}"
        : > "${CONTROLLER_DST}/ControllerLog.txt"
        : > "${CONTROLLER_DST}/ControllerErrorLog.txt"
    fi
    copy_file "${REPO_ROOT}/Controller/DualSenseServer.py" "${CONTROLLER_DST}/DualSenseServer.py"
    copy_file "${REPO_ROOT}/Controller/DualSenseClient.py" "${CONTROLLER_DST}/DualSenseClient.py"
    copy_file "${UDEV_RULES_SRC}" "${UDEV_RULES_DST}"
    if command -v udevadm >/dev/null 2>&1; then
        run udevadm control --reload-rules || warn "udevadm reload failed"
        run udevadm trigger || warn "udevadm trigger failed"
    fi
}

write_unit() {
    local dest="$1"
    if is_true "${DRY_RUN}"; then
        printf 'DRY-RUN: write systemd unit %s\n' "${dest}"
        return 0
    fi
    cat > "${dest}"
}

install_systemd_units() {
    local python_bin="${VENV_DIR}/bin/python"
    local flask_bin="${VENV_DIR}/bin/flask"
    local units=()

    log "writing systemd units"
    if ! is_true "${SKIP_VISION}"; then
        write_unit /etc/systemd/system/PyServer.service <<EOF
[Unit]
Description=ArfBotOS vision command socket server
After=network-online.target multi-user.target
Wants=network-online.target

[Service]
Type=simple
Restart=always
RestartSec=3
WorkingDirectory=${VISION_DST}
StandardOutput=append:${VISION_DST}/VisionLog.txt
StandardError=append:${VISION_DST}/VisionErrorLog.txt
ExecStart=${python_bin} ${VISION_DST}/PyServer.py

[Install]
WantedBy=multi-user.target
EOF
        write_unit /etc/systemd/system/VisionWeb.service <<EOF
[Unit]
Description=ArfBotOS vision web server
After=network-online.target multi-user.target
Wants=network-online.target

[Service]
Type=simple
Restart=always
RestartSec=3
Environment=FLASK_APP=vision.py
WorkingDirectory=${VISION_DST}/VisionWebServer
ExecStart=${flask_bin} run -h 0.0.0.0 -p 5000

[Install]
WantedBy=multi-user.target
EOF
        units+=(PyServer.service VisionWeb.service)
    fi

    if ! is_true "${SKIP_CONTROLLER}"; then
        write_unit /etc/systemd/system/DualSenseController.service <<EOF
[Unit]
Description=ArfBotOS DualSense controller socket server
After=network-online.target multi-user.target
Wants=network-online.target

[Service]
Type=simple
Restart=always
RestartSec=3
WorkingDirectory=${CONTROLLER_DST}
StandardOutput=append:${CONTROLLER_DST}/ControllerLog.txt
StandardError=append:${CONTROLLER_DST}/ControllerErrorLog.txt
ExecStart=${python_bin} ${CONTROLLER_DST}/DualSenseServer.py

[Install]
WantedBy=multi-user.target
EOF
        units+=(DualSenseController.service)
    fi

    if [[ ${#units[@]} -eq 0 ]]; then
        log "no systemd units to install"
        return 0
    fi

    if command -v systemctl >/dev/null 2>&1; then
        run systemctl daemon-reload
        run systemctl enable "${units[@]}"
        if ! is_true "${NO_START}"; then
            run systemctl restart "${units[@]}" || warn "one or more services failed to start (see systemctl status)"
        else
            log "services enabled but not started (--no-start)"
        fi
    else
        warn "systemctl not found; systemd units were written but not enabled"
    fi
}

upsert_ini() {
    local file="$1"
    local section="$2"
    local key="$3"
    local value="$4"
    if is_true "${DRY_RUN}"; then
        printf 'DRY-RUN: set [%s] %s=%s in %s\n' "${section}" "${key}" "${value}" "${file}"
        return 0
    fi
    python3 - "${file}" "${section}" "${key}" "${value}" <<'PY'
import pathlib, sys
path = pathlib.Path(sys.argv[1])
section, key, value = sys.argv[2], sys.argv[3], sys.argv[4]
lines = path.read_text().splitlines() if path.exists() else []
out = []
in_section = False
section_found = False
key_written = False
header = f"[{section}]"

def is_section(line: str) -> bool:
    s = line.strip()
    return s.startswith("[") and s.endswith("]")

for line in lines:
    stripped = line.strip()
    if is_section(stripped):
        if in_section and not key_written:
            out.append(f"{key}={value}")
            key_written = True
        in_section = stripped.lower() == header.lower()
        if in_section:
            section_found = True
        out.append(line)
        continue
    if in_section and stripped.lower().startswith(key.lower() + "="):
        out.append(f"{key}={value}")
        key_written = True
        continue
    out.append(line)

if in_section and not key_written:
    out.append(f"{key}={value}")
    key_written = True
if not section_found:
    if out and out[-1].strip() != "":
        out.append("")
    out.append(header)
    out.append(f"{key}={value}")

path.parent.mkdir(parents=True, exist_ok=True)
path.write_text("\n".join(out) + "\n")
PY
}

configure_codesys() {
    if is_true "${SKIP_CODESYS}"; then
        log "skipping CODESYS SysProcess config"
        return 0
    fi

    local candidates=(
        /etc/CODESYSControl.cfg
        /etc/CODESYSControl_User.cfg
        /etc/codesyscontrol/CODESYSControl.cfg
        /etc/codesyscontrol/CODESYSControl_User.cfg
    )
    local found=0
    local cfg
    for cfg in "${candidates[@]}"; do
        if [[ -f "${cfg}" ]]; then
            found=1
            log "setting SysProcess Command=AllowAll in ${cfg}"
            backup_once "${cfg}"
            upsert_ini "${cfg}" SysProcess Command AllowAll
        fi
    done
    if [[ "${found}" -eq 0 ]]; then
        warn "CODESYS runtime config not found. Install the runtime from Windows (Tools > Update Raspberry Pi), then re-run this script to allow PLC process execution."
    fi
}

chown_deploy_tree() {
    local user="${SUDO_USER:-}"
    if [[ -z "${user}" || "${user}" == "root" ]]; then
        return 0
    fi
    if ! is_true "${DRY_RUN}"; then
        if [[ -d /var/opt/codesys ]]; then
            chown -R "${user}:${user}" /var/opt/codesys || warn "could not chown /var/opt/codesys to ${user}"
        fi
    else
        printf 'DRY-RUN: chown -R %s:%s /var/opt/codesys\n' "${user}" "${user}"
    fi
}

venv_python() {
    if [[ -x "${VENV_DIR}/bin/python" ]]; then
        printf '%s\n' "${VENV_DIR}/bin/python"
    else
        printf '%s\n' python3
    fi
}

verify_import() {
    local py="$1"
    local expr="$2"
    local label="$3"
    if is_true "${DRY_RUN}"; then
        printf 'DRY-RUN: %s -c %s\n' "${py}" "${expr}"
        return 0
    fi
    if "${py}" -c "${expr}"; then
        log "ok: ${label}"
    else
        warn "failed: ${label}"
        return 1
    fi
}

print_service_status() {
    local unit="$1"
    if ! command -v systemctl >/dev/null 2>&1; then
        return 0
    fi
    if is_true "${DRY_RUN}"; then
        printf 'DRY-RUN: systemctl status %s\n' "${unit}"
        return 0
    fi
    systemctl --no-pager --full status "${unit}" || true
}

verify_install() {
    log "verifying installation"
    local py
    py="$(venv_python)"
    local failed=0
    if ! is_true "${SKIP_VISION}"; then
        verify_import "${py}" "import cv2; print('cv2', cv2.__version__)" "OpenCV" || failed=1
        if ! is_true "${SKIP_CAMERA}"; then
            verify_import "${py}" "from picamera2 import Picamera2; print('picamera2 ok')" "picamera2" || failed=1
        fi
        verify_import "${py}" "import flask, flask_wtf; print('flask ok')" "Flask" || failed=1
    fi
    if ! is_true "${SKIP_CONTROLLER}"; then
        verify_import "${py}" "from pydualsense import pydualsense; print('pydualsense ok')" "pydualsense" || failed=1
    fi

    if is_true "${DRY_RUN}"; then
        log "dry-run: skipping destination file checks"
    elif ! is_true "${SKIP_VISION}"; then
        local required_files=(
            "${VISION_DST}/PyServer.py"
            "${VISION_DST}/CaptureImage.py"
            "${VISION_DST}/FastTemplateMatching.py"
            "${VISION_DST}/CalibrateCamera.py"
            "${VISION_DST}/cal.yaml"
            "${VISION_DST}/roi.yaml"
            "${VISION_DST}/VisionWebServer/vision.py"
        )
        local f
        for f in "${required_files[@]}"; do
            if [[ -f "${f}" ]]; then
                log "ok: ${f}"
            else
                warn "missing ${f}"
                failed=1
            fi
        done
    fi

    if command -v systemctl >/dev/null 2>&1 && ! is_true "${DRY_RUN}"; then
        if ! is_true "${SKIP_VISION}"; then
            print_service_status PyServer.service
            print_service_status VisionWeb.service
        fi
        if ! is_true "${SKIP_CONTROLLER}"; then
            print_service_status DualSenseController.service
        fi
    fi
    return "${failed}"
}

print_next_steps() {
    local ip=""
    ip="$(hostname -I 2>/dev/null | awk '{print $1}')" || true
    cat <<EOF

----------------------------------------------------------------------
ArfBotOS Pi install finished
  mode:            $(install_mode)
  clone / files:   ${REPO_ROOT}
  venv:            ${VENV_DIR}
EOF
    if ! is_true "${SKIP_VISION}"; then
        cat <<EOF
  vision files:    ${VISION_DST}
  vision web:      http://${ip:-<pi-ip>}:5000
EOF
    fi
    if ! is_true "${SKIP_CONTROLLER}"; then
        cat <<EOF
  controller:      ${CONTROLLER_DST}
EOF
    fi
    if is_true "${NEED_REBOOT}"; then
        cat <<'EOF'
  camera overlay:  changed — reboot before using the camera:
                     sudo reboot
EOF
    fi
    if ! is_true "${SKIP_CODESYS}" && [[ ! -f /etc/CODESYSControl.cfg && ! -f /etc/codesyscontrol/CODESYSControl.cfg ]]; then
        cat <<EOF
  CODESYS runtime: not detected. From Windows CODESYS:
                     Tools > Update Raspberry Pi  (32-bit multicore)
                     then re-run this installer (same curl command, or sudo ${SCRIPT_FILE:-./scripts/install-pi.sh})
                     then Multiple Download of ArfBot.project
EOF
    fi
    if ! is_true "${SKIP_CONTROLLER}"; then
        cat <<'EOF'
  DualSense:       USB works without pairing. For Bluetooth:
                     bluetoothctl
                     pairable on
                     agent on
                     default-agent
                     scan on
                     (hold PS + Share until the trackpad flashes)
                     pair <MAC>
                     trust <MAC>
                     exit
EOF
    fi
    if is_true "${VISION_ONLY}"; then
        cat <<'EOF'
  Vision-only Pi:  set this host's IP on the Programming HMI vision settings.
EOF
    fi
    cat <<'EOF'
----------------------------------------------------------------------
EOF
}

print_check_plan() {
    cat <<EOF
ArfBotOS installer checkout looks complete.

Repo:              ${REPO_ROOT}
Mode:              $(install_mode)
Would deploy to:   ${VISION_DST}
Controller dest:   ${CONTROLLER_DST}
Venv:              ${VENV_DIR}

From SSH (no prior clone):
  curl -sSL https://raw.githubusercontent.com/dalethomas81/ArfBotOS/${REPO_REF}/scripts/install-pi.sh | bash
  curl -sSL https://raw.githubusercontent.com/dalethomas81/ArfBotOS/${REPO_REF}/scripts/install-pi.sh | bash -s -- --plc-only
  curl -sSL https://raw.githubusercontent.com/dalethomas81/ArfBotOS/${REPO_REF}/scripts/install-pi.sh | bash -s -- --vision-only
EOF
}

main() {
    parse_args "$@"
    resolve_script_paths
    if ! is_repo_checkout; then
        bootstrap_from_git "$@"
    fi
    require_repo_files
    if is_true "${CHECK_ONLY}"; then
        print_check_plan
        exit 0
    fi
    require_root "$@"
    require_pi
    install_packages
    configure_camera
    enable_i2c
    ensure_venv
    deploy_vision_files
    deploy_controller_files
    install_systemd_units
    configure_codesys
    chown_deploy_tree
    local verify_rc=0
    verify_install || verify_rc=$?
    print_next_steps
    if [[ "${verify_rc}" -ne 0 ]]; then
        warn "install completed with verification warnings"
        exit 1
    fi
}

main "$@"
