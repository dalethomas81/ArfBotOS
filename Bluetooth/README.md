# Bluetooth

The `Bluetooth/` folder contains a standalone service for pairing Bluetooth controllers on a Raspberry Pi without coupling that work to the vision stack. The service is intended to become the shared backend that both a browser UI and the ArfBotOS HMI can call.

Primary goal: make it easy to connect a PlayStation DualSense (or similar gamepad) to the Pi so `Controller/DualSenseServer.py` can use it.

## Files

- [`BluetoothServer.py`](BluetoothServer.py): Flask server and HTTP API
- [`bluetoothctl_wrapper.py`](bluetoothctl_wrapper.py): wrapper around `bluetoothctl`, including a persistent scan session
- [`templates/index.html`](templates/index.html): guided browser UI for connecting a controller
- [`test_bluetoothctl_wrapper.py`](test_bluetoothctl_wrapper.py): unit tests with mocked `bluetoothctl`

## What It Does

- adapter status + power on/off
- start / stop scan (persistent scan process)
- discovered device list (controllers sorted first)
- paired device list
- one-click setup: pair + trust + connect
- individual pair / trust / connect / disconnect / remove actions

## Fresh Raspberry Pi Setup

These steps assume a new Raspberry Pi OS install and that you are SSH'd into the Pi.

### 1. Update the system

```bash
sudo apt-get update
sudo apt-get upgrade -y
```

### 2. Install Bluetooth and Python web dependencies

```bash
which bluetoothctl || sudo apt-get install -y bluez bluetooth pi-bluetooth
sudo apt-get install -y python3 python3-pip python3-flask
```

Notes:

- `bluez` provides `bluetoothctl`
- many Raspberry Pi OS images already include the Bluetooth stack, so the first command only installs it if `bluetoothctl` is missing
- `pi-bluetooth` is commonly present on Raspberry Pi OS but it is included here to make the setup explicit when Bluetooth packages are missing
- newer Raspberry Pi OS images may block `pip install` into the system Python environment, so `python3-flask` from `apt` is the simplest path
- if we later need Python packages that are not available through `apt`, we can switch this service to a virtual environment

### 3. Make sure Bluetooth is enabled

```bash
sudo systemctl enable bluetooth
sudo systemctl start bluetooth
sudo systemctl status bluetooth
```

### 4. Verify the adapter is visible

```bash
bluetoothctl show
bluetoothctl list
```

If the adapter is visible, `bluetoothctl show` should return controller information rather than an error.

### 5. Make sure the adapter is unblocked and powered on

On a fresh Raspberry Pi, it is possible for the Bluetooth service to be running while the adapter is still soft-blocked.

Check the rfkill state:

```bash
rfkill list
```

If Bluetooth shows `Soft blocked: yes`, run:

```bash
sudo rfkill unblock bluetooth
sudo systemctl restart bluetooth
bluetoothctl power on
```

Then verify again:

```bash
rfkill list
bluetoothctl show
```

Expected healthy state:

- `Soft blocked: no`
- `Powered: yes`
- `PowerState: on`

### 6. Copy or clone ArfBotOS onto the Pi

Example:

```bash
git clone <your-repo-url>
cd ArfBotOS/Bluetooth
```

Or copy just the `Bluetooth/` folder if you prefer.

### 7. Start the service

```bash
python3 BluetoothServer.py
```

By default the service listens on:

```text
http://0.0.0.0:50014/
```

### 8. Test the API locally on the Pi

In a second SSH terminal:

```bash
curl http://localhost:50014/api/status
curl -X POST http://localhost:50014/api/power/on
curl -X POST http://localhost:50014/api/scan/start
sleep 8
curl http://localhost:50014/api/devices
curl http://localhost:50014/api/paired
curl -X POST http://localhost:50014/api/scan/stop
```

One-click controller setup (replace the MAC):

```bash
curl -X POST http://localhost:50014/api/setup \
  -H 'Content-Type: application/json' \
  -d '{"mac":"AA:BB:CC:DD:EE:FF"}'
```

### 9. Open the browser UI from another machine

From a PC on the same network:

```text
http://<raspberry-pi-ip>:50014/
```

## Recommended DualSense Workflow

1. Open the web UI.
2. Confirm the adapter badge says powered on (use **Power On Bluetooth** if needed).
3. On the DualSense, hold **Create + PS** until the light bar blinks.
4. Click **Scan for Controllers**.
5. When `DualSense Wireless Controller` appears, click **Connect Controller**.
6. Confirm the top badge shows the controller as connected.
7. Start `Controller/DualSenseServer.py` as usual so the PLC/HMI can read the pad.

## How It Works

`BluetoothServer.py` starts a Flask app and creates one `BluetoothctlWrapper` instance.

Most commands still open a short `bluetoothctl` session, send commands, then `quit`.

Scanning is different: **Start Scan** launches a long-lived `bluetoothctl` process and keeps `scan on` active until **Stop Scan** (or process exit). The old MVP sent `scan on` and immediately quit, which stopped discovery before devices could appear.

The browser page is guided around controller setup. Advanced adapter details and the raw JSON response stay collapsed for debugging.

## Default Network Settings

When started directly, the server uses:

- host: `0.0.0.0`
- port: `50014`

These can be overridden with environment variables:

- `ARFBOTOS_BT_HOST`
- `ARFBOTOS_BT_PORT`
- `ARFBOTOS_BT_DEBUG`

## API Endpoints

- `GET /`
- `GET /api/status`
- `GET /api/devices?enrich=false`
- `GET /api/paired?enrich=true`
- `POST /api/scan/start`
- `POST /api/scan/stop`
- `POST /api/power/on`
- `POST /api/power/off`
- `POST /api/setup` (pair + trust + connect)
- `POST /api/pair`
- `POST /api/trust`
- `POST /api/connect`
- `POST /api/disconnect`
- `POST /api/remove`

Device action endpoints expect a JSON body like:

```json
{
  "mac": "AA:BB:CC:DD:EE:FF"
}
```

## Running It

```bash
python3 BluetoothServer.py
```

Then open:

```text
http://<raspberry-pi-hostname-or-ip>:50014/
```

## Optional Environment Variables

```bash
export ARFBOTOS_BT_HOST=0.0.0.0
export ARFBOTOS_BT_PORT=50014
export ARFBOTOS_BT_DEBUG=false
python3 BluetoothServer.py
```

## Local Unit Tests

From this folder, on a machine with Python 3 (no Bluetooth hardware required):

```bash
python3 -m unittest test_bluetoothctl_wrapper.py -v
```

## Notes

- This version uses `bluetoothctl` rather than the BlueZ DBus API to keep the implementation simple and inspectable.
- The wrapper validates MAC address format before pair/trust/connect/disconnect/remove/setup.
- Discovered-device refresh skips per-device `info` calls by default so scanning stays responsive. Paired-device refresh still enriches by default.
- Controllers are detected heuristically from names/icons (`DualSense`, `Wireless Controller`, `Xbox`, etc.) and sorted to the top of the list.
- The service is designed as a standalone backend so a future CODESYS HMI page can call the same endpoints.
- Pairing can still fail if the DualSense is not in pairing mode, already bonded to another host, or if the Pi adapter is soft-blocked.
