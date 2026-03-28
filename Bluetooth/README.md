# Bluetooth

The `Bluetooth/` folder contains a standalone MVP service for pairing Bluetooth devices on a Raspberry Pi without coupling that work to the vision stack. The service is intended to become the shared backend that both a browser UI and the ArfBotOS HMI can call.

## Files

- [`BluetoothServer.py`](c:/Users/dalet/Github/ArfBotOS/Bluetooth/BluetoothServer.py): Flask server and HTTP API
- [`bluetoothctl_wrapper.py`](c:/Users/dalet/Github/ArfBotOS/Bluetooth/bluetoothctl_wrapper.py): wrapper around `bluetoothctl`
- [`templates/index.html`](c:/Users/dalet/Github/ArfBotOS/Bluetooth/templates/index.html): simple browser UI

## Current MVP

The MVP provides:

- adapter status
- start scan
- stop scan
- discovered device list
- paired device list
- pair device
- trust device
- connect device
- disconnect device
- remove device

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
- newer Raspberry Pi OS images may block `pip install` into the system Python environment, so `python3-flask` from `apt` is the simplest path for this MVP
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
curl http://localhost:50014/api/devices
curl http://localhost:50014/api/paired
curl -X POST http://localhost:50014/api/scan/start
sleep 5
curl http://localhost:50014/api/devices
curl -X POST http://localhost:50014/api/scan/stop
```

### 9. Open the browser UI from another machine

From a PC on the same network:

```text
http://<raspberry-pi-ip>:50014/
```

## First Test Workflow

Once the service is up, the recommended first test is:

1. Confirm `/api/status` shows a controller and `powered: true`.
2. Start a scan.
3. Refresh the discovered devices list after a few seconds.
4. Pick a device MAC address.
5. Try `pair`.
6. Try `trust`.
7. Try `connect`.

If anything fails, the first things to capture are:

- `bluetoothctl show`
- `curl http://localhost:50014/api/status`
- `curl -X POST http://localhost:50014/api/scan/start`
- `curl http://localhost:50014/api/devices`

## How It Works

`BluetoothServer.py` starts a Flask app and creates one `BluetoothctlWrapper` instance.

The wrapper opens `bluetoothctl` in a subprocess session, sends one or more commands followed by `quit`, captures stdout and stderr, and returns parsed results to the API layer.

The browser page in `templates/index.html` calls the API endpoints, shows adapter status, renders discovered and paired device tables, and provides action buttons for each device.

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
- `GET /api/devices`
- `GET /api/paired`
- `POST /api/scan/start`
- `POST /api/scan/stop`
- `POST /api/pair`
- `POST /api/trust`
- `POST /api/connect`
- `POST /api/disconnect`
- `POST /api/remove`

The device action endpoints expect a JSON body like:

```json
{
  "mac": "AA:BB:CC:DD:EE:FF"
}
```

## Running It

Example:

```bash
python BluetoothServer.py
```

Then open:

```text
http://<raspberry-pi-hostname-or-ip>:50014/
```

## Optional Environment Variables

You can override the default bind settings with:

```bash
export ARFBOTOS_BT_HOST=0.0.0.0
export ARFBOTOS_BT_PORT=50014
export ARFBOTOS_BT_DEBUG=false
python3 BluetoothServer.py
```

## Notes

- This version uses `bluetoothctl` rather than the BlueZ DBus API to keep the first implementation simple.
- The wrapper validates MAC address format before pair, trust, connect, disconnect, and remove actions.
- The service is designed to be a standalone backend so a future CODESYS HMI page can call the same endpoints.
- This is an MVP scaffold. The current parsing is intentionally conservative and will likely need refinement after testing on the Pi with real devices.
