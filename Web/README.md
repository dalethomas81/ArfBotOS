# ArfBot web

One Flask app on port 5000 that hosts both operator pages:

| URL | Page |
| --- | --- |
| `http://<pi>:5000/vision` | Vision template capture |
| `http://<pi>:5000/vision/files` | Saved templates |
| `http://<pi>:5000/bluetooth` | Bluetooth pairing (DualSense and other adapters) |
| `http://<pi>:5000/vision/output_sized?width=365&height=255` | HMI vision result image |

`/` redirects to `/vision` when vision is enabled, or `/bluetooth` on a `--plc-only` install. Old `/template`, `/files/`, and `/output_sized` URLs redirect to the new paths.

The CODESYS HMI loads `/vision/output_sized`. Template files still go to `/var/opt/codesys/PlcLogic/Application/Vision/Templates`. The latest processed image is still `/var/opt/codesys/PlcLogic/visu/outputimage.jpg`.

Theme tokens match **ArfBot Night** in `Codesys/VisualizationStyles/ArfBot Night/`.

## Layout

```
Web/
  wsgi.py                 Flask entry (FLASK_APP=wsgi.py)
  config.py
  app/
    __init__.py           factory; lazy-registers blueprints
    vision.py             camera / templates / /vision/output_sized
    bluetooth.py          pairing API under /bluetooth/api/...
    bluetoothctl_wrapper.py
    static/css/arfbot-night.css
    templates/
  tests/
```

`picamera2` is imported only inside the vision capture path. `--plc-only` starts this app with vision disabled so Flask never imports the camera stack.

## Environment

| Variable | Default | Role |
| --- | --- | --- |
| `ARFBOT_ENABLE_VISION` | `1` | Register vision routes (`/vision`, `/vision/output_sized`, templates) |
| `ARFBOT_ENABLE_BLUETOOTH` | `1` | Register `/bluetooth` |
| `ARFBOT_TEMPLATE_DIR` | `/var/opt/codesys/PlcLogic/Application/Vision/Templates` | Saved crops |
| `ARFBOT_VISU_OUTPUT` | `/var/opt/codesys/PlcLogic/visu/outputimage.jpg` | HMI image |

If Bluetooth is on and vision is off, `/` redirects to `/bluetooth`.

## Deploy

The Pi installer copies this folder to:

```text
/var/opt/codesys/PlcLogic/Application/Web
```

and runs it as `VisionWeb.service` on `0.0.0.0:5000`.

The older `OpenCV/VisionWebServer` tree is no longer deployed. Keep it only as history.

## Bluetooth pairing

`Power On` and `Scan` call `rfkill unblock bluetooth` then `bluetoothctl power on` so Lite images that ship `off-blocked` work from the browser. DualSense CLI pairing in the wiki remains a fallback.

API (all under `/bluetooth`):

- `GET /api/status`
- `GET /api/devices`
- `GET /api/paired`
- `POST /api/scan/start` `/api/scan/stop`
- `POST /api/power/on` `/api/power/off`
- `POST /api/setup` (pair + trust + connect)
- `POST /api/pair` `/api/trust` `/api/connect` `/api/disconnect` `/api/remove`

JSON body for device actions: `{"mac":"AA:BB:CC:DD:EE:FF"}`.

## Tests

From this folder (no Bluetooth hardware required):

```bash
python3 -m unittest tests.test_bluetoothctl_wrapper -v
```
