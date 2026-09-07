# ArfBot web

One Flask app on port 5000 that hosts the operator pages:

| URL | Page |
| --- | --- |
| `http://<pi>:5000/vision` | Vision template capture |
| `http://<pi>:5000/vision/files` | Saved templates |
| `http://<pi>:5000/animator` | 6-axis robot animator (joints, TCP / MCS / PCS, Euler convention) |
| `http://<pi>:5000/bluetooth` | Bluetooth pairing (DualSense and other adapters) |
| `http://<pi>:5000/vision/output_sized?width=365&height=255` | HMI vision result image |

`/` redirects to `/vision` when vision is enabled, `/bluetooth` on a `--plc-only` install, or `/animator` if that is the only enabled page. Old `/template`, `/files/`, and `/output_sized` URLs redirect to the new paths.

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
    animator.py           /animator page; serves ElementWrapper.js
    static/css/arfbot-night.css
    static/js/animator-page.js
    templates/
  tests/
```

`picamera2` is imported only inside the vision capture path. `--plc-only` starts this app with vision disabled so Flask never imports the camera stack.

## Environment

| Variable | Default | Role |
| --- | --- | --- |
| `ARFBOT_ENABLE_VISION` | `1` | Register vision routes (`/vision`, `/vision/output_sized`, templates) |
| `ARFBOT_ENABLE_BLUETOOTH` | `1` | Register `/bluetooth` |
| `ARFBOT_ENABLE_ANIMATOR` | `1` | Register `/animator` (always on in the Pi installer) |
| `ARFBOT_TEMPLATE_DIR` | `/var/opt/codesys/PlcLogic/Application/Vision/Templates` | Saved crops |
| `ARFBOT_VISU_OUTPUT` | `/var/opt/codesys/PlcLogic/visu/outputimage.jpg` | HMI image |

The animator page loads the same `ElementWrapper.js` as the CODESYS HTML5 control (`Codesys/Html5Controls/RobotAnimator/`). From a git checkout it is served from that path. The installer also copies it to `Web/app/static/js/ElementWrapper.js` on the Pi. Sliders drive joints and SoftMotion poses locally; they are not live axis-group values (those stay on the WebVisu control).

If Bluetooth is on and vision is off, `/` redirects to `/bluetooth`. If only the animator is on, `/` redirects to `/animator`.

## Deploy

The Pi installer copies this folder to:

```text
/var/opt/codesys/PlcLogic/Application/Web
```

and runs it as `VisionWeb.service` on `0.0.0.0:5000`.

The older `OpenCV/VisionWebServer` tree is no longer deployed. Keep it only as history.

## Bluetooth pairing

`Power On` and `Scan` call `rfkill unblock bluetooth` then `bluetoothctl power on` so Lite images that ship `off-blocked` work from the browser.

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
python3 -m unittest tests.test_bluetoothctl_wrapper tests.test_app_factory -v
```
