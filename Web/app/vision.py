import io
import os
import re

import cv2
from flask import (
    Blueprint,
    Response,
    abort,
    current_app,
    flash,
    jsonify,
    redirect,
    render_template,
    request,
    send_file,
    send_from_directory,
    url_for,
)
from werkzeug.utils import secure_filename

bp = Blueprint("vision", __name__)

ALLOWED_EXTENSIONS = {"png", "jpg", "jpeg"}

croppedImage = None
buffer = None
frame = None
capArray = None


def allowed_file(filename):
    return "." in filename and filename.rsplit(".", 1)[1].lower() in ALLOWED_EXTENSIONS


def upload_folder():
    return current_app.config["UPLOAD_FOLDER"]


def visu_output_path():
    return current_app.config["VISU_OUTPUT"]


DEFAULT_ROI_TL = [0.0, 0.0]
DEFAULT_ROI_BR = [640.0, 400.0]
_ROI_DATA_RE = re.compile(
    r"data:\s*\[\s*([-+0-9.eE]+)\s*,\s*([-+0-9.eE]+)",
)


def roi_file_path():
    configured = current_app.config.get("ROI_FILE")
    if configured:
        return configured
    return os.path.join(os.path.dirname(upload_folder()), "roi.yaml")


def _parse_roi_yaml(text):
    def vec_after(key):
        idx = text.find(key)
        if idx < 0:
            raise ValueError("missing %s" % key)
        match = _ROI_DATA_RE.search(text[idx:idx + 400])
        if not match:
            raise ValueError("no data for %s" % key)
        return [float(match.group(1)), float(match.group(2))]

    return vec_after("top_left"), vec_after("bot_right")


def read_roi(path=None):
    path = path or roi_file_path()
    if not os.path.isfile(path):
        return list(DEFAULT_ROI_TL), list(DEFAULT_ROI_BR)
    with open(path, "r") as handle:
        return _parse_roi_yaml(handle.read())


def write_roi(top_left, bot_right, path=None):
    path = path or roi_file_path()
    folder = os.path.dirname(path)
    if folder and not os.path.exists(folder):
        os.makedirs(folder)
    body = (
        "%YAML:1.0\n"
        "---\n"
        "top_left: !!opencv-matrix\n"
        "   rows: 2\n"
        "   cols: 1\n"
        "   dt: f\n"
        "   data: [ {0:.2f}, {1:.2f} ]\n"
        "bot_right: !!opencv-matrix\n"
        "   rows: 2\n"
        "   cols: 1\n"
        "   dt: f\n"
        "   data: [ {2:.2f}, {3:.2f} ]\n"
    ).format(top_left[0], top_left[1], bot_right[0], bot_right[1])
    tmp = path + ".tmp"
    with open(tmp, "w") as handle:
        handle.write(body)
    os.replace(tmp, path)


def _as_xy(value, name):
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise ValueError("%s must be [x, y]" % name)
    return float(value[0]), float(value[1])


def normalize_roi(top_left, bot_right, max_w=None, max_h=None):
    x0, y0 = _as_xy(top_left, "top_left")
    x1, y1 = _as_xy(bot_right, "bot_right")
    left, right = sorted((x0, x1))
    top, bottom = sorted((y0, y1))
    if max_w is None or max_h is None:
        global capArray
        if capArray is not None:
            max_h, max_w = capArray.shape[:2]
        else:
            max_w, max_h = 5000, 5000
    left = max(0.0, min(left, max_w - 1))
    top = max(0.0, min(top, max_h - 1))
    right = max(left + 1.0, min(right, float(max_w)))
    bottom = max(top + 1.0, min(bottom, float(max_h)))
    if right - left < 8 or bottom - top < 8:
        raise ValueError("ROI is too small")
    return [left, top], [right, bottom]


@bp.route("/upload", methods=["GET", "POST"])
def upload_file():
    if request.method == "POST":
        if "file" not in request.files:
            flash("No file part")
            return redirect(request.url)
        file = request.files["file"]
        if file.filename == "":
            flash("No selected file")
            return redirect(request.url)
        if file and allowed_file(file.filename):
            folder = upload_folder()
            if not os.path.exists(folder):
                os.makedirs(folder)
            filename = secure_filename(file.filename)
            file.save(os.path.join(folder, filename))
            return redirect(url_for("vision.download_file", name=filename))
    return """
    <!doctype html>
    <title>Upload new File</title>
    <h1>Upload new File</h1>
    <form method=post enctype=multipart/form-data>
      <input type=file name=file>
      <input type=submit value=Upload>
    </form>
    """


@bp.route("/uploads/<name>")
def download_file(name):
    return send_from_directory(upload_folder(), name)


@bp.route("/save_template")
@bp.route("/save_template/<filename>")
def save_template(filename=None):
    global croppedImage
    folder = upload_folder()
    if not os.path.exists(folder):
        os.makedirs(folder)
    cv2.imwrite(os.path.join(folder, filename), croppedImage)
    data = {"status": "saved"}
    return data, 200


@bp.route("/delete_template")
@bp.route("/delete_template/<filename>")
def delete_template(filename=None):
    path = os.path.join(upload_folder(), filename)
    if os.path.exists(path):
        os.remove(path)
    data = {"status": "deleted"}
    return data, 200


@bp.route("/vision/files")
@bp.route("/vision/files/")
def files():
    folder = upload_folder()
    if not os.path.exists(folder):
        os.makedirs(folder)
    filenames = os.listdir(folder)
    return render_template("vision/saved_templates.html", title="Templates", files=filenames)


@bp.route("/vision/files/<path:filename>")
def file(filename):
    return send_from_directory(upload_folder(), filename, as_attachment=False)


@bp.route("/files/")
def files_legacy():
    return redirect(url_for("vision.files"))


@bp.route("/files/<path:filename>")
def file_legacy(filename):
    return redirect(url_for("vision.file", filename=filename))


@bp.route("/output")
def output():
    path = visu_output_path()
    return send_from_directory(
        os.path.dirname(path),
        os.path.basename(path),
        as_attachment=False,
    )


# http://ArfBot:5000/vision/output_sized?width=320&height=200
@bp.route("/vision/output_sized")
def output_sized():
    width = request.args.get("width", default=640, type=int)
    height = request.args.get("height", default=400, type=int)
    image_path = visu_output_path()
    image = cv2.imread(image_path)
    if image is None:
        abort(404)
    resized_image = cv2.resize(image, (width, height))
    _, encoded = cv2.imencode(".jpg", resized_image)
    img_io = io.BytesIO(encoded)
    return send_file(img_io, mimetype="image/jpeg")


@bp.route("/output_sized")
def output_sized_legacy():
    target = url_for("vision.output_sized")
    query = request.query_string.decode()
    if query:
        target = "{0}?{1}".format(target, query)
    return redirect(target)


@bp.route("/vision")
@bp.route("/vision/")
def template():
    return render_template("vision/template.html", title="Vision")


@bp.route("/template")
def template_legacy():
    return redirect(url_for("vision.template"))


@bp.route("/vision/roi", methods=["GET", "POST"])
def roi():
    if request.method == "GET":
        top_left, bot_right = read_roi()
        return jsonify({
            "top_left": top_left,
            "bot_right": bot_right,
            "path": roi_file_path(),
        })

    payload = request.get_json(silent=True) or {}
    try:
        top_left, bot_right = normalize_roi(
            payload.get("top_left"),
            payload.get("bot_right"),
        )
        write_roi(top_left, bot_right)
    except (TypeError, ValueError) as exc:
        return jsonify({"status": "error", "error": str(exc)}), 400
    return jsonify({
        "status": "saved",
        "top_left": top_left,
        "bot_right": bot_right,
        "path": roi_file_path(),
    })


@bp.route("/captured_image")
def captured_image():
    width = request.args.get("width", default=640, type=int)
    height = request.args.get("height", default=400, type=int)
    return Response(gen_frames(width, height), mimetype="multipart/x-mixed-replace; boundary=frame")


@bp.route("/cropped")
def cropped():
    top_left = request.args.get("top_left")
    bot_right = request.args.get("bot_right")
    return Response(crop_template(top_left, bot_right), mimetype="multipart/x-mixed-replace; boundary=frame")


def crop_template(top_left=None, bot_right=None):
    global croppedImage, buffer, frame, capArray
    try:
        _tl = top_left.split(",")
        _br = bot_right.split(",")
        _tla = list(map(int, _tl))
        _bra = list(map(int, _br))
        croppedImage = crop_image(capArray, _tla, _bra)
        ret, buffercropped = cv2.imencode(".jpg", croppedImage)
        frame = buffercropped.tobytes()
        yield (b"--frame\r\n"
               b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
    except Exception:
        pass


def crop_image(image, top_left, bot_right):
    y = int(top_left[1])
    x = int(top_left[0])
    h = int(bot_right[1] - top_left[1])
    w = int(bot_right[0] - top_left[0])
    cropped = image[y:y + h, x:x + w].copy()
    return cropped


def gen_frames(width, height):
    global buffer, frame, capArray
    from picamera2 import Picamera2

    with Picamera2() as camera:
        config = camera.create_preview_configuration(
            main={"size": (int(width), int(height)), "format": "RGB888"}
        )
        camera.configure(config)
        camera.start()
        import time
        time.sleep(0.1)
        capArray = camera.capture_array()
        camera.stop()
        camera.close()

        try:
            ret, buffer = cv2.imencode(".jpg", capArray)
            frame = buffer.tobytes()
            yield (b"--frame\r\n"
                   b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
        except Exception:
            pass
