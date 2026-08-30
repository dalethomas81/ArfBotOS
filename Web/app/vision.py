import io
import os

import cv2
from flask import (
    Blueprint,
    Response,
    abort,
    current_app,
    flash,
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
