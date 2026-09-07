from pathlib import Path

from flask import Blueprint, Response, abort, current_app, render_template

bp = Blueprint("animator", __name__, url_prefix="/animator")


def element_wrapper_path():
    here = Path(__file__).resolve()
    repo_js = here.parents[2] / "Codesys" / "Html5Controls" / "RobotAnimator" / "ElementWrapper.js"
    if repo_js.is_file():
        return repo_js
    static_js = Path(current_app.static_folder) / "js" / "ElementWrapper.js"
    if static_js.is_file():
        return static_js
    return None


@bp.route("")
@bp.route("/")
def index():
    return render_template("animator/index.html", title="Animator")


@bp.route("/element-wrapper.js")
def element_wrapper_js():
    path = element_wrapper_path()
    if path is None:
        abort(404)
    return Response(path.read_text(encoding="utf-8"), mimetype="application/javascript")
