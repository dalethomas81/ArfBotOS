import os

basedir = os.path.abspath(os.path.dirname(__file__))


def env_flag(name, default=True):
    raw = os.environ.get(name)
    if raw is None:
        return default
    return raw.strip().lower() in {"1", "true", "yes", "on"}


class Config(object):
    SECRET_KEY = os.environ.get("SECRET_KEY") or "arfbot-web"
    MAX_CONTENT_LENGTH = 16 * 1000 * 1000
    JSON_SORT_KEYS = False
    UPLOAD_FOLDER = os.environ.get(
        "ARFBOT_TEMPLATE_DIR",
        "/var/opt/codesys/PlcLogic/Application/Vision/Templates",
    )
    VISU_OUTPUT = os.environ.get(
        "ARFBOT_VISU_OUTPUT",
        "/var/opt/codesys/PlcLogic/visu/outputimage.jpg",
    )
