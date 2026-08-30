from config import Config, env_flag


def create_app(config_class=Config):
    from flask import Flask, redirect

    app = Flask(__name__)
    app.config.from_object(config_class)

    enable_vision = env_flag("ARFBOT_ENABLE_VISION", True)
    enable_bluetooth = env_flag("ARFBOT_ENABLE_BLUETOOTH", True)
    app.config["ENABLE_VISION"] = enable_vision
    app.config["ENABLE_BLUETOOTH"] = enable_bluetooth

    if enable_vision:
        from .vision import bp as vision_bp

        app.register_blueprint(vision_bp)

    if enable_bluetooth:
        from .bluetooth import bp as bluetooth_bp

        app.register_blueprint(bluetooth_bp)

    @app.route("/")
    def home():
        if enable_vision:
            return redirect("/vision")
        if enable_bluetooth:
            return redirect("/bluetooth")
        return "ArfBot web is running, but no pages are enabled.", 404

    @app.context_processor
    def inject_nav():
        return {
            "enable_vision": enable_vision,
            "enable_bluetooth": enable_bluetooth,
        }

    return app
