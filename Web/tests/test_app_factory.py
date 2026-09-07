from __future__ import annotations

import os
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from config import env_flag  # noqa: E402


class EnvFlagTests(unittest.TestCase):
    def test_default_true(self):
        with patch.dict(os.environ, {}, clear=False):
            os.environ.pop("ARFBOT_TEST_FLAG", None)
            self.assertTrue(env_flag("ARFBOT_TEST_FLAG", True))
            self.assertFalse(env_flag("ARFBOT_TEST_FLAG", False))

    def test_parses_common_truthy_values(self):
        for value in ("1", "true", "YES", "on"):
            with patch.dict(os.environ, {"ARFBOT_TEST_FLAG": value}):
                self.assertTrue(env_flag("ARFBOT_TEST_FLAG", False), value)

    def test_parses_falsey_values(self):
        for value in ("0", "false", "no", "off"):
            with patch.dict(os.environ, {"ARFBOT_TEST_FLAG": value}):
                self.assertFalse(env_flag("ARFBOT_TEST_FLAG", True), value)


class FactoryTests(unittest.TestCase):
    def test_bluetooth_only_does_not_import_vision(self):
        try:
            import flask  # noqa: F401
        except ImportError:
            self.skipTest("flask not installed")

        with patch.dict(os.environ, {
            "ARFBOT_ENABLE_VISION": "0",
            "ARFBOT_ENABLE_BLUETOOTH": "1",
        }):
            from app import create_app

            application = create_app()
            rules = {rule.rule for rule in application.url_map.iter_rules()}

        self.assertIn("/bluetooth", rules)
        self.assertIn("/bluetooth/api/status", rules)
        self.assertIn("/animator", rules)
        self.assertIn("/animator/element-wrapper.js", rules)
        self.assertNotIn("/output_sized", rules)
        self.assertNotIn("/vision/output_sized", rules)
        self.assertNotIn("/captured_image", rules)
        client = application.test_client()
        response = client.get("/")
        self.assertIn(response.status_code, (301, 302, 308))
        self.assertIn("/bluetooth", response.headers.get("Location", ""))

    def test_full_app_keeps_hmi_output_sized(self):
        try:
            import flask  # noqa: F401
        except ImportError:
            self.skipTest("flask not installed")

        cv2 = types.ModuleType("cv2")
        cv2.imread = lambda *args, **kwargs: None
        cv2.imwrite = lambda *args, **kwargs: None
        cv2.imencode = lambda *args, **kwargs: (True, b"")
        cv2.resize = lambda *args, **kwargs: None
        sys.modules.setdefault("cv2", cv2)

        with patch.dict(os.environ, {
            "ARFBOT_ENABLE_VISION": "1",
            "ARFBOT_ENABLE_BLUETOOTH": "1",
        }):
            from app import create_app

            application = create_app()
            rules = {rule.rule for rule in application.url_map.iter_rules()}

        self.assertIn("/", rules)
        self.assertIn("/vision", rules)
        self.assertIn("/vision/files", rules)
        self.assertIn("/vision/output_sized", rules)
        self.assertIn("/output_sized", rules)
        self.assertIn("/captured_image", rules)
        self.assertIn("/bluetooth", rules)
        self.assertIn("/bluetooth/api/setup", rules)
        self.assertIn("/animator", rules)
        self.assertIn("/animator/", rules)
        self.assertIn("/animator/element-wrapper.js", rules)

        client = application.test_client()
        response = client.get("/")
        self.assertIn(response.status_code, (301, 302, 308))
        self.assertIn("/vision", response.headers.get("Location", ""))


class AnimatorPageTests(unittest.TestCase):
    def _app(self, **flags):
        try:
            import flask  # noqa: F401
        except ImportError:
            self.skipTest("flask not installed")
        env = {
            "ARFBOT_ENABLE_VISION": "0",
            "ARFBOT_ENABLE_BLUETOOTH": "0",
            "ARFBOT_ENABLE_ANIMATOR": "1",
        }
        env.update(flags)
        with patch.dict(os.environ, env):
            from app import create_app

            return create_app()

    def test_animator_page_and_wrapper_js(self):
        application = self._app()
        client = application.test_client()

        page = client.get("/animator")
        self.assertEqual(page.status_code, 200)
        html = page.get_data(as_text=True)
        self.assertIn("Robot Animator", html)
        self.assertIn('id="stage"', html)
        self.assertIn("Animator", html)
        self.assertIn("/animator/element-wrapper.js", html)
        self.assertIn("animator-page.js", html)

        js = client.get("/animator/element-wrapper.js")
        self.assertEqual(js.status_code, 200)
        body = js.get_data(as_text=True)
        self.assertIn("RobotAnimatorElementWrapper", body)
        self.assertIn("attachTo", body)
        self.assertIn("setOriConvention", body)

    def test_animator_can_be_disabled(self):
        application = self._app(ARFBOT_ENABLE_ANIMATOR="0", ARFBOT_ENABLE_BLUETOOTH="1")
        rules = {rule.rule for rule in application.url_map.iter_rules()}
        self.assertNotIn("/animator", rules)
        self.assertIn("/bluetooth", rules)

    def test_home_redirects_to_animator_when_only_page(self):
        application = self._app()
        client = application.test_client()
        response = client.get("/")
        self.assertIn(response.status_code, (301, 302, 308))
        self.assertIn("/animator", response.headers.get("Location", ""))


if __name__ == "__main__":
    unittest.main()
