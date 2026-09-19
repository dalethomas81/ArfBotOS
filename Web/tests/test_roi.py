from __future__ import annotations

import os
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def _stub_cv2():
    cv2 = types.ModuleType("cv2")
    cv2.imread = lambda *args, **kwargs: None
    cv2.imwrite = lambda *args, **kwargs: None
    cv2.imencode = lambda *args, **kwargs: (True, b"")
    cv2.resize = lambda *args, **kwargs: None
    sys.modules.setdefault("cv2", cv2)


class RoiApiTests(unittest.TestCase):
    def setUp(self):
        try:
            import flask  # noqa: F401
        except ImportError:
            self.skipTest("flask not installed")
        _stub_cv2()
        self.tmp = tempfile.TemporaryDirectory()
        self.roi_path = os.path.join(self.tmp.name, "roi.yaml")
        with patch.dict(os.environ, {
            "ARFBOT_ENABLE_VISION": "1",
            "ARFBOT_ENABLE_BLUETOOTH": "0",
            "ARFBOT_ENABLE_ANIMATOR": "0",
            "ARFBOT_ROI_FILE": self.roi_path,
        }):
            from app import create_app

            self.app = create_app()
        self.app.config["ROI_FILE"] = self.roi_path
        self.client = self.app.test_client()

    def tearDown(self):
        self.tmp.cleanup()

    def test_get_default_when_missing(self):
        response = self.client.get("/vision/roi")
        self.assertEqual(response.status_code, 200)
        data = response.get_json()
        self.assertEqual(data["top_left"], [0.0, 0.0])
        self.assertEqual(data["bot_right"], [640.0, 400.0])

    def test_post_writes_opencv_yaml(self):
        response = self.client.post("/vision/roi", json={
            "top_left": [40, 20],
            "bot_right": [400, 300],
        })
        self.assertEqual(response.status_code, 200)
        data = response.get_json()
        self.assertEqual(data["status"], "saved")
        self.assertEqual(data["top_left"], [40.0, 20.0])
        self.assertEqual(data["bot_right"], [400.0, 300.0])
        self.assertTrue(os.path.isfile(self.roi_path))
        text = Path(self.roi_path).read_text()
        self.assertIn("top_left: !!opencv-matrix", text)
        self.assertIn("data: [ 40.00, 20.00 ]", text)
        self.assertIn("data: [ 400.00, 300.00 ]", text)

        again = self.client.get("/vision/roi")
        loaded = again.get_json()
        self.assertEqual(loaded["top_left"], [40.0, 20.0])
        self.assertEqual(loaded["bot_right"], [400.0, 300.0])

    def test_post_rejects_tiny_roi(self):
        response = self.client.post("/vision/roi", json={
            "top_left": [10, 10],
            "bot_right": [12, 12],
        })
        self.assertEqual(response.status_code, 400)

    def test_post_normalizes_reversed_corners(self):
        response = self.client.post("/vision/roi", json={
            "top_left": [400, 300],
            "bot_right": [40, 20],
        })
        self.assertEqual(response.status_code, 200)
        data = response.get_json()
        self.assertEqual(data["top_left"], [40.0, 20.0])
        self.assertEqual(data["bot_right"], [400.0, 300.0])

    def test_vision_page_has_roi_mode(self):
        page = self.client.get("/vision")
        self.assertEqual(page.status_code, 200)
        html = page.get_data(as_text=True)
        self.assertIn('id="mode-roi"', html)
        self.assertIn('id="save-roi"', html)
        self.assertIn("/vision/roi", html)
        self.assertIn("Capture size vs field of view", html)
        self.assertIn("1332", html)


if __name__ == "__main__":
    unittest.main()
