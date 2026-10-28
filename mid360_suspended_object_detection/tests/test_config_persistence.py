import json
import tempfile
import threading
import unittest
from concurrent.futures import ThreadPoolExecutor
from dataclasses import asdict
from pathlib import Path
from unittest.mock import patch
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import yaml
from scan_web.detection import DetectionConfig, load_config
from scan_web.server import make_server
from scan_web.service import ScanService


class PersistenceTests(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.path = Path(self.folder.name) / "config.yaml"
        self.path.write_text(yaml.safe_dump({"mid360_suspended_object_detection": {
            "ros__parameters": dict(asdict(DetectionConfig()), min_z=1.2, expected_frame="unchanged")},
            "other": {"keep": [1, 2]}}), encoding="utf-8")
        self.service = ScanService(load_config(self.path), config_path=self.path)

    def test_roundtrip_memory_only_and_unrelated_values(self):
        before = self.path.read_bytes()
        self.service.update_config({"min_x": -2})
        self.assertEqual(self.path.read_bytes(), before)
        saved = self.service.update_config({"max_x": 7.25, "min_y": -3, "max_y": 2.75,
            "cluster_distance": .2, "min_cluster_points": 12, "max_cluster_points": 5000,
            "min_width": .1, "max_width": 3.5, "min_depth": .15, "max_depth": 4.5,
            "cloud_timeout": 2.5}, persist=True)
        self.assertEqual(asdict(load_config(self.path)), saved)
        document = yaml.safe_load(self.path.read_text())
        self.assertEqual(document["other"], {"keep": [1, 2]})
        self.assertEqual(document["mid360_suspended_object_detection"]["ros__parameters"]["min_z"], 1.2)
        self.assertIsInstance(document["mid360_suspended_object_detection"]["ros__parameters"]["min_cluster_points"], int)

    def test_invalid_and_replace_failure_leave_disk_and_memory_unchanged(self):
        before, config = self.path.read_bytes(), self.service.get_config()
        with self.assertRaises(ValueError):
            self.service.update_config({"min_x": 100}, persist=True)
        with patch("scan_web.detection.os.replace", side_effect=PermissionError("read-only test")):
            with self.assertRaises(PermissionError):
                self.service.update_config({"min_x": -2}, persist=True)
        self.assertEqual(self.path.read_bytes(), before)
        self.assertEqual(self.service.get_config(), config)
        self.assertEqual(list(self.path.parent.glob(".scan-web-*")), [])

    def test_symlink_updates_target(self):
        link = self.path.parent / "installed.yaml"
        try:
            link.symlink_to(self.path)
        except OSError as error:
            self.skipTest(str(error))
        service = ScanService(load_config(link), config_path=link)
        service.update_config({"min_x": -4}, persist=True)
        self.assertTrue(link.is_symlink())
        self.assertEqual(load_config(self.path).min_x, -4)

    def test_concurrent_partial_saves_merge_and_disk_matches_memory(self):
        with ThreadPoolExecutor(max_workers=4) as pool:
            list(pool.map(lambda i: self.service.update_config({"max_x": 5 + i}, persist=True), range(20)))
        self.assertEqual(asdict(load_config(self.path)), self.service.get_config())
        self.assertEqual(list(self.path.parent.glob(".scan-web-*")), [])

    def test_http_save_and_failure(self):
        server = make_server(self.service, port=0)
        thread = threading.Thread(target=server.serve_forever)
        thread.start()
        try:
            def post(body):
                return urlopen(Request(f"http://127.0.0.1:{server.server_port}/api/config/save",
                                       data=json.dumps(body).encode(), headers={"Content-Type": "application/json"}), timeout=5)
            with post({"min_x": -2}) as response:
                self.assertEqual(json.load(response), asdict(load_config(self.path)))
            with patch("scan_web.detection.os.replace", side_effect=OSError("disk failure")):
                with self.assertRaises(HTTPError) as caught:
                    post({"min_x": -3})
                self.assertEqual(caught.exception.code, 500)
            self.assertEqual(self.service.get_config()["min_x"], -2)
        finally:
            server.shutdown()
            thread.join()
            server.server_close()
