import json
import math
import queue
import threading
import time
import unittest
from dataclasses import replace
from pathlib import Path
from unittest.mock import patch
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import websocket

from scan_web.detection import DetectionConfig, detect, load_config, largest_obstacle
from scan_web.protocol import parse_scan
from scan_web.server import make_server
from scan_web.service import ScanService


def envelope(**changes):
    item = {
        "rame_id": "scan_link", "name": "scan", "scan": [1000, 2000, 3000],
        "angle_start": 0.0, "increment": math.pi / 2,
        "range_min": 0.03, "range_max": 50.0,
        "laserOffset": [0.94, 0, -0.017], "timeoutSec": 0,
    }
    item.update(changes)
    return {"channel": "ros", "op": "publish", "topic": "/scan", "msg": [item]}


class ProtocolTests(unittest.TestCase):
    def test_units_polar_conversion_and_unapplied_offset(self):
        packet = parse_scan(envelope())
        self.assertEqual(packet.ranges, (1, 2, 3))
        self.assertEqual(packet.laser_offset, (0.94, 0, -0.017))
        for actual, expected in zip(packet.points(), [(1, 0), (0, 2), (-3, 0)]):
            self.assertAlmostEqual(actual[0], expected[0])
            self.assertAlmostEqual(actual[1], expected[1])

    def test_mixed_frames_and_optional_slash(self):
        message = envelope(rame_id="/scan_link")
        message["msg"].insert(0, envelope(rame_id="/scan4_link")["msg"][0])
        self.assertEqual(parse_scan(message, "/scan_link").frame, "/scan_link")
        self.assertIsNone(parse_scan(envelope(rame_id="scan3_link")))

    def test_status_and_unrelated_publications_are_ignored(self):
        for message in [{"channel": "ros", "op": "status"}, [],
                        dict(envelope(), topic="/other"), dict(envelope(), channel="other")]:
            self.assertIsNone(parse_scan(message))

    def test_invalid_distances_are_filtered_and_boundaries_retained(self):
        packet = parse_scan(envelope(scan=[0, -1, None, "bad", True, float("nan"), float("inf"), 29, 30, 50000, 50001]))
        self.assertEqual(packet.ranges[-3:], (0.03, 50, None))
        self.assertEqual(len(packet.points()), 2)
        self.assertEqual(parse_scan(envelope(scan=[0, 0])).points(), [])

    def test_scan_parameters_and_sensor_timeout(self):
        for changes in [{"increment": 0}, {"angle_start": float("nan")}, {"range_min": -1},
                        {"range_max": 0.01}, {"scan": []}, {"timeoutSec": 1},
                        {"increment": True}, {"laserOffset": [1]}, {"timeoutSec": -1}]:
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                parse_scan(envelope(**changes))
        message = envelope()
        del message["msg"][0]["increment"]
        with self.assertRaisesRegex(ValueError, "increment"):
            parse_scan(message)


class DetectionTests(unittest.TestCase):
    def test_largest_area_distance_and_center_ties(self):
        def box(x, y, area=1, distance=2):
            return dict(min_x=x, max_x=x + area, min_y=y, max_y=y + 1,
                        center_x=x + area / 2, center_y=y + .5,
                        width=1, depth=area, distance=distance, count=8)
        small = box(0, 0, .5, 1)
        far = box(3, 0)
        near = box(2, 0, distance=1.5)
        tied = box(-2, -1, distance=1.5)
        self.assertIsNone(largest_obstacle([]))
        self.assertEqual(largest_obstacle([small]), small)
        self.assertEqual(largest_obstacle([small, far]), far)
        self.assertEqual(largest_obstacle([far, near]), near)
        for sequence in ([far, near, tied], [tied, near, far]):
            self.assertEqual(largest_obstacle(sequence), tied)

    def setUp(self):
        self.config = DetectionConfig()
        self.block = [(1 + column * 0.05, -0.05 + row * 0.05)
                      for column in range(3) for row in range(3)]

    def test_clusters_boxes_noise_and_roi(self):
        second = [(point_x + 1, point_y + 0.5) for point_x, point_y in self.block]
        result = detect(self.block + second + [(4, 0), (100, 0)], self.config)
        self.assertEqual(result["roi_count"], 19)
        self.assertEqual(len(result["objects"]), 2)
        first = result["objects"][0]
        self.assertEqual(first["count"], 9)
        self.assertAlmostEqual(first["width"], 0.1)
        self.assertAlmostEqual(first["depth"], 0.1)
        self.assertAlmostEqual(first["distance"], 1)

    def test_size_count_distance_filters_and_empty_roi(self):
        for changes in [{"min_width": 0.2}, {"min_depth": 0.2}, {"min_cluster_points": 10},
                        {"max_cluster_points": 8}, {"cluster_distance": 0.01}, {"min_x": 2}]:
            with self.subTest(changes=changes):
                self.assertEqual(detect(self.block, self.config.updated(changes))["objects"], [])
        self.assertEqual(detect([], self.config), {"roi_count": 0, "objects": []})

    def test_neighbors_across_cell_and_negative_coordinate_boundaries(self):
        points = [(0.99 + column * 0.05, -0.13 + row * 0.05)
                  for column in range(3) for row in range(3)]
        self.assertEqual(len(detect(points, self.config)["objects"]), 1)

    def test_load_ros_config_and_validate_atomic_updates(self):
        loaded = load_config(Path(__file__).resolve().parents[1] / "config" / "detection.yaml")
        self.assertEqual(loaded, self.config)
        for changes in [{"min_x": 6}, {"cluster_distance": 0}, {"max_width": -1},
                        {"min_cluster_points": 1.5}, {"min_cluster_points": True},
                        {"cloud_timeout": float("nan")}, {"min_z": 1}]:
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                loaded.updated(changes)


class StateTests(unittest.TestCase):
    def test_largest_and_unsupported_telemetry_lifecycle(self):
        state = self.service.state()
        self.assertIsNone(state['largest_obstacle'])
        self.assertEqual(state['ros_publication']['status'], 'unsupported')
        self.assertIsNone(state['ros_publication']['message'])
        self.service.update_config({'min_depth': 0})
        self.service.accept(self.packet)
        self.service.process_latest()
        state = self.service.state()
        self.assertEqual(state['largest_obstacle'], largest_obstacle(state['objects']))
        self.assertIsNotNone(state['largest_obstacle'])
        self.service.update_config({'min_x': -3, 'max_x': -1})
        self.assertIsNone(self.service.state()['largest_obstacle'])
        self.service.process_latest()
        self.assertEqual(self.service.state()['status'], 'clear')
        self.assertIsNone(self.service.state()['largest_obstacle'])
        self.service.update_config({'min_x': .3, 'max_x': 5})
        self.service.process_latest()
        self.now += 1.01
        self.assertIsNone(self.service.state()['largest_obstacle'])
        self.service.accept(self.packet)
        self.assertIsNotNone(self.service.state()['largest_obstacle'])
        self.service.fail('lost', 'disconnected')
        self.assertIsNone(self.service.state()['largest_obstacle'])

    def setUp(self):
        self.now = 100.0
        self.service = ScanService(DetectionConfig(), clock=lambda: self.now)
        self.packet = parse_scan(envelope(scan=[1000] * 20, angle_start=-0.2, increment=0.02))

    def test_no_data_clear_detected_timeout_and_recovery(self):
        self.assertEqual(self.service.state()["status"], "waiting")
        self.service.accept(self.packet)
        self.service.process_latest()
        self.assertEqual(self.service.state()["status"], "clear")
        self.service.update_config({"min_depth": 0})
        self.service.process_latest()
        self.assertEqual(self.service.state()["status"], "detected")
        self.now += 1.01
        stale = self.service.state()
        self.assertEqual(stale["status"], "timeout")
        self.assertEqual(stale["objects"], [])
        self.assertFalse(stale["detected"])
        self.service.accept(self.packet)
        self.assertEqual(self.service.state()["status"], "detected")

    def test_duplicate_reception_refreshes_age_without_reprocessing(self):
        self.service.accept(self.packet)
        self.service.process_latest()
        self.now += 0.9
        self.service.accept(self.packet)
        self.service.process_latest()
        self.now += 0.9
        state = self.service.state()
        self.assertEqual(state["status"], "clear")
        self.assertEqual(state["content_version"], 1)
        self.assertEqual(state["processed_count"], 1)
        self.assertAlmostEqual(state["receive_age_s"], 0.9)
        self.assertGreater(state["content_age_s"], 1)

    def test_invalid_data_clears_candidates_immediately(self):
        self.service.update_config({"min_depth": 0})
        self.service.accept(self.packet)
        self.service.process_latest()
        self.assertTrue(self.service.state()["detected"])
        self.service.fail("雷达报告超时: timeoutSec=1")
        self.assertEqual(self.service.state()["status"], "invalid")
        self.assertEqual(self.service.state()["objects"], [])

    def test_show_last_completed_analysis_while_latest_is_pending(self):
        self.service.accept(self.packet)
        self.service.process_latest()
        self.now += 0.1
        self.service.accept(replace(self.packet, ranges=(2.0,) * 20))
        state = self.service.state()
        self.assertEqual(state["status"], "clear")
        self.assertEqual(state["analyzed_content_version"], 1)
        self.assertEqual(state["content_version"], 2)
        self.assertAlmostEqual(math.hypot(*state["points"][0]), 1)
        self.now += 1
        self.service.accept(replace(self.packet, ranges=(3.0,) * 20))
        self.assertEqual(self.service.state()["status"], "processing")
        self.assertEqual(self.service.state()["objects"], [])

    def test_latest_scan_only_and_config_change_during_analysis(self):
        self.service.accept(self.packet)
        second = replace(self.packet, ranges=(2.0,) * 20)
        self.service.accept(second)
        self.service.process_latest()
        self.assertEqual(self.service.state()["processed_count"], 1)
        self.assertAlmostEqual(math.hypot(*self.service.state()["points"][0]), 2)
        self.service.update_config({"max_x": 4})
        with patch("scan_web.service.detect", side_effect=lambda points, config:
                   (self.service.update_config({"max_x": 3}) and detect(points, config))):
            self.service.process_latest()
        self.assertEqual(self.service.state()["objects"], [])
        self.assertEqual(self.service.state()["processed_count"], 1)
        self.service.process_latest()
        self.assertEqual(self.service.state()["processed_count"], 2)

    def test_new_scan_during_analysis_does_not_starve_publication(self):
        self.service.accept(self.packet)
        newer = replace(self.packet, ranges=(2.0,) * 20)
        real_detect = detect

        def detect_with_new_packet(points, config):
            self.service.accept(newer)
            return real_detect(points, config)

        with patch("scan_web.service.detect", side_effect=detect_with_new_packet):
            self.service.process_latest()
        self.assertEqual(self.service.state()["analyzed_content_version"], 1)
        self.assertEqual(self.service.state()["content_version"], 2)
        self.service.process_latest()
        self.assertEqual(self.service.state()["analyzed_content_version"], 2)

    def test_connection_loss_during_analysis_discards_old_result(self):
        self.service.accept(self.packet)

        def detect_after_connection_loss(points, config):
            self.service.fail("lost", "disconnected")
            self.service.accept(self.packet)
            return detect(points, config)

        with patch("scan_web.service.detect", side_effect=detect_after_connection_loss):
            self.service.process_latest()
        self.assertEqual(self.service.state()["processed_count"], 0)
        self.assertEqual(self.service.state()["objects"], [])


class FakeSocket:
    def __init__(self, messages=()):
        self.messages = queue.Queue()
        for message in messages:
            self.messages.put(message)
        self.sent = []
        self.closed = False

    def send(self, message):
        self.sent.append(json.loads(message))

    def settimeout(self, timeout):
        self.timeout = timeout

    def recv(self):
        try:
            result = self.messages.get(timeout=min(self.timeout, 0.02))
        except queue.Empty as error:
            raise websocket.WebSocketTimeoutException() from error
        if isinstance(result, Exception):
            raise result
        return json.dumps(result)

    def shutdown(self):
        self.messages.put(ConnectionError("closed"))

    def close(self, timeout=0.2):
        self.closed = True


class ConnectionTests(unittest.TestCase):
    def wait_until(self, predicate, timeout=2):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.01)
        self.fail("等待测试条件超时")

    def test_subscribe_disconnect_reconnect_and_invalid_recovery(self):
        first = FakeSocket([{"op": "status"}, ConnectionError("lost")])
        second = FakeSocket([envelope(timeoutSec=1)])
        sockets = [first, second]

        def connect(*arguments, **options):
            return sockets.pop(0)

        service = ScanService(DetectionConfig(), connection_factory=connect, retry_delays=(0.01,))
        service.start()
        try:
            self.wait_until(lambda: service.state()["status"] == "invalid")
            self.assertEqual(first.sent, [{"channel": "ros", "op": "subscribe_scan", "topic": "/scan"}])
            self.assertTrue(first.closed)
            second.messages.put(envelope())
            self.wait_until(lambda: service.state()["status"] == "clear")
            self.assertEqual(service.state()["reconnects"], 1)
        finally:
            service.close()
        self.assertTrue(all(not thread.is_alive() for thread in service.threads))

    def test_other_sensor_traffic_does_not_keep_target_alive(self):
        socket = FakeSocket([envelope(rame_id="scan4_link")] * 20)
        service = ScanService(DetectionConfig(cloud_timeout=0.05), connection_factory=lambda *args, **kwargs: socket,
                              retry_delays=(1,))
        service.start()
        try:
            self.wait_until(lambda: service.state()["status"] == "disconnected")
            self.assertIn("超时", service.state()["error"])
            self.assertEqual(service.state()["received_count"], 0)
        finally:
            service.close()


class HttpTests(unittest.TestCase):
    def setUp(self):
        self.service = ScanService(DetectionConfig())
        self.server = make_server(self.service, port=0)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.url = f"http://127.0.0.1:{self.server.server_port}"

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)

    def request(self, path, data=None, headers=None):
        request = Request(self.url + path, data=data, headers=headers or {})
        return urlopen(request, timeout=2)

    def test_assets_state_and_config(self):
        for route, content_type in [("/", "text/html"), ("/app.js", "text/javascript"), ("/style.css", "text/css")]:
            with self.request(route) as response:
                self.assertEqual(response.status, 200)
                self.assertIn(content_type, response.headers["Content-Type"])
                self.assertGreater(len(response.read()), 100)
        with self.request("/api/state") as response:
            self.assertEqual(json.load(response)["status"], "waiting")
        with self.request("/api/config", b'{"min_x": -2}', {"Content-Type": "application/json"}) as response:
            self.assertEqual(json.load(response)["min_x"], -2)
        with self.request("/api/config") as response:
            self.assertEqual(json.load(response)["min_x"], -2)

    def test_invalid_updates_do_not_change_config(self):
        before = self.service.get_config()
        for body in [b'{"max_x": -1}', b'{"cloud_timeout": NaN}', b'[]', b'{bad', b'{"min_cluster_points": 1.2}']:
            with self.assertRaises(HTTPError) as caught:
                self.request("/api/config", body, {"Content-Type": "application/json"})
            self.assertEqual(caught.exception.code, 400)
            self.assertEqual(self.service.get_config(), before)

    def test_unknown_routes_and_cross_origin_update(self):
        with self.assertRaises(HTTPError) as caught:
            self.request("/api/config", b'{}', {"Content-Type": "application/json", "Origin": "http://elsewhere.test"})
        self.assertEqual(caught.exception.code, 403)
        with self.assertRaises(HTTPError) as caught:
            self.request("/../requirements.txt")
        self.assertEqual(caught.exception.code, 404)


if __name__ == "__main__":
    unittest.main()
