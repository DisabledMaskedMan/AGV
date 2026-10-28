"""Board-only fixed replay: compare ROS/Web, signed axes, clearing and recovery."""
import argparse
import json
import math
from pathlib import Path
import subprocess
import sys
import time
from dataclasses import asdict
import tempfile
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tools.board_verify import Replay, request, stop, wait_ready
from scan_web.protocol import parse_scan
from scan_web.detection import DetectionConfig
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict
from mid360_suspended_object_detection.msg import LargestObstacle


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    scan = {"rame_id": "/scan_link", "name": "synthetic", "scan": [0] * 6000,
            "angle_start": -math.pi, "increment": 2 * math.pi / 6000,
            "range_min": 0.03, "range_max": 50, "timeoutSec": 0,
            "laserOffset": [0.94, 0, -0.017]}

    def records(angle, invalid=False):
        scan["scan"] = [0] * 6000
        center = round((angle - scan["angle_start"]) / scan["increment"])
        for index in range(-45, 46):
            scan["scan"][(center + index) % 6000] = 2000 if index % 2 else 2100
        scan["timeoutSec"] = 60 if invalid else 0
        return [{"raw": json.dumps({"channel": "ros", "op": "publish", "topic": "/scan", "msg": [scan]}),
                 "delay": 1 / 65}]

    replay = Replay(records(0))
    temporary = tempfile.TemporaryDirectory(prefix='mid360-publication-')
    config_path = Path(temporary.name) / 'scan.yaml'
    config_path.write_text(yaml.safe_dump({'mid360_suspended_object_detection': {'ros__parameters': asdict(DetectionConfig())}}))
    log = (args.output / "ros-replay-backend.log").open("w")
    parameter_topic = "/mid360_test/largest_obstacle"
    topic = "/mid360_test/remapped_obstacle"
    process = subprocess.Popen([str(args.binary.resolve()), "--ws-url", f"ws://127.0.0.1:{replay.port}",
                                "--port", "18767", "--config", str(config_path), "--ros-args", "-p", f"obstacle_topic:={parameter_topic}",
                                "-r", f"{parameter_topic}:={topic}"],
                               cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
    rclpy.init()
    node = Node("mid360_replay_verifier")
    messages = []
    node.create_subscription(LargestObstacle, topic, messages.append,
                             QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                                        durability=DurabilityPolicy.VOLATILE))
    phases = []

    def wait_message(predicate, seconds=7):
        messages.clear()
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if messages and predicate(messages[-1]):
                return messages[-1]
        raise AssertionError(f"ROS condition not met; statuses={[m.status for m in messages]}")

    def cleared(message):
        assert not message.detected and message.candidate_count == message.point_count == 0
        assert all(getattr(message, key) == 0 for key in
                   ("distance_x", "distance_y", "center_distance", "distance", "width", "depth", "area"))
        assert all(getattr(point, axis) == 0 for point in (message.center, message.nearest_point)
                   for axis in ("x", "y", "z"))

    try:
        wait_ready(18767, process)
        request(18767, "/api/config", {"min_x": -5, "max_x": 5, "min_y": -5, "max_y": 5})
        for name, angle in (("front", 0), ("left", math.pi / 2), ("rear", math.pi),
                            ("right", -math.pi / 2), ("front_right", -0.197)):
            replay.records = records(angle)
            target = (2 * math.cos(angle), 2 * math.sin(angle))
            deadline = time.monotonic() + 7
            while time.monotonic() < deadline:
                state = request(18767)
                if state["status"] == "detected" and any(
                        math.hypot(b["center_x"] - target[0], b["center_y"] - target[1]) < 0.15
                        for b in state["objects"]):
                    break
                time.sleep(0.02)
            else:
                raise AssertionError(f"Web did not analyze fixed replay: {name}, {state['status']}")
            message = wait_message(lambda m: m.detected and
                                   math.hypot(m.center.x - target[0], m.center.y - target[1]) < 0.15)
            state = request(18767)
            assert state["status"] == "detected", state["status"]
            boxes = sorted(state["objects"], key=lambda b: (-b["width"] * b["depth"], b["distance"], b["center_x"], b["center_y"]))
            box = boxes[0]
            pairs = ((message.distance_x, box["center_x"]), (message.distance_y, box["center_y"]),
                     (message.width, box["width"]), (message.depth, box["depth"]),
                     (message.distance, box["distance"]), (message.center.x, message.distance_x),
                     (message.center.y, message.distance_y),
                     (message.center_distance, math.hypot(box["center_x"], box["center_y"])))
            assert all(abs(a - b) < 1e-8 for a, b in pairs), pairs
            assert message.candidate_count == len(boxes) and message.point_count == box["count"]
            points = parse_scan(json.loads(replay.records[0]["raw"])).points()
            nearest = min(points, key=lambda p: (math.hypot(*p), p))
            assert abs(message.distance - math.hypot(*nearest)) < 1e-8
            assert any(math.hypot(message.nearest_point.x - p[0], message.nearest_point.y - p[1]) < 1e-8
                       for p in points), "Nearest point must be a real scan sample"
            phases.append({"phase": name, "distance_x": message.distance_x, "distance_y": message.distance_y,
                           "center_distance": message.center_distance, "web_ros_equal": True})
        # A fixed scan continues publishing even while duplicate scans skip analysis.
        messages.clear()
        cached = {}
        rates = []
        started = time.monotonic()
        deadline = started + 12
        # Start comparisons after a fresh publication, so clearing the subscriber
        # list cannot discard the very message already cached before this phase.
        next_query = started + .3
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.01)
            if time.monotonic() >= next_query:
                publication = request(18767)['ros_publication']
                assert publication['topic'] == topic and publication['status'] == 'publishing'
                assert publication['message_type'] == 'mid360_suspended_object_detection/msg/LargestObstacle'
                payload = publication['message']
                stamp = payload['header']['stamp']
                cached[(stamp['sec'], stamp['nanosec'])] = payload
                if time.monotonic() - started > 5:
                    rates.append(publication['frequency_hz'])
                next_query = time.monotonic() + .2
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=.01)
        received = {(m.header.stamp.sec, m.header.stamp.nanosec): message_to_ordereddict(m) for m in messages}
        assert len(cached) >= 40
        for stamp, payload in cached.items():
            assert stamp in received, f'Cached message not received: {stamp}'
            assert received[stamp] == payload, (received[stamp], payload)
        stamps = [m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for m in messages]
        frequency = (len(stamps) - 1) / (stamps[-1] - stamps[0])
        assert 9 <= frequency <= 11 and len(messages) >= 100
        assert all(rate is not None and abs(rate - frequency) < .5 for rate in rates)
        request(18767, "/api/config", {"min_x": -5, "max_x": -4})
        message = wait_message(lambda m: m.status == "clear")
        assert message.data_valid
        cleared(message)
        phases.append({"phase": "empty_roi", "valid_clear": True})
        replay.records = records(0, invalid=True)
        message = wait_message(lambda m: m.status == "invalid")
        assert not message.data_valid and message.error
        cleared(message)
        phases.append({"phase": "invalid_metadata", "fields_cleared": True})
        port = replay.port
        replay.close()
        message = wait_message(lambda m: m.status in ("connecting", "disconnected", "timeout"))
        assert not message.data_valid
        cleared(message)
        phases.append({"phase": "disconnected", "fields_cleared": True})
        replay = Replay(records(0), port)
        request(18767, "/api/config", {"min_x": -5, "max_x": 5})
        message = wait_message(lambda m: m.detected)
        assert message.data_valid
        phases.append({"phase": "recovered", "detected": True})
        report = {"passed": True, "synthetic_only": True, "frequency_hz": frequency,
                  "topic_parameter": parameter_topic, "resolved_topic": topic, "phases": phases,
                  "frequency_measurement_s": 12, "cache_messages_matched_all_fields": len(cached),
                  "cache_frequency_min_hz": min(rates), "cache_frequency_max_hz": max(rates)}
        (args.output / "ros-replay.json").write_text(json.dumps(report, indent=2))
        print(json.dumps(report), flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        stop(process)
        replay.close()
        log.close()
        temporary.cleanup()


if __name__ == "__main__":
    main()
