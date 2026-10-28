"""Check largest-obstacle ROS messages, XY signs, clearing and publication rate."""
import argparse
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from mid360_suspended_object_detection.msg import LargestObstacle


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/mid360_suspended_object_detection/largest_obstacle")
    parser.add_argument("--seconds", type=float, default=4)
    parser.add_argument("--require-detected", action="store_true")
    parser.add_argument("--expect-status", default="")
    args = parser.parse_args()
    rclpy.init()
    node = Node("largest_obstacle_topic_check")
    messages = []
    qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                     reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
    node.create_subscription(LargestObstacle, args.topic, messages.append, qos)
    deadline = time.monotonic() + args.seconds
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.1, max(0.0, deadline - time.monotonic())))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    stamps = [message.header.stamp.sec + message.header.stamp.nanosec * 1e-9 for message in messages]
    frequency = (len(stamps) - 1) / (stamps[-1] - stamps[0]) if len(stamps) > 1 and stamps[-1] > stamps[0] else 0
    assert frequency >= 8.5, f"Publication below 10 Hz: {len(messages)} in {args.seconds}s ({frequency:.2f} Hz)"
    for message in messages:
        if message.detected:
            assert message.data_valid and message.candidate_count > 0 and message.point_count > 0
            assert abs(message.distance_x - message.center.x) < 1e-10
            assert abs(message.distance_y - message.center.y) < 1e-10
            assert abs(message.center_distance - math.hypot(message.distance_x, message.distance_y)) < 1e-9
            assert abs(message.distance - math.hypot(message.nearest_point.x, message.nearest_point.y)) < 1e-9
            assert abs(message.area - message.width * message.depth) < 1e-10
            assert message.nearest_point.z == 0 and message.center.z == 0
            assert message.header.frame_id.lstrip("/") == "scan_link"
        else:
            assert not message.detected and message.candidate_count == 0 and message.point_count == 0
            assert all(getattr(message, field) == 0 for field in
                       ("distance_x", "distance_y", "center_distance", "distance", "width", "depth", "area"))
            assert all(getattr(point, axis) == 0 for point in (message.center, message.nearest_point)
                       for axis in ("x", "y", "z"))
    detected = [message for message in messages if message.detected]
    assert detected or not args.require_detected, "No detected message received"
    if args.expect_status:
        assert any(message.status == args.expect_status for message in messages), \
            f"Expected {args.expect_status}; saw {[message.status for message in messages]}"
    report = {"topic": args.topic, "messages": len(messages), "duration_s": args.seconds,
              "frequency_hz": frequency,
              "detected_messages": len(detected), "fields_valid": True,
              "clearing_valid": True, "statuses": sorted(set(message.status for message in messages))}
    print(json.dumps(report), flush=True)


if __name__ == "__main__":
    main()
