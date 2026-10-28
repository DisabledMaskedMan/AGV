"""Exercise real C++ node transport, decisions and data loss without a LiDAR."""

import math
import os
import random
import signal
import struct
import subprocess
import time

import pytest
import rclpy
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from mid360_obstacle_avoidance.msg import ObstacleStatus
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool
from visualization_msgs.msg import InteractiveMarkerFeedback, Marker
from visualization_msgs.srv import GetInteractiveMarkers


@pytest.fixture(scope='module')
def detector(tmp_path_factory, request):
    # Isolate synthetic obstacle clouds from the robot's live ROS domain.
    os.environ['ROS_DOMAIN_ID'] = str(100 + os.getpid() % 100)
    rclpy.init()
    node = rclpy.create_node('mid360_obstacle_test')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    statuses, stops, enabled_states = [], [], []
    node.create_subscription(ObstacleStatus, '/mid360_obstacle_avoidance/status',
                             statuses.append, qos)
    node.create_subscription(Bool, '/mid360_obstacle_avoidance/stop_required',
                             stops.append, qos)
    node.create_subscription(Bool, '/mid360_obstacle_avoidance/detection_enabled',
                             enabled_states.append, qos)
    control = node.create_publisher(Bool, '/mid360_obstacle_avoidance/enable_detection', 1)
    publisher = node.create_publisher(CustomMsg, '/test/mid360', qos_profile_sensor_data)
    directory = tmp_path_factory.mktemp('detector')
    logfile = (directory / 'node.log').open('w+')
    # Explicitly configure the test ROI instead of relying on source or YAML defaults.
    parameters = {
        'input_topic': '/test/mid360', 'expected_frame': 'livox_frame',
        'roi_min_x': 0.2, 'roi_max_x': 3.0,
        'roi_min_y': -0.6, 'roi_max_y': 0.6,
        'roi_min_z': -0.3, 'roi_max_z': 1.0,
        'cluster_distance_threshold': 0.10, 'min_obstacle_points': 3,
        'min_cluster_diagonal': 0.05, 'min_cluster_volume': 0.0,
        'cloud_timeout': 0.4,
        # Existing geometry tests explicitly opt in; the production default is paused.
        'detection_enabled_on_start': True,
        'enable_roi_editor': False, 'enable_web_editor': False,
    }
    parameters.update(getattr(request, 'param', {}))
    if parameters.get('detection_enabled_on_start') is None:
        parameters.pop('detection_enabled_on_start', None)
    config_file = directory / 'obstacle.yaml'
    # An actual, writable block-style YAML; CLI overrides stay explicit for all tests.
    config_file.write_text(
        '# Preserve calibration comments\nmid360_obstacle_avoidance:\n  ros__parameters:\n' +
        ''.join(f'    {key}: {str(value).lower() if isinstance(value, bool) else value}\n'
                for key, value in parameters.items() if key != 'roi_config_file') +
        '    # Preserve all unrelated settings\n', encoding='utf-8')
    if parameters['enable_roi_editor'] or parameters['enable_web_editor']:
        parameters.setdefault('roi_config_file', str(config_file.resolve()))
    command = [os.environ['MID360_NODE_EXECUTABLE'], '--ros-args']
    for name, value in parameters.items():
        command.extend(['-p', f'{name}:={value}'])
    process = subprocess.Popen(command, stdout=logfile, stderr=subprocess.STDOUT)

    class Harness:
        sequence = 0
        ros_node = node
        yaml_file = config_file
        control_publisher = control
        detection_states = enabled_states

        def wait(self, predicate, timeout=5.0):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.02)
                if predicate():
                    return
                if process.poll() is not None:
                    logfile.seek(0)
                    pytest.fail('Detector exited: ' + logfile.read())
            pytest.fail('Timed out waiting for detector result')

        def send(self, points, *, frame='livox_frame', point_num=None):
            self.sequence += 1
            message = CustomMsg()
            message.header.frame_id = frame
            message.header.stamp.sec = self.sequence
            message.header.stamp.nanosec = 123456789
            message.points = [CustomPoint(x=float(x), y=float(y), z=float(z))
                              for x, y, z in points]
            message.point_num = len(points) if point_num is None else point_num
            publisher.publish(message)
            self.wait(lambda: statuses and statuses[-1].header.stamp.sec == self.sequence)
            return statuses[-1]

        def set_enabled(self, enabled):
            self.wait(lambda: control.get_subscription_count() > 0)
            before_states, before_statuses = len(enabled_states), len(statuses)
            control.publish(Bool(data=enabled))
            self.wait(lambda: len(enabled_states) > before_states and
                      enabled_states[-1].data == enabled and len(statuses) > before_statuses)
            return statuses[-1]

    harness = Harness()
    try:
        harness.wait(lambda: publisher.get_subscription_count() > 0 and statuses and stops and
                     enabled_states and control.get_subscription_count() > 0)
        assert not statuses[-1].data_valid
        startup_enabled = parameters.get('detection_enabled_on_start', False)
        assert enabled_states[-1].data == startup_enabled
        if not startup_enabled:
            assert not statuses[-1].stop_required and not stops[-1].data
        yield harness, statuses, stops
    finally:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)
        logfile.close()
        node.destroy_node()
        rclpy.shutdown()


def line(x, y=0.0, z=0.0):
    """Three nearby points spanning more than the default diagonal threshold."""
    return [(x, y, z + offset) for offset in (0.0, 0.03125, 0.0625)]


@pytest.mark.parametrize('detector', [{'detection_enabled_on_start': None}], indirect=True)
def test_detection_control_default_pause_transitions_and_health(detector):
    h, statuses, stops = detector

    def paused(result):
        assert not result.stop_required and not result.obstacle_detected
        assert result.obstacle_points == result.stop_zone_points == 0
        assert math.isnan(result.nearest_distance)

    assert not h.detection_states[-1].data
    result = h.send(line(0.8))
    assert result.data_valid
    paused(result)
    # Repeating false must not invalidate a healthy input frame.
    result = h.set_enabled(False)
    assert result.data_valid
    paused(result)
    h.wait(lambda: not statuses[-1].data_valid, timeout=2)
    paused(statuses[-1])
    assert not stops[-1].data
    for points, kwargs in [([], {}), ([(0, 0, 0)], {}), ([(math.nan, 1, 1)], {}),
                           (line(0.8), {'frame': 'wrong'}),
                           (line(0.8), {'point_num': 4})]:
        result = h.send(points, **kwargs)
        assert not result.data_valid
        paused(result)

    before = h.send([(4, 0, 0)])
    assert before.data_valid
    enabled = h.set_enabled(True)
    assert enabled.header == before.header and enabled.data_valid
    assert not enabled.stop_required and not enabled.obstacle_detected
    assert math.isnan(enabled.nearest_distance)
    # A heartbeat cannot reinterpret the old clear frame as an evaluated one.
    count = len(statuses)
    h.wait(lambda: len(statuses) >= count + 2, timeout=0.35)
    assert not statuses[-1].stop_required
    h.wait(lambda: statuses[-1].stop_required, timeout=2)
    clear = h.send([(4, 0, 0)])
    assert clear.data_valid and not clear.stop_required
    assert math.isinf(clear.nearest_distance)
    obstacle = h.send(line(0.8))
    assert obstacle.stop_required and obstacle.obstacle_detected
    repeated = h.set_enabled(True)
    assert repeated.data_valid and repeated.obstacle_detected and repeated.obstacle_points == 3
    assert repeated.header == obstacle.header
    disabled = h.set_enabled(False)
    assert disabled.data_valid and disabled.header == obstacle.header
    paused(disabled)
    h.set_enabled(True)
    # No new cloud: enabled state must eventually fail-stop on timeout.
    h.wait(lambda: not statuses[-1].data_valid, timeout=2)
    assert statuses[-1].stop_required
    assert h.send(line(0.8)).obstacle_detected
    h.set_enabled(False)


def test_detection_control_qos_and_late_state_subscriber(detector):
    h, _, _ = detector
    from rclpy.qos import ReliabilityPolicy, HistoryPolicy
    inputs = h.ros_node.get_subscriptions_info_by_topic(
        '/mid360_obstacle_avoidance/enable_detection')
    endpoint = next(p for p in inputs if p.node_name == 'mid360_obstacle_avoidance')
    # Fast DDS endpoint discovery advertises UNKNOWN history and depth=0.
    # Check depth when the middleware actually exposes it; the retained-command
    # integration test below separately verifies the required volatile behavior.
    assert endpoint.qos_profile.history in (HistoryPolicy.UNKNOWN, HistoryPolicy.KEEP_LAST)
    if endpoint.qos_profile.history == HistoryPolicy.KEEP_LAST:
        assert endpoint.qos_profile.depth == 1
    assert endpoint.qos_profile.reliability == ReliabilityPolicy.RELIABLE
    assert endpoint.qos_profile.durability == DurabilityPolicy.VOLATILE
    outputs = h.ros_node.get_publishers_info_by_topic(
        '/mid360_obstacle_avoidance/detection_enabled')
    output = next(p for p in outputs if p.node_name == 'mid360_obstacle_avoidance')
    assert output.qos_profile.reliability == ReliabilityPolicy.RELIABLE
    assert output.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL
    h.set_enabled(False)
    late = []
    sub = h.ros_node.create_subscription(Bool, '/mid360_obstacle_avoidance/detection_enabled',
        late.append, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    try:
        h.wait(lambda: late)
        assert late[-1].data is False
        h.set_enabled(True)
        h.wait(lambda: late[-1].data)
    finally:
        h.ros_node.destroy_subscription(sub)


def test_historical_enable_command_is_not_replayed(detector, tmp_path):
    h, _, _ = detector
    prefix = '/control_replay_probe'
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    command = h.ros_node.create_publisher(Bool, prefix + '/enable_detection', qos)
    command.publish(Bool(data=True))
    states = []
    state_sub = h.ros_node.create_subscription(Bool, prefix + '/detection_enabled',
                                               states.append, qos)
    with (tmp_path / 'replay.log').open('w+') as logfile:
        process = subprocess.Popen([os.environ['MID360_NODE_EXECUTABLE'], '--ros-args',
            '-r', '__node:=control_replay_probe', '-p', 'input_topic:=/test/replay'],
            stdout=logfile, stderr=subprocess.STDOUT)
        try:
            h.wait(lambda: states and command.get_subscription_count() > 0)
            h.wait(lambda: len(states) >= 4)
            assert all(not s.data for s in states)
            # Only a newly published true command should enable the new process.
            command.publish(Bool(data=True))
            h.wait(lambda: states[-1].data)
        finally:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill(); process.wait(timeout=5)
            h.ros_node.destroy_subscription(state_sub)
            h.ros_node.destroy_publisher(command)


@pytest.mark.parametrize('points,count,distance', [
    (line(0.8), 3, 0.8),
    (line(2.0), 3, 2.0),  # Entire ROI is now a stop zone.
    ([(0.5, 0.0, 0.0), (1.0, 0.0, 0.0), (2.0, 0.0, 0.0)], 0, math.inf),
    (line(0.8)[::2], 0, math.inf),  # Diagonal passes, point count fails.
    ([(0.8, 0.0, z) for z in (0.0, 0.01, 0.02)], 0, math.inf),
    ([(0.8, 0.0, 0.0)] * 20, 0, math.inf),
    (line(0.8)[:2] + line(2.0)[:2], 0, math.inf),
    (line(0.8) + line(2.0) + [(0.3, 0.0, 0.0)], 6, 0.8),
    # All six ROI faces are inclusive, including decimal float32 coordinates.
    (line(0.2), 3, 0.2),
    (line(3.0), 3, 3.0),
    (line(0.8, -0.6), 3, 0.8),
    (line(0.8, 0.6), 3, 0.8),
    ([(0.8, y, -0.3) for y in (0.0, 0.03125, 0.0625)], 3, 0.8),
    ([(0.8, y, 1.0) for y in (0.0, 0.03125, 0.0625)], 3, 0.8),
    (line(0.19) + line(3.01) + line(0.8, -0.61) + line(0.8, 0.61) +
     [(0.8, y, z) for y in (0.0, 0.03125, 0.0625) for z in (-0.31, 1.01)],
     0, math.inf),
    # Outside points must not connect two undersized ROI components.
    (line(0.5, 0.59)[:2] + line(0.75, 0.59)[:2] +
     [(x, 0.61, 0.0) for x in (0.55, 0.625, 0.70)], 0, math.inf),
    ([(math.nan, 0.0, 0.0), (math.inf, 0.0, 0.0), (0.0, 0.0, 0.0)] +
     line(2.0), 3, 2.0),
    ([(4.0, 0.0, 0.0)], 0, math.inf),
])
def test_roi_decisions(detector, points, count, distance):
    harness, _, _ = detector
    result = harness.send(points)
    assert result.data_valid
    assert result.obstacle_detected == (count > 0)
    assert result.stop_required == (count > 0)
    assert result.obstacle_points == count
    assert result.stop_zone_points == count
    assert result.nearest_distance == pytest.approx(distance)


def test_chain_and_input_order(detector):
    harness, _, _ = detector
    points = [(0.8, y, 0.0) for y in (-0.125, -0.0625, 0.0, 0.0625, 0.125)]
    randomizer = random.Random(360)
    for _ in range(5):
        randomizer.shuffle(points)
        result = harness.send(points)
        assert result.data_valid and result.stop_required
        assert result.obstacle_points == result.stop_zone_points == 5
        assert result.nearest_distance == pytest.approx(0.8)


@pytest.mark.parametrize('detector', [
    {'cluster_distance_threshold': 0.125, 'min_cluster_diagonal': 0.25},
], indirect=True)
def test_exact_cluster_thresholds(detector):
    harness, _, _ = detector
    # Binary-exact coordinates test inclusive comparisons without decimal rounding.
    result = harness.send([(0.5, y, 0.0) for y in (-0.125, 0.0, 0.125)])
    assert result.data_valid and result.stop_required
    assert result.obstacle_points == result.stop_zone_points == 3
    assert not harness.send([(0.5, y, 0.0) for y in (-0.125, 0.0, 0.126)]).stop_required
    assert not harness.send([(0.5, y, 0.0) for y in (-0.124, 0.0, 0.125)]).stop_required


@pytest.mark.parametrize('detector', [
    {'cluster_distance_threshold': 0.125, 'min_cluster_volume': 0.001953125},
], indirect=True)
def test_optional_volume(detector):
    harness, _, _ = detector
    assert not harness.send(line(0.5)).stop_required  # A plane has zero box volume.
    result = harness.send([(0.5, 0.0, 0.0), (0.625, 0.0, 0.0),
                           (0.625, 0.125, 0.0), (0.625, 0.125, 0.125)])
    assert result.data_valid and result.stop_required
    assert result.obstacle_points == result.stop_zone_points == 4
    assert result.nearest_distance == pytest.approx(0.5)


@pytest.mark.parametrize('points,kwargs', [
    ([], {}),
    ([(math.nan, 0.0, 0.0), (0.0, 0.0, 0.0)], {}),
    ([(2.0, 0.0, 0.0)], {'point_num': 10}),
    ([(2.0, 0.0, 0.0)], {'frame': 'incorrect_frame'}),
])
def test_invalid_cloud_waits_for_timeout(detector, points, kwargs):
    harness, _, _ = detector
    assert not harness.send([(4.0, 0.0, 0.0)]).stop_required
    result = harness.send(points, **kwargs)
    assert not result.data_valid
    assert not result.stop_required
    assert math.isnan(result.nearest_distance)
    assert result.obstacle_points == 0
    assert result.stop_zone_points == 0
    assert not result.obstacle_detected
    recovered = harness.send(line(2.0))
    assert recovered.data_valid and recovered.stop_required
    assert recovered.obstacle_points == 3


def test_timeout_and_recovery(detector):
    harness, statuses, stops = detector
    harness.send(line(2.0))
    assert not harness.send([(4.0, 0.0, 0.0)]).stop_required
    harness.wait(lambda: not statuses[-1].data_valid and stops[-1].data, timeout=2.0)
    assert statuses[-1].stop_required
    assert math.isnan(statuses[-1].nearest_distance)
    assert statuses[-1].obstacle_points == statuses[-1].stop_zone_points == 0
    assert harness.send([(4.0, 0.0, 0.0)]).data_valid
    harness.wait(lambda: not stops[-1].data)
    assert harness.send(line(2.0)).stop_required


def test_heartbeat_keeps_cloud_header(detector):
    harness, statuses, _ = detector
    result = harness.send(line(2.0))
    # Require two additional publications; one immediately queued copy is insufficient.
    before = len(statuses)
    harness.wait(lambda: len(statuses) >= before + 2, timeout=0.35)
    assert statuses[-1].data_valid and statuses[-1].stop_required
    assert statuses[-1].header == result.header
    assert statuses[-1].obstacle_points == 3


@pytest.mark.parametrize('parameter', [
    'roi_min_x:=4.0', 'min_obstacle_points:=0', 'cloud_timeout:=0.0',
    'roi_min_x:=-0.001', 'roi_max_x:=0.205',
    'roi_min_y:=1.0', 'min_obstacle_points:=4294967296',
    'cluster_distance_threshold:=0.0', 'cluster_distance_threshold:=-0.1',
    'cluster_distance_threshold:=.nan', 'cluster_distance_threshold:=.inf',
    'cluster_distance_threshold:=1.0e-300',
    'min_cluster_diagonal:=-0.01', 'min_cluster_diagonal:=.nan',
    'min_cluster_diagonal:=.inf', 'min_cluster_volume:=-0.001',
    'min_cluster_volume:=.nan', 'min_cluster_volume:=.inf',
])
def test_invalid_configuration(parameter):
    result = subprocess.run([
        os.environ['MID360_NODE_EXECUTABLE'], '--ros-args', '-p', parameter,
    ], capture_output=True, text=True, timeout=10)
    assert result.returncode != 0
    assert 'Invalid configuration' in result.stderr


def test_headless_mode_has_no_visual_resources(detector):
    harness, _, _ = detector
    publishers = dict(harness.ros_node.get_publisher_names_and_types_by_node(
        'mid360_obstacle_avoidance', '/'))
    services = dict(harness.ros_node.get_service_names_and_types_by_node(
        'mid360_obstacle_avoidance', '/'))
    assert '/mid360_obstacle_avoidance/pointcloud' not in publishers
    assert '/mid360_obstacle_avoidance/roi_marker' not in publishers
    assert '/mid360_obstacle_avoidance/roi_state' not in publishers
    assert '/mid360_obstacle_avoidance/detector_config' not in publishers
    assert not any(name.endswith(('/set_roi_boundary', '/save_roi', '/reset_roi',
                                  '/apply_detector_config')) for name in services)
    assert not any('/roi_editor/' in name for name in (*publishers, *services))


@pytest.mark.parametrize('detector', [{'enable_roi_editor': True}], indirect=True)
def test_roi_editor_cloud_drag_save_and_reset(detector):
    harness, _, _ = detector
    node = harness.ros_node
    prefix = '/mid360_obstacle_avoidance'
    visuals, boxes = [], []
    subscriptions = [
        node.create_subscription(PointCloud2, prefix + '/pointcloud',
                                 visuals.append, qos_profile_sensor_data),
        node.create_subscription(Marker, prefix + '/roi_marker', boxes.append,
                                 QoSProfile(depth=1,
                                            durability=DurabilityPolicy.TRANSIENT_LOCAL)),
    ]
    feedback_pub = node.create_publisher(
        InteractiveMarkerFeedback, prefix + '/roi_editor/feedback', 10)
    client = node.create_client(
        GetInteractiveMarkers, prefix + '/roi_editor/get_interactive_markers')
    harness.wait(lambda: boxes and client.service_is_ready() and
                 feedback_pub.get_subscription_count() > 0 and
                 node.count_publishers(prefix + '/pointcloud') == 1)
    harness.wait(lambda: node.get_publishers_info_by_topic(prefix + '/pointcloud'))

    def get_markers():
        future = client.call_async(GetInteractiveMarkers.Request())
        harness.wait(future.done)
        return {marker.name: marker for marker in future.result().markers}

    def feedback(name, *, coordinate=None, frame='livox_frame', menu=None):
        message = InteractiveMarkerFeedback()
        message.client_id = 'mid360_roi_test'
        message.marker_name = name
        message.control_name = 'move' if menu is None else 'menu'
        message.header.frame_id = frame
        message.pose = get_markers()[name].pose
        message.event_type = (InteractiveMarkerFeedback.POSE_UPDATE if menu is None
                              else InteractiveMarkerFeedback.MENU_SELECT)
        if coordinate is not None:
            setattr(message.pose.position, name[-1], coordinate)
        if menu is not None:
            message.menu_entry_id = menu
        before = len(boxes)
        feedback_pub.publish(message)
        if menu is None:
            harness.wait(lambda: len(boxes) > before)

    def bounds():
        box = boxes[-1]
        assert box.header.frame_id == 'livox_frame'
        return {
            f'roi_{side}_{axis}': getattr(box.pose.position, axis) +
            (-0.5 if side == 'min' else 0.5) * getattr(box.scale, axis)
            for axis in 'xyz' for side in ('min', 'max')
        }

    startup = bounds()
    original_yaml = harness.yaml_file.read_bytes()
    markers = get_markers()
    assert set(markers) == set(startup) | {'roi_editor_menu'}
    assert all(marker.header.frame_id == 'livox_frame' for marker in markers.values())
    menu_entries = {entry.title: entry.id
                    for entry in markers['roi_editor_menu'].menu_entries}
    assert set(menu_entries) == {'Save ROI to YAML', 'Reload ROI from startup values'}

    # Conversion includes ROI-external points, drops NaN/Inf/zero and preserves Header.
    points = line(2.0) + [(4.0, 0.0, 0.0), (0.0, 0.0, 0.0),
                          (math.nan, 0.0, 0.0), (math.inf, 0.0, 0.0)]
    status = harness.send(points)
    harness.wait(lambda: visuals and visuals[-1].header == status.header)
    cloud = visuals[-1]
    assert cloud.height == 1 and cloud.width == 4 and cloud.is_dense
    fields = {field.name: field for field in cloud.fields}
    assert all(fields[name].datatype == PointField.FLOAT32 and fields[name].count == 1
               for name in 'xyz')
    byte_order = '>' if cloud.is_bigendian else '<'
    converted = [tuple(struct.unpack_from(byte_order + 'f', cloud.data,
                       i * cloud.point_step + fields[axis].offset)[0] for axis in 'xyz')
                 for i in range(cloud.width)]
    assert converted[-1] == (4.0, 0.0, 0.0)
    assert status.data_valid and status.stop_required and status.obstacle_points == 3

    invalid = harness.send([(0.0, 0.0, 0.0), (math.nan, 0.0, 0.0)])
    harness.wait(lambda: visuals[-1].header == invalid.header)
    assert not invalid.data_valid and not invalid.stop_required
    assert visuals[-1].width == 0 and len(visuals[-1].data) == 0
    assert harness.send(line(2.0)).stop_required

    # Each face changes exactly its own bound; dragging is not persisted implicitly.
    changed = dict(startup)
    for name, value in [('roi_min_x', 0.3), ('roi_max_x', 1.5),
                        ('roi_min_y', -0.5), ('roi_max_y', 0.5),
                        ('roi_min_z', -0.2), ('roi_max_z', 0.8)]:
        feedback(name, coordinate=value)
        changed[name] = value
        assert bounds() == pytest.approx(changed)
        assert all(marker.header.frame_id == 'livox_frame'
                   for marker in get_markers().values())
    assert harness.yaml_file.read_bytes() == original_yaml
    assert not harness.send(line(2.0)).stop_required
    assert harness.send(line(0.8)).stop_required

    feedback('roi_editor_menu', menu=menu_entries['Save ROI to YAML'])
    harness.wait(lambda: harness.yaml_file.read_bytes() != original_yaml)
    saved_yaml = harness.yaml_file.read_bytes()
    original_lines = original_yaml.decode().splitlines(keepends=True)
    saved_lines = saved_yaml.decode().splitlines(keepends=True)
    assert len(saved_lines) == len(original_lines)
    for before, after in zip(original_lines, saved_lines):
        key = before.strip().partition(':')[0]
        if key in changed:
            assert float(after.partition(':')[2]) == pytest.approx(changed[key])
        else:
            assert before == after

    feedback('roi_editor_menu', menu=menu_entries['Reload ROI from startup values'])
    harness.wait(lambda: bounds() == pytest.approx(startup))
    assert harness.yaml_file.read_bytes() == saved_yaml
    assert harness.send(line(2.0)).stop_required

    # Reject frame changes and invalid coordinates; marker state also stays finite.
    feedback('roi_max_x', coordinate=1.0, frame='wrong_frame')
    assert bounds() == pytest.approx(startup)
    feedback('roi_max_x', coordinate=math.nan)
    assert bounds() == pytest.approx(startup)
    for axis in 'xyz':
        feedback(f'roi_min_{axis}', coordinate=100.0)
        assert bounds()[f'roi_max_{axis}'] - bounds()[f'roi_min_{axis}'] == pytest.approx(0.01)
        feedback(f'roi_max_{axis}', coordinate=-100.0)
        assert bounds()[f'roi_max_{axis}'] - bounds()[f'roi_min_{axis}'] == pytest.approx(0.01)
    feedback('roi_min_x', coordinate=-1.0)
    assert bounds()['roi_min_x'] == pytest.approx(0.0)
    # Keep subscriptions alive throughout all DDS traffic.
    assert len(subscriptions) == 2


@pytest.mark.parametrize('editor', ['enable_roi_editor', 'enable_web_editor'])
@pytest.mark.parametrize('extra', [[], ['-p', 'roi_config_file:=relative.yaml']])
def test_editor_requires_absolute_config_path(extra, editor):
    result = subprocess.run([os.environ['MID360_NODE_EXECUTABLE'], '--ros-args',
                             '-p', editor+':=true', *extra],
                            capture_output=True, text=True, timeout=10)
    assert result.returncode != 0
    assert 'absolute roi_config_file' in result.stderr


class ConfigClient:
    def __init__(self, harness):
        from mid360_obstacle_avoidance.msg import DetectorConfig
        from mid360_obstacle_avoidance.srv import ApplyDetectorConfig, SetRoiBoundary
        self.h = harness
        self.states = []
        self.kind = ApplyDetectorConfig
        self.boundary_kind = SetRoiBoundary
        prefix = '/mid360_obstacle_avoidance/'
        harness.ros_node.create_subscription(DetectorConfig, prefix+'detector_config',
            self.states.append, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.client = harness.ros_node.create_client(ApplyDetectorConfig, prefix+'apply_detector_config')
        self.boundary = harness.ros_node.create_client(SetRoiBoundary, prefix+'set_roi_boundary')
        harness.wait(lambda: self.states and self.client.service_is_ready() and self.boundary.service_is_ready())

    def call(self, client, request):
        future = client.call_async(request)
        self.h.wait(future.done)
        return future.result()

    def apply(self, **changes):
        keys = ('cluster_distance_threshold', 'min_obstacle_points', 'min_cluster_diagonal',
                'min_cluster_volume', 'cloud_timeout')
        request = self.kind.Request(**{k: getattr(self.states[-1], k) for k in keys})
        for key, value in changes.items():
            setattr(request, key, value)
        response = self.call(self.client, request)
        if response.success:
            self.h.wait(lambda: self.states[-1].revision == response.config.revision)
        return response

    def move(self, key, value):
        return self.call(self.boundary, self.boundary_kind.Request(boundary=key, value=value))


@pytest.mark.parametrize('detector', [{'enable_web_editor': True, 'cloud_timeout': 1.0}], indirect=True)
def test_config_transaction_and_algorithm_effects(detector):
    h, statuses, _ = detector
    c = ConfigClient(h)
    assert c.apply(min_obstacle_points=4).success
    assert not h.send(line(.8)).obstacle_detected
    assert c.apply(min_obstacle_points=3, cluster_distance_threshold=.02).success
    assert not h.send(line(.8)).obstacle_detected
    assert c.apply(cluster_distance_threshold=.1, min_cluster_diagonal=.2).success
    assert not h.send(line(.8)).obstacle_detected
    assert c.apply(min_cluster_diagonal=.05, min_cluster_volume=.000001).success
    assert not h.send(line(.8)).obstacle_detected
    solid = [(.8, 0., 0.), (.85, 0., 0.), (.85, .05, 0.), (.85, .05, .05)]
    assert h.send(solid).obstacle_detected
    revision = c.states[-1].revision
    assert c.apply().success  # Unchanged settings do not erase a valid decision.
    assert c.states[-1].revision == revision
    h.wait(lambda: statuses[-1].obstacle_detected)
    assert c.apply(min_cluster_volume=0.).success
    assert h.send(line(.8)).obstacle_detected
    original = h.yaml_file.read_bytes()
    assert c.move('roi_max_x', 2.5).success
    assert h.yaml_file.read_bytes() == original
    response = c.apply(min_obstacle_points=5, cloud_timeout=1.2)
    assert response.success and response.config.roi_max_x == 2.5
    saved = h.yaml_file.read_bytes()
    allowed = {'roi_min_x', 'roi_max_x', 'roi_min_y', 'roi_max_y', 'roi_min_z', 'roi_max_z',
               'cluster_distance_threshold', 'min_obstacle_points', 'min_cluster_diagonal',
               'min_cluster_volume', 'cloud_timeout'}
    for before, after in zip(original.splitlines(keepends=True), saved.splitlines(keepends=True)):
        if before.decode().strip().partition(':')[0] not in allowed:
            assert before == after
    # ROS parameters remain startup-only; authoritative values are in DetectorConfig.
    from rcl_interfaces.srv import GetParameters
    parameters = h.ros_node.create_client(GetParameters, '/mid360_obstacle_avoidance/get_parameters')
    h.wait(parameters.service_is_ready)
    result = c.call(parameters, GetParameters.Request(names=['min_obstacle_points']))
    assert result.values[0].integer_value == 3
    baseline = c.states[-1]
    for change in [{'min_obstacle_points': 0}, {'cluster_distance_threshold': 0.},
                   {'cluster_distance_threshold': 1.e-300}, {'cloud_timeout': math.nan},
                   {'cloud_timeout': math.inf}, {'min_cluster_diagonal': -.1},
                   {'min_cluster_volume': math.inf}]:
        rejected = c.apply(**change)
        assert not rejected.success and rejected.message
        assert rejected.config.revision == baseline.revision
        assert h.yaml_file.read_bytes() == saved
    assert c.move('roi_max_x', 2.4).success
    mode = h.yaml_file.stat().st_mode
    try:
        h.yaml_file.chmod(0o444)
        rejected = c.apply(min_obstacle_points=6)
        assert not rejected.success
        assert rejected.config.min_obstacle_points == 5 and rejected.config.roi_max_x == 2.4
        assert h.yaml_file.read_bytes() == saved
    finally:
        h.yaml_file.chmod(mode)
    # Missing/duplicate keys fail without changing either algorithm or file.
    for text in [saved.decode().replace('    cloud_timeout:', '    missing_timeout:'),
                 saved.decode()+'    cloud_timeout: 9.0\n']:
        h.yaml_file.write_text(text)
        assert not c.apply(min_obstacle_points=6).success
        assert h.yaml_file.read_text() == text
    h.yaml_file.write_bytes(saved)


@pytest.mark.parametrize('detector', [{'enable_web_editor': True, 'cloud_timeout': .8,
                                    'detection_enabled_on_start': False}], indirect=True)
def test_timeout_clock_survives_controls_edits_and_invalid_stream(detector):
    h, statuses, _ = detector
    c = ConfigClient(h)
    assert not h.set_enabled(True).stop_required
    start = time.monotonic()
    h.wait(lambda: time.monotonic()-start >= .4)
    assert not h.set_enabled(True).stop_required
    assert c.move('roi_max_x', 2.8).success
    assert c.apply(min_cluster_diagonal=.06).success
    # Invalid frames neither cause immediate stop nor renew the acquisition window.
    while time.monotonic()-start < .7:
        assert not h.send([]).stop_required
        h.wait(lambda: time.monotonic()-start >= .7 or statuses[-1].stop_required, timeout=1)
    h.wait(lambda: statuses[-1].stop_required, timeout=.45)
    assert time.monotonic()-start < 1.2
    assert not statuses[-1].data_valid
    assert h.set_enabled(True).stop_required
    assert c.move('roi_max_x', 2.7).success
    assert c.apply(min_cluster_diagonal=.05).success
    h.wait(lambda: statuses[-1].stop_required)
    assert not h.send([(4., 0., 0.)]).stop_required
    valid_at = time.monotonic()
    h.wait(lambda: time.monotonic()-valid_at >= .35)
    assert c.apply(cloud_timeout=.2).success
    h.wait(lambda: statuses[-1].stop_required, timeout=.3)
    assert c.apply(cloud_timeout=1.0).success
    h.wait(lambda: not statuses[-1].stop_required, timeout=.3)
    h.wait(lambda: statuses[-1].stop_required, timeout=1.0)
    assert time.monotonic()-valid_at < 1.4
    assert h.send(line(.8)).stop_required
    assert not h.send([(4., 0., 0.)]).stop_required
    # A continuous stream of invalid frames eventually expires the last valid receipt.
    start = time.monotonic()
    while not statuses[-1].stop_required:
        h.send([], frame='wrong_frame')
        until = time.monotonic()+.08
        h.wait(lambda: time.monotonic() >= until or statuses[-1].stop_required)
        assert time.monotonic()-start < 1.5
    assert not h.set_enabled(False).stop_required
    h.wait(lambda: time.monotonic()-start > 1.3)
    assert not statuses[-1].stop_required
