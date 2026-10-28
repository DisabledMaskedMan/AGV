#!/usr/bin/python3
"""Loopback HTTP/WebSocket bridge. ROS callbacks never wait on browser traffic."""
import asyncio
import contextlib
import json
import math
from pathlib import Path
import threading
import time

import numpy as np
from aiohttp import web, WSMsgType
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker
from rosidl_runtime_py.convert import message_to_ordereddict
from std_srvs.srv import Trigger
from mid360_obstacle_avoidance.msg import ObstacleStatus, RoiState, DetectorConfig
from mid360_obstacle_avoidance.srv import SetRoiBoundary, ApplyDetectorConfig

BOUNDS = ('min_x', 'max_x', 'min_y', 'max_y', 'min_z', 'max_z')
ALGORITHM = ('cluster_distance_threshold', 'min_obstacle_points', 'min_cluster_diagonal',
             'min_cluster_volume', 'cloud_timeout')
TOPICS = {'status': ('mid360_obstacle_avoidance/msg/ObstacleStatus', False),
          'stop_required': ('std_msgs/msg/Bool', False),
          'detection_enabled': ('std_msgs/msg/Bool', False),
          'roi_state': ('mid360_obstacle_avoidance/msg/RoiState', True),
          'detector_config': ('mid360_obstacle_avoidance/msg/DetectorConfig', True),
          'pointcloud': ('sensor_msgs/msg/PointCloud2', False),
          'roi_marker': ('visualization_msgs/msg/Marker', True)}


def json_safe(value):
    if isinstance(value, float) and not math.isfinite(value):
        return 'NaN' if math.isnan(value) else '+inf' if value > 0 else '-inf'
    if isinstance(value, dict):
        return {k: json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(v) for v in value]
    return value


def config_dict(msg):
    return json_safe(dict(message_to_ordereddict(msg)))


def roi_dict(msg):
    return dict({k: getattr(msg, k) for k in BOUNDS}, frame=msg.header.frame_id,
                revision=msg.revision, cloud_timeout=msg.cloud_timeout)


def pack_xyz(msg):
    """Honor offsets, point padding, row padding and endianness; return LE XYZ."""
    if not msg.width or not msg.height:
        return b''
    fields = {f.name: f for f in msg.fields}
    for name in ('x', 'y', 'z'):
        if name not in fields or fields[name].datatype != 7 or fields[name].count != 1:
            raise ValueError('Expected float32 XYZ fields')
        if fields[name].offset + 4 > msg.point_step:
            raise ValueError('Invalid field offset')
    if msg.row_step < msg.width * msg.point_step:
        raise ValueError('Invalid row stride')
    dtype = np.dtype({'names': ['x', 'y', 'z'],
                      'formats': [('>' if msg.is_bigendian else '<') + 'f4'] * 3,
                      'offsets': [fields[n].offset for n in ('x', 'y', 'z')],
                      'itemsize': msg.point_step})
    points = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                        strides=(msg.row_step, msg.point_step))
    xyz = np.stack([points[n].ravel() for n in ('x', 'y', 'z')], axis=1)
    valid = np.isfinite(xyz).all(axis=1) & (xyz != 0).any(axis=1)
    return xyz[valid].astype('<f4', copy=False).tobytes()


class Bridge(Node):
    def __init__(self):
        super().__init__('mid360_web_editor')
        self.host = self.declare_parameter('web_host', '127.0.0.1').value
        self.port = self.declare_parameter('web_port', 8080).value
        if not self.host or not 1 <= self.port <= 65535:
            raise ValueError('Invalid web_host/web_port')
        self.lock = threading.Lock()
        self.state = {'roi': None, 'status': None, 'cloud': None, 'detection': None,
                      'config': None, 'stop': None}
        self.topics = {name: None for name in TOPICS}
        self.cloud = (0, b'')
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        prefix = '/mid360_obstacle_avoidance/'
        self.create_subscription(RoiState, prefix+'roi_state', self.on_roi, latched)
        self.create_subscription(ObstacleStatus, prefix+'status', self.on_status, latched)
        self.create_subscription(Bool, prefix+'detection_enabled', self.on_detection, latched)
        self.create_subscription(Bool, prefix+'stop_required', self.on_stop, latched)
        self.create_subscription(DetectorConfig, prefix+'detector_config', self.on_config, latched)
        self.create_subscription(Marker, prefix+'roi_marker', self.on_marker, latched)
        self.create_subscription(PointCloud2, prefix+'pointcloud', self.on_cloud,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.roi_clients = {
            'set': self.create_client(SetRoiBoundary, prefix+'set_roi_boundary'),
            'save': self.create_client(Trigger, prefix+'save_roi'),
            'reset': self.create_client(Trigger, prefix+'reset_roi'),
            'apply_config': self.create_client(ApplyDetectorConfig, prefix+'apply_detector_config'),
        }

    def record_topic(self, name, payload):
        """Called while holding lock; retain one immutable message per topic."""
        previous = self.topics[name]
        self.topics[name] = dict(message=json_safe(payload), received=time.monotonic(),
            received_time=time.time(), count=1 if previous is None else previous['count']+1)

    def on_roi(self, msg):
        with self.lock:
            self.state['roi'] = roi_dict(msg)
            self.record_topic('roi_state', dict(message_to_ordereddict(msg)))

    def on_config(self, msg):
        with self.lock:
            self.state['config'] = config_dict(msg)
            self.record_topic('detector_config', self.state['config'])

    def on_marker(self, msg):
        with self.lock:
            self.record_topic('roi_marker', {k: json_safe(message_to_ordereddict(getattr(msg, k)))
                if k in ('header', 'pose', 'scale', 'color') else getattr(msg, k)
                for k in ('header', 'ns', 'id', 'type', 'action', 'pose', 'scale', 'color')})

    def on_stop(self, msg):
        with self.lock:
            self.state['stop'] = dict(data=msg.data, received=time.monotonic())
            self.record_topic('stop_required', {'data': msg.data})

    def on_status(self, msg):
        nearest = msg.nearest_distance
        with self.lock:
            self.state['status'] = dict(data_valid=msg.data_valid,
                stop_required=msg.stop_required, obstacle_detected=msg.obstacle_detected,
                obstacle_points=msg.obstacle_points,
                nearest_distance=nearest if math.isfinite(nearest) else None,
                nearest_kind=json_safe(nearest) if not math.isfinite(nearest) else 'finite',
                header=dict(message_to_ordereddict(msg.header)),
                stop_zone_points=msg.stop_zone_points,
                received=time.monotonic())
            self.record_topic('status', dict(message_to_ordereddict(msg)))

    def on_detection(self, msg):
        with self.lock:
            self.state['detection'] = dict(enabled=msg.data, received=time.monotonic())
            self.record_topic('detection_enabled', {'data': msg.data})

    def on_cloud(self, msg):
        try:
            payload = pack_xyz(msg)
        except (ValueError, TypeError, BufferError) as exc:
            self.get_logger().error(f'Invalid visualization cloud: {exc}')
            return
        with self.lock:
            seq = self.cloud[0] + 1
            self.cloud = (seq, payload)  # Single latest slot, never a queue.
            self.state['cloud'] = dict(sequence=seq, points=len(payload)//12,
                frame=msg.header.frame_id, sec=msg.header.stamp.sec,
                nanosec=msg.header.stamp.nanosec, received=time.monotonic())
            self.record_topic('pointcloud', dict(header=message_to_ordereddict(msg.header),
                height=msg.height, width=msg.width, is_dense=msg.is_dense,
                is_bigendian=msg.is_bigendian, point_step=msg.point_step, row_step=msg.row_step,
                fields=[dict(message_to_ordereddict(f)) for f in msg.fields], data_bytes=len(msg.data)))

    def snapshot(self):
        with self.lock:
            state = {k: dict(v) if v else None for k, v in self.state.items()}
            cloud = self.cloud
            topics = {k: dict(v) if v else None for k, v in self.topics.items()}
        now = time.monotonic()
        for key in ('status', 'cloud', 'detection', 'stop'):
            if state[key]:
                state[key]['age'] = now - state[key].pop('received')
        state['topics'] = {}
        for name, (kind, latched) in TOPICS.items():
            entry = topics[name]
            if entry:
                entry['age'] = now - entry.pop('received')
            state['topics'][name] = dict(topic='/mid360_obstacle_avoidance/'+name,
                type=kind, latched=latched, sample=entry)
        return state, cloud

    async def command(self, data):
        command = data.get('command')
        if command not in self.roi_clients:
            raise ValueError('Unknown command')
        client = self.roi_clients[command]
        if not client.service_is_ready():
            raise ValueError('Detector service unavailable')
        if command == 'set':
            value = data.get('value')
            name = data.get('boundary')
            if name not in ['roi_'+k for k in BOUNDS] or isinstance(value, bool) or not isinstance(value, (int,float)) or not math.isfinite(value):
                raise ValueError('Invalid boundary/value')
            request = SetRoiBoundary.Request(boundary=name, value=float(value))
        elif command == 'apply_config':
            values = data.get('parameters')
            if not isinstance(values, dict) or set(values) != set(ALGORITHM):
                raise ValueError('Exactly five algorithm parameters are required')
            for name, value in values.items():
                if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                    raise ValueError('Invalid finite numeric parameter: '+name)
                if name == 'min_obstacle_points':
                    if int(value) != value or not 1 <= value <= 4294967295:
                        raise ValueError('min_obstacle_points must be an integer from 1 to 4294967295')
                elif value < 0 or (name in ('cluster_distance_threshold', 'cloud_timeout') and value == 0):
                    raise ValueError('Invalid parameter range: '+name)
            request = ApplyDetectorConfig.Request(**{k: int(v) if k == 'min_obstacle_points'
                else float(v) for k, v in values.items()})
        else:
            request = Trigger.Request()
        loop = asyncio.get_running_loop()
        result = loop.create_future()
        ros_future = client.call_async(request)
        def finished(future):
            def complete():
                if not result.done():
                    try:
                        result.set_result(future.result())
                    except Exception as exc:
                        result.set_exception(exc)
            loop.call_soon_threadsafe(complete)
        ros_future.add_done_callback(finished)
        response = await asyncio.wait_for(result, 5.0)
        reply = dict(type='ack', id=data.get('id'), success=response.success,
                     message=response.message)
        if command == 'set':
            reply['roi'] = roi_dict(response.roi)
        if command == 'apply_config':
            reply['config'] = config_dict(response.config)
        return reply


def make_app(bridge, assets=None):
    @web.middleware
    async def revalidate(request, handler):
        response = await handler(request)
        if request.path == '/' or request.path.endswith(('.js', '.html')):
            response.headers['Cache-Control'] = 'no-cache'
        return response
    app = web.Application(client_max_size=4096, middlewares=[revalidate])
    app['command_lock'] = asyncio.Lock()
    app['sockets'] = set()
    root = Path(assets or Path(get_package_share_directory('mid360_obstacle_avoidance'))/'web')
    # Resolve the installed index symlink to serve its sibling source assets safely.
    root = (root/'index.html').resolve(strict=True).parent

    async def socket(request):
        # Browsers must connect from this same forwarded origin.
        origin = request.headers.get('Origin')
        if origin and origin != request.scheme+'://'+request.host:
            raise web.HTTPForbidden(text='Cross-origin WebSocket denied')
        ws = web.WebSocketResponse(heartbeat=10, max_msg_size=4096, compress=False)
        await ws.prepare(request)
        app['sockets'].add(ws)
        send_lock = asyncio.Lock()
        async def stream():
            last_seq = -1
            while not ws.closed:
                state, (seq, payload) = bridge.snapshot()
                async with send_lock:
                    # Bound writes; disconnect slow receivers instead of buffering old clouds.
                    await asyncio.wait_for(ws.send_json(dict(type='state', **state)), 2)
                    if seq != last_seq and seq:
                        await asyncio.wait_for(ws.send_bytes(payload), 2)
                        last_seq = seq
                await asyncio.sleep(0.1)
        async def supervise():
            try:
                await stream()
            except (ConnectionError, asyncio.TimeoutError, RuntimeError):
                await ws.close()
        sender = asyncio.create_task(supervise())
        try:
            async for msg in ws:
                if msg.type != WSMsgType.TEXT:
                    continue
                data = {}
                try:
                    data = json.loads(msg.data)
                    if not isinstance(data, dict):
                        raise ValueError('Expected an object')
                    async with app['command_lock']:
                        reply = await bridge.command(data)
                except (ValueError, TypeError, asyncio.TimeoutError) as exc:
                    reply = dict(type='ack', id=data.get('id') if isinstance(data,dict) else None,
                                 success=False, message=str(exc) or 'Service timeout; verify ROI before retry')
                async with send_lock:
                    await asyncio.wait_for(ws.send_json(reply), 2)
        finally:
            sender.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await sender
            app['sockets'].discard(ws)
        return ws

    async def index(_):
        return web.FileResponse(root/'index.html', headers={'Cache-Control':'no-cache'})

    async def shutdown(_):
        await asyncio.gather(*(ws.close(code=1001) for ws in list(app['sockets'])))
    app.on_shutdown.append(shutdown)
    app.router.add_get('/ws', socket)
    app.router.add_get('/', index)
    app.router.add_static('/', root, show_index=False)
    return app


def main():
    rclpy.init()
    bridge = Bridge()
    def spin():
        try:
            rclpy.spin(bridge)
        except ExternalShutdownException:
            pass
    worker = threading.Thread(target=spin, daemon=True)
    worker.start()
    bridge.get_logger().info(f'Web ROI editor: http://{bridge.host}:{bridge.port} (forward port in VS Code)')
    try:
        web.run_app(make_app(bridge), host=bridge.host, port=bridge.port, handle_signals=True)
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        worker.join(timeout=3)
        bridge.destroy_node()


if __name__ == '__main__':
    main()
