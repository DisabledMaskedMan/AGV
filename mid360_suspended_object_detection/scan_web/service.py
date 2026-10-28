import json
import threading
import time
from collections import deque
from dataclasses import asdict
from pathlib import Path
from urllib.parse import urlsplit

import websocket

from .detection import detect, save_config, largest_obstacle
from .protocol import parse_scan


class ScanService:
    def __init__(self, config, url="ws://192.168.20.199:6060", frame="scan_link",
                 connection_factory=None, clock=time.monotonic, retry_delays=(1, 2, 5), config_path=None):
        self.url = url
        self.frame = frame
        self.config = config
        self.config_path = Path(config_path).resolve(strict=True) if config_path is not None else None
        self.factory = connection_factory or websocket.create_connection
        self.clock = clock
        self.retry_delays = retry_delays
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.socket = None
        self.threads = []
        self.connection = "waiting"
        self.error = None
        self.packet = None
        self.received_at = None
        self.received_count = 0
        self.content_version = 0
        self.content_changed_at = None
        self.config_version = 0
        self.analysis = None
        self.analysis_key = None
        self.processed_count = 0
        self.processed_at = None
        self.processing_ms = 0.0
        self.receive_times = deque(maxlen=4096)
        self.change_times = deque(maxlen=4096)
        self.reconnects = 0
        self.source_generation = 0

    def start(self):
        if self.threads:
            raise RuntimeError("服务已经启动")
        self.threads = [threading.Thread(target=self._reader, name="scan-reader", daemon=True),
                        threading.Thread(target=self._processor, name="scan-detector", daemon=True)]
        for thread in self.threads:
            thread.start()

    def close(self):
        self.stop_event.set()
        with self.lock:
            socket = self.socket
        if socket is not None:
            try:
                socket.shutdown()
            except (OSError, websocket.WebSocketException):
                pass
        for thread in self.threads:
            thread.join(timeout=6)

    def update_config(self, changes, persist=False):
        with self.lock:
            next_config = self.config.updated(changes)
            if persist:
                if self.config_path is None:
                    raise OSError("未配置 YAML 保存路径")
                save_config(self.config_path, next_config)
            self.config = next_config
            self.config_version += 1
            self.analysis = None
            return asdict(self.config)

    def get_config(self):
        with self.lock:
            return asdict(self.config)

    def accept(self, packet):
        now = self.clock()
        with self.lock:
            if packet != self.packet:
                self.content_version += 1
                self.content_changed_at = now
                self.change_times.append(now)
            self.packet = packet
            self.received_at = now
            self.received_count += 1
            self.receive_times.append(now)
            self.connection = "streaming"
            self.error = None

    def fail(self, message, connection="invalid"):
        with self.lock:
            self.connection = connection
            self.error = str(message)
            self.packet = None
            self.analysis = None
            self.source_generation += 1

    def _reader(self):
        failures = 0
        while not self.stop_event.is_set():
            socket = None
            try:
                with self.lock:
                    self.connection = "connecting"
                socket = self.factory(self.url, timeout=5, skip_utf8_validation=True,
                                      http_no_proxy=[urlsplit(self.url).hostname])
                with self.lock:
                    self.socket = socket
                    self.connection = "waiting"
                socket.send(json.dumps({"channel": "ros", "op": "subscribe_scan", "topic": "/scan"}))
                last_target = self.clock()
                while not self.stop_event.is_set():
                    with self.lock:
                        timeout = self.config.cloud_timeout
                    if self.clock() - last_target > timeout:
                        raise TimeoutError("目标扫描接收超时")
                    socket.settimeout(min(0.25, timeout))
                    try:
                        raw = socket.recv()
                    except websocket.WebSocketTimeoutException:
                        continue
                    if not raw:
                        raise ConnectionError("WebSocket 已关闭")
                    try:
                        packet = parse_scan(json.loads(raw), self.frame)
                    except (ValueError, TypeError, OverflowError) as error:
                        self.fail(error)
                        continue
                    if packet is not None:
                        self.accept(packet)
                        last_target = self.clock()
                        failures = 0
            except (OSError, websocket.WebSocketException, ValueError) as error:
                if not self.stop_event.is_set():
                    self.fail(error, "disconnected")
            finally:
                with self.lock:
                    self.socket = None
                if socket is not None:
                    try:
                        socket.close(timeout=0.2)
                    except (OSError, websocket.WebSocketException):
                        pass
            if not self.stop_event.is_set():
                delay = self.retry_delays[min(failures, len(self.retry_delays) - 1)]
                failures += 1
                with self.lock:
                    self.reconnects += 1
                self.stop_event.wait(delay)

    def process_latest(self):
        with self.lock:
            packet = self.packet
            config = self.config
            key = (self.content_version, self.config_version)
            generation = self.source_generation
            if packet is None or key == self.analysis_key or self.clock() - self.received_at > config.cloud_timeout:
                return
        started = time.perf_counter()
        points = packet.points()
        analysis = dict(detect(points, config), points=points, valid_count=len(points))
        analysis["largest_obstacle"] = largest_obstacle(analysis["objects"])
        processing_ms = (time.perf_counter() - started) * 1000
        finished = self.clock()
        with self.lock:
            if (self.packet is not None and generation == self.source_generation
                    and key[1] == self.config_version):
                self.analysis = analysis
                self.analysis_key = key
                self.processed_count += 1
                self.processed_at = finished
                self.processing_ms = processing_ms

    def _processor(self):
        while not self.stop_event.is_set():
            started = self.clock()
            self.process_latest()
            self.stop_event.wait(max(0.0, 0.1 - (self.clock() - started)))

    @staticmethod
    def _rate(events, now):
        recent = [value for value in events if now - value <= 5]
        if len(recent) < 2:
            return 0.0
        return (len(recent) - 1) / max(now - recent[0], 0.001)

    def state(self):
        now = self.clock()
        with self.lock:
            age = None if self.received_at is None else max(0.0, now - self.received_at)
            status = self.connection
            error = self.error
            current = self.analysis_key is not None and self.analysis_key[1] == self.config_version
            analysis = self.analysis if current and self.analysis is not None else {}
            pending = self.analysis_key is None or self.analysis_key[0] != self.content_version
            if pending and self.processed_at is not None and now - self.processed_at > self.config.cloud_timeout:
                analysis = {}
            if self.packet is not None and age > self.config.cloud_timeout:
                status, error, analysis = "timeout", "目标扫描接收超时", {}
            elif self.packet is not None:
                status = "processing" if not analysis else ("detected" if analysis["objects"] else "clear")
            packet = self.packet
            return {
                "status": status, "connection": self.connection, "error": error,
                "source_url": self.url, "target_frame": self.frame,
                "frame": packet.frame if packet else None,
                "name": packet.name if packet else None,
                "source_timestamp": None, "distance_unit": "m", "scan_unit": "mm",
                "laser_offset": packet.laser_offset if packet else None,
                "scan_count": len(packet.ranges) if packet else 0,
                "points": analysis.get("points", []), "valid_count": analysis.get("valid_count", 0),
                "roi_count": analysis.get("roi_count", 0), "objects": analysis.get("objects", []),
                "largest_obstacle": analysis.get("largest_obstacle"),
                "ros_publication": {"supported": False, "enabled": False, "topic": None,
                    "message_type": "mid360_suspended_object_detection/msg/LargestObstacle",
                    "status": "unsupported", "frequency_hz": None, "published_count": 0,
                    "last_publish_age_s": None, "error": "", "message": None},
                "detected": status == "detected", "receive_age_s": age,
                "content_age_s": None if self.content_changed_at is None else max(0, now - self.content_changed_at),
                "receive_hz": self._rate(self.receive_times, now),
                "content_hz": self._rate(self.change_times, now),
                "received_count": self.received_count, "content_version": self.content_version,
                "analyzed_content_version": self.analysis_key[0] if analysis else None,
                "processed_count": self.processed_count, "processing_ms": self.processing_ms,
                "processed_age_s": None if self.processed_at is None else max(0, now - self.processed_at),
                "reconnects": self.reconnects, "config": asdict(self.config),
            }
