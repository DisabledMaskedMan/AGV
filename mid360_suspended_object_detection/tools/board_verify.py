"""Run on the board: record once, replay identically, benchmark both backends."""
import argparse
import base64
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from urllib.error import HTTPError
from urllib.request import Request, urlopen

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from scan_web.detection import DetectionConfig, detect
from scan_web.protocol import parse_scan


def request(port, path="/api/state", data=None):
    body = None if data is None else json.dumps(data).encode()
    with urlopen(Request(f"http://127.0.0.1:{port}{path}", data=body,
                         headers={"Content-Type": "application/json"}), timeout=5) as response:
        return json.load(response)


class Replay:
    def __init__(self, records, port=0):
        self.records = records
        self.listener = socket.socket()
        self.listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.listener.bind(("127.0.0.1", port))
        self.listener.listen()
        self.listener.settimeout(0.25)
        self.port = self.listener.getsockname()[1]
        self.stopping = threading.Event()
        self.clients = []
        self.workers = []
        self.thread = threading.Thread(target=self.serve, daemon=True)
        self.thread.start()

    def serve(self):
        while not self.stopping.is_set():
            try:
                client, _ = self.listener.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            self.clients.append(client)
            worker = threading.Thread(target=self.send, args=(client,), daemon=True)
            self.workers.append(worker)
            worker.start()

    def send(self, client):
        try:
            client.settimeout(3)
            header = b""
            while b"\r\n\r\n" not in header:
                chunk = client.recv(4096)
                if not chunk:
                    return
                header += chunk
            key = next(line.split(b":", 1)[1].strip() for line in header.split(b"\r\n")
                       if line.lower().startswith(b"sec-websocket-key:"))
            accept = base64.b64encode(hashlib.sha1(key + b"258EAFA5-E914-47DA-95CA-C5AB0DC85B11").digest())
            client.sendall(b"HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Accept: " + accept + b"\r\n\r\n")
            schedule = time.monotonic()
            while not self.stopping.is_set():
                for record in self.records:
                    if self.stopping.wait(max(0, schedule - time.monotonic())):
                        return
                    payload = record["raw"].encode()
                    length = len(payload)
                    prefix = bytes([0x81, length]) if length < 126 else (b"\x81\x7e" + struct.pack("!H", length) if length < 65536
                                                                      else b"\x81\x7f" + struct.pack("!Q", length))
                    client.sendall(prefix + payload)
                    schedule += record["delay"]
        except (OSError, StopIteration):
            pass
        finally:
            client.close()

    def close(self):
        self.stopping.set()
        self.listener.close()
        for client in self.clients:
            try:
                client.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        self.thread.join(2)
        for worker in self.workers:
            worker.join(2)


def wait_ready(port, process, timeout=15):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f"Backend exited: {process.returncode}")
        try:
            state = request(port)
            if state["status"] in ("clear", "detected"):
                return state
        except OSError:
            pass
        time.sleep(0.1)
    raise RuntimeError("Backend did not become ready")


def stop(process):
    process.send_signal(signal.SIGTERM)
    try:
        process.wait(8)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait()


def process_stats(pid):
    fields = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
    cpu = (int(fields[11]) + int(fields[12])) / os.sysconf("SC_CLK_TCK")
    rss = int(Path(f"/proc/{pid}/statm").read_text().split()[1]) * os.sysconf("SC_PAGE_SIZE") / 1048576
    return cpu, rss


def percentile(values, fraction):
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, math.ceil(len(ordered) * fraction) - 1)] if values else None


def compare_fixture(binary, records):
    inputs = []
    expected = []
    previous = None
    for record in records:
        message = json.loads(record["raw"])
        packet = parse_scan(message)
        if packet is None or packet == previous:
            continue
        previous = packet
        for changes in ({}, {"min_x": -5, "max_x": 5, "min_y": -5, "max_y": 5}):
            config = DetectionConfig().updated(changes)
            inputs.append(json.dumps({"message": message, "config": changes}))
            points = packet.points()
            expected.append(dict(detect(points, config), points=points))
        if len(inputs) >= 20:
            break
    process = subprocess.run([str(binary)], input="\n".join(inputs) + "\n", text=True, capture_output=True, check=True)
    actual = [json.loads(line) for line in process.stdout.splitlines()]
    assert len(actual) == len(expected) and actual
    for first, second in zip(expected, actual):
        assert first["roi_count"] == second["roi_count"]
        assert len(first["points"]) == len(second["points"])
        for point_a, point_b in zip(first["points"], second["points"]):
            assert all(abs(value_a - value_b) < 1e-8 for value_a, value_b in zip(point_a, point_b))
        assert len(first["objects"]) == len(second["objects"])
        for box_a, box_b in zip(first["objects"], second["objects"]):
            assert all(abs(box_a[key] - box_b[key]) < 1e-8 for key in box_a)
    return {"fixtures": len(actual), "absolute_tolerance": 1e-8, "passed": True}


def benchmark(binary, records, output, seconds=60, warmup=10):
    replay = Replay(records)
    results = []
    try:
        for name, command in (("python", [sys.executable, "-m", "scan_web"]), ("cpp", [str(binary)])):
            for roi_name, roi in (("default", {}), ("expanded", {"min_x": -5, "max_x": 5, "min_y": -5, "max_y": 5})):
                port = 18765
                with (output / f"{name}-{roi_name}.log").open("w") as log:
                    process = subprocess.Popen(command + ["--ws-url", f"ws://127.0.0.1:{replay.port}", "--port", str(port)],
                                               cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
                    try:
                        wait_ready(port, process)
                        request(port, "/api/config", roi)
                        deadline = time.monotonic() + warmup
                        while time.monotonic() < deadline:
                            request(port)
                            time.sleep(0.2)
                        before = request(port)
                        cpu_before, _ = process_stats(process.pid)
                        started = time.monotonic()
                        samples = []
                        schedule = started
                        while time.monotonic() - started < seconds:
                            state = request(port)
                            _, rss = process_stats(process.pid)
                            samples.append({key: state[key] for key in ("status", "processing_ms", "processed_age_s", "receive_age_s", "processed_count", "received_count", "content_version", "reconnects")})
                            samples[-1]["rss_mb"] = rss
                            schedule += 0.2
                            time.sleep(max(0, schedule - time.monotonic()))
                        elapsed = time.monotonic() - started
                        cpu_after, _ = process_stats(process.pid)
                        after = request(port)
                        healthy = [sample for sample in samples if sample["status"] in ("clear", "detected")]
                        result = {"backend": name, "roi": roi_name, "seconds": elapsed,
                                  "cpu_percent_one_core": (cpu_after - cpu_before) / elapsed * 100,
                                  "rss_mb_mean": sum(sample["rss_mb"] for sample in samples) / len(samples),
                                  "rss_mb_max": max(sample["rss_mb"] for sample in samples),
                                  "processing_ms_p50": percentile([sample["processing_ms"] for sample in healthy], .5),
                                  "processing_ms_p95": percentile([sample["processing_ms"] for sample in healthy], .95),
                                  "result_age_s_p95": percentile([sample["processed_age_s"] for sample in healthy], .95),
                                  "analysis_hz": (after["processed_count"] - before["processed_count"]) / elapsed,
                                  "receive_hz": (after["received_count"] - before["received_count"]) / elapsed,
                                  "unhealthy_samples": len(samples) - len(healthy), "reconnects": after["reconnects"],
                                  "samples": samples}
                        results.append(result)
                        (output / "benchmark.json").write_text(json.dumps(results, indent=2), encoding="utf-8")
                        print(json.dumps({key: value for key, value in result.items() if key != "samples"}), flush=True)
                    finally:
                        stop(process)
    finally:
        replay.close()
    return results


def integration(binary, records, output, check_ros=False):
    synthetic_records = []
    for record in records:
        message = json.loads(record["raw"])
        for item in message.get("msg", []):
            if str(item.get("rame_id", "")).lstrip("/") != "scan_link":
                continue
            scan = [0] * len(item["scan"])
            center = round((-0.197 - item["angle_start"]) / item["increment"])
            for index in range(center - 40, center + 41):
                scan[index] = round((2 + (index - center) * 0.002) * 1000)
            item["scan"] = scan
            item["timeoutSec"] = 0
        synthetic_records.append({"raw": json.dumps(message), "delay": record["delay"]})
    replay = Replay(synthetic_records)
    log = (output / "integration.log").open("w")
    process = subprocess.Popen([str(binary), "--ws-url", f"ws://127.0.0.1:{replay.port}", "--port", "18766"],
                               cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
    try:
        initial = wait_ready(18766, process)
        for asset in ("/", "/app.js", "/roi.js", "/style.css"):
            with urlopen(f"http://127.0.0.1:18766{asset}", timeout=5) as response:
                assert response.status == 200
        for changes in ({"min_x": 9}, {"min_cluster_points": 1.2}, {"min_z": 0}, {"cloud_timeout": True}):
            try:
                request(18766, "/api/config", changes)
                raise AssertionError("Expected invalid update rejection")
            except HTTPError as error:
                assert error.code == 400
            assert request(18766, "/api/config") == initial["config"]
        request(18766, "/api/config", {"min_x": -5, "max_x": 5, "min_y": -5, "max_y": 5})
        time.sleep(0.3)
        assert request(18766)["roi_count"] > 0
        def topic_check(seconds, *options):
            if not check_ros:
                return {"skipped": "Enable --check-ros for a ROS-enabled backend"}
            result = subprocess.run(
                [sys.executable, str(ROOT / "tests" / "ros_topic_check.py"), "--seconds", str(seconds), *options],
                cwd=ROOT, capture_output=True, text=True, timeout=seconds + 8)
            if result.returncode:
                raise AssertionError(result.stdout + result.stderr)
            return json.loads(result.stdout)
        topic_report = topic_check(4, "--require-detected")
        with ThreadPoolExecutor(max_workers=4) as pool:
            list(pool.map(lambda index: request(18766, "/api/config", {"max_x": 5 + index % 2}), range(40)))
        malformed = json.loads(records[0]["raw"])
        for item in malformed["msg"]:
            if str(item.get("rame_id", "")).lstrip("/") == "scan_link":
                item["timeoutSec"] = 60
        replay.records = [{"raw": json.dumps(malformed), "delay": 0.02}]
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            state = request(18766)
            if state["status"] == "invalid":
                assert not state["objects"] and not state["points"]
                break
            time.sleep(0.05)
        else:
            raise AssertionError("Sensor timeout did not clear results")
        invalid_topic = topic_check(2, "--expect-status", "invalid")
        replay.records = records
        wait_ready(18766, process)
        replay_port = replay.port
        replay.close()
        time.sleep(1.5)
        state = request(18766)
        assert state["status"] in ("timeout", "disconnected", "connecting")
        assert state["objects"] == []
        disconnected_topic = topic_check(2, "--expect-status", state["status"])
        replay = Replay(records, replay_port)
        recovered = wait_ready(18766, process)
        assert recovered["reconnects"] > 0
        return {"assets": True, "atomic_config": True, "expanded_roi": True,
                "ros_topic": topic_report, "ros_invalid": invalid_topic,
                "ros_disconnected": disconnected_topic,
                "concurrent_updates": True, "sensor_timeout_clears": True,
                "disconnect_clears": True, "reconnect_recovers": True}
    finally:
        stop(process)
        log.close()
        if not replay.stopping.is_set():
            replay.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--fixture", type=Path, required=True)
    parser.add_argument("--check-ros", action="store_true", help="Check custom ROS topic (requires ROS-enabled binary and sourced overlay)")
    parser.add_argument("--source", default="ws://192.168.31.7:6060")
    parser.add_argument("--allow-reported-timeout-replay", action="store_true",
                        help="Replay only: preserve original records and normalize timeout metadata; never a live acceptance")
    parser.add_argument("--output", type=Path, default=ROOT / "test-results" / "rk3576")
    arguments = parser.parse_args()
    output = arguments.output
    output.mkdir(parents=True, exist_ok=True)
    import websocket
    connection = websocket.create_connection(arguments.source, timeout=5, skip_utf8_validation=True)
    records = []
    originals = []
    reported_timeouts = set()
    rejected_messages = 0
    try:
        connection.send(json.dumps({"channel": "ros", "op": "subscribe_scan", "topic": "/scan"}))
        started = time.monotonic()
        while time.monotonic() - started < 4:
            raw = connection.recv()
            message = json.loads(raw)
            originals.append(raw)
            if arguments.allow_reported_timeout_replay and message.get("op") == "publish":
                for item in message.get("msg", []):
                    if str(item.get("rame_id", "")).lstrip("/") == "scan_link":
                        reported_timeouts.add(item.get("timeoutSec", 0))
                        item["timeoutSec"] = 0
                raw = json.dumps(message)
            try:
                packet = parse_scan(message)
            except ValueError:
                rejected_messages += 1
                continue
            if packet is None:
                continue
            records.append({"raw": raw, "delay": 1 / 65})
    finally:
        connection.close()
    assert records, "No real scan received"
    (output / "source-original.json").write_text(json.dumps(originals), encoding="utf-8")
    (output / "replay.json").write_text(json.dumps(records), encoding="utf-8")
    report = {"source": arguments.source, "recorded_messages": len(records),
              "reported_timeouts": sorted(reported_timeouts),
              "rejected_source_messages": rejected_messages,
              "timeout_normalized_for_replay_only": arguments.allow_reported_timeout_replay,
              "replay_hz": 65, "warmup_s": 10, "measurement_s": 60, "poll_hz": 5,
              "fixture": compare_fixture(arguments.fixture.resolve(), records),
              "integration": integration(arguments.binary.resolve(), records, output, check_ros=arguments.check_ros)}
    (output / "verification.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report), flush=True)
    benchmark(arguments.binary.resolve(), records, output)


if __name__ == "__main__":
    main()
