"""Run on Linux against Python and installed/standalone C++; only temporary YAMLs/ports."""
import argparse
from concurrent.futures import ThreadPoolExecutor
from dataclasses import asdict
import json
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time
from urllib.error import HTTPError

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tools.board_verify import request, stop
from scan_web.detection import load_config
import yaml


def verify(name, command, output):
    with tempfile.TemporaryDirectory(prefix="scan-save-verify-") as directory:
        folder = Path(directory)
        target, link = folder / "target.yaml", folder / "installed.yaml"
        document = yaml.safe_load((ROOT / "config/scan_web.yaml").read_text())
        document["other"] = {"preserve": [1, "keep"]}
        document["mid360_suspended_object_detection"]["ros__parameters"]["min_z"] = 1.2
        target.write_text(yaml.safe_dump(document))
        link.symlink_to(target)
        with socket.socket() as listener:
            listener.bind(("127.0.0.1", 0))
            port = listener.getsockname()[1]
        log = (output / f"save-{name}.log").open("w")
        full = command + ["--config", str(link), "--port", str(port), "--ws-url", "ws://127.0.0.1:1"]

        def start():
            child = subprocess.Popen(full, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
            try:
                for _ in range(150):
                    if child.poll() is not None:
                        raise AssertionError(f"Backend exited: {name}")
                    try:
                        request(port, "/api/config")
                        return child
                    except OSError:
                        time.sleep(.1)
                raise AssertionError("Startup timed out")
            except BaseException:
                stop(child)
                raise
        child = start()
        try:
            original = target.read_bytes()
            request(port, "/api/config", {"min_x": -2})
            assert target.read_bytes() == original
            saved = request(port, "/api/config/save", {"min_y": -3, "min_cluster_points": 12})
            assert saved == asdict(load_config(target)) and saved["min_x"] == -2
            assert link.is_symlink()
            parsed = yaml.safe_load(target.read_text())
            assert parsed["other"] == document["other"]
            assert parsed["mid360_suspended_object_detection"]["ros__parameters"]["min_z"] == 1.2
            assert isinstance(parsed["mid360_suspended_object_detection"]["ros__parameters"]["min_cluster_points"], int)
            for changes in ({"min_x": 99}, {"min_cluster_points": 1.5}, {"path": "/tmp/forbidden"}):
                before = target.read_bytes()
                try:
                    request(port, "/api/config/save", changes)
                    raise AssertionError("Invalid save accepted")
                except HTTPError as error:
                    assert error.code == 400
                assert target.read_bytes() == before and request(port, "/api/config") == saved
            with ThreadPoolExecutor(max_workers=4) as pool:
                list(pool.map(lambda i: request(port, "/api/config/save", {"max_x": 5 + i}), range(20)))
            confirmed = request(port, "/api/config")
            assert confirmed == asdict(load_config(target))
            before = target.read_bytes()
            folder.chmod(0o500)
            try:
                try:
                    request(port, "/api/config/save", {"min_x": -4})
                    raise AssertionError("Write failure expected")
                except HTTPError as error:
                    assert error.code == 500
            finally:
                folder.chmod(0o700)
            assert target.read_bytes() == before and request(port, "/api/config") == confirmed
            assert not list(folder.glob(".scan-web-*"))
            stop(child)
            child = start()
            assert request(port, "/api/config") == confirmed
            return {"backend": name, "passed": True, "restart_retained": True, "symlink_preserved": True,
                    "memory_only_not_saved": True, "partial_save_merges": True, "concurrent_saves": 20,
                    "write_failure_atomic": True, "unrelated_fields_preserved": True, "port": port}
        finally:
            stop(child)
            log.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--binary", action="append", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    results = [verify("python", [sys.executable, "-m", "scan_web"], args.output)]
    for index, binary in enumerate(args.binary):
        results.append(verify(f"cpp-{index}", [str(binary.resolve())], args.output))
    (args.output / "http-save-verification.json").write_text(json.dumps(results, indent=2))
    print(json.dumps(results), flush=True)


if __name__ == "__main__":
    main()
