"""Verify honest unsupported/disabled publication states on temporary ports/configs."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tools.board_verify import request, stop
from scan_web.detection import DetectionConfig


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--ros', required=True)
    parser.add_argument('--standalone', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    reports = []
    for name, command, options in (
        ('python', [sys.executable, '-m', 'scan_web'], []),
        ('standalone', [args.standalone], []),
        ('disabled', [args.ros], ['--ros-args', '-p', 'publish_obstacle:=false', '-p',
                                'obstacle_topic:=/mid360_test/disabled', '-r',
                                '/mid360_test/disabled:=/mid360_test/disabled_remap'])):
        with tempfile.TemporaryDirectory(prefix='mid360-modes-') as folder:
            config = Path(folder) / 'scan.yaml'
            config.write_text(yaml.safe_dump({'mid360_suspended_object_detection': {'ros__parameters': asdict(DetectionConfig())}}))
            with socket.socket() as listener:
                listener.bind(('127.0.0.1', 0))
                port = listener.getsockname()[1]
            with (args.output / f'mode-{name}.log').open('w') as log:
                child = subprocess.Popen(command + ['--config', str(config), '--port', str(port),
                                                    '--ws-url', 'ws://127.0.0.1:1'] + options,
                                         cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
                try:
                    for _ in range(100):
                        assert child.poll() is None, name
                        try:
                            state = request(port)
                            break
                        except OSError:
                            time.sleep(.1)
                    else:
                        raise AssertionError('HTTP not available')
                    publication = state['ros_publication']
                    assert publication['status'] == ('disabled' if name == 'disabled' else 'unsupported')
                    assert publication['supported'] == (name == 'disabled')
                    assert not publication['enabled'] and publication['message'] is None
                    assert publication['published_count'] == 0 and publication['frequency_hz'] is None
                    assert state['largest_obstacle'] is None
                    if name == 'disabled':
                        assert publication['topic'] == '/mid360_test/disabled_remap'
                    reports.append(dict(backend=name, passed=True, publication=publication))
                finally:
                    stop(child)
    (args.output / 'backend-modes.json').write_text(json.dumps(reports, indent=2))
    print(json.dumps(reports))


if __name__ == '__main__':
    main()
