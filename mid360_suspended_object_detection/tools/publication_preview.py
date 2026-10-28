"""Bounded, synthetic-only browser preview; never uses the saved deployment YAML."""
import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tools.board_verify import Replay, stop, wait_ready, request
from scan_web.detection import DetectionConfig


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--binary', required=True)
    parser.add_argument('--seconds', type=int, default=240)
    args = parser.parse_args()
    scan = dict(rame_id='scan_link', name='synthetic_preview', scan=[0] * 6000,
                angle_start=-math.pi, increment=2 * math.pi / 6000,
                range_min=.03, range_max=50, timeoutSec=0, laserOffset=[0, 0, 0])
    for angle, radius, count in ((0, 2000, 40), (.65, 3000, 70), (-.65, 2000, 45)):
        center = round((angle + math.pi) / scan['increment'])
        for i in range(-count, count + 1):
            scan['scan'][center + i] = radius + (100 if i % 2 else 0)
    replay = Replay([dict(raw=json.dumps(dict(channel='ros', op='publish', topic='/scan', msg=[scan])), delay=.02)])
    output = ROOT / 'test-results/web-publication'
    with tempfile.TemporaryDirectory(prefix='mid360-preview-') as folder:
        config = Path(folder) / 'scan.yaml'
        config.write_text(yaml.safe_dump(dict(mid360_suspended_object_detection=dict(ros__parameters=asdict(
            DetectionConfig(min_x=-4, max_x=4, min_y=-4, max_y=4))))))
        with (output / 'preview-backend.log').open('w') as log:
            child = subprocess.Popen([args.binary, '--config', str(config), '--port', '18768', '--ws-url',
                                      f'ws://127.0.0.1:{replay.port}', '--ros-args', '-p',
                                      'obstacle_topic:=/mid360_test/browser_preview'], cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
            try:
                state = wait_ready(18768, child)
                assert len(state['objects']) == 3 and state['largest_obstacle']
                print('Synthetic ROS browser preview ready on 127.0.0.1:18768', flush=True)
                time.sleep(args.seconds)
                (output / 'preview-final-state.json').write_text(json.dumps(request(18768), indent=2))
            finally:
                stop(child)
                replay.close()


if __name__ == '__main__':
    main()
