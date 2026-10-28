#!/usr/bin/env bash
set -eo pipefail
cd /home/neardi/ws_livox
source /opt/ros/jazzy/setup.bash
source install/setup.bash
pkg=/home/neardi/ws_livox/src/mid360_suspended_object_detection
out="$pkg/test-results/web-publication"
mkdir -p "$out"
colcon build --packages-select mid360_suspended_object_detection --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DPython3_EXECUTABLE=/usr/bin/python3 >"$out/ros-build.log" 2>&1
source install/setup.bash
ctest --test-dir build/mid360_suspended_object_detection --output-on-failure >"$out/cpp-tests.log" 2>&1
colcon test --packages-select mid360_suspended_object_detection >"$out/colcon-test.log" 2>&1
colcon test-result --test-result-base build/mid360_suspended_object_detection >"$out/colcon-test-result.log" 2>&1
cmake -S "$pkg" -B /tmp/mid360-publication-standalone -DBUILD_ROS2=OFF -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release >"$out/standalone.log" 2>&1
cmake --build /tmp/mid360-publication-standalone -j2 >>"$out/standalone.log" 2>&1
ctest --test-dir /tmp/mid360-publication-standalone --output-on-failure >>"$out/standalone.log" 2>&1
cd "$pkg"
python3 -m unittest discover -s tests -p 'test_*.py' >"$out/python-tests.log" 2>&1
node tests/test_roi.js >"$out/roi-tests.log" 2>&1
node tests/test_roi_interaction.js >>"$out/roi-tests.log" 2>&1
binary=/home/neardi/ws_livox/install/mid360_suspended_object_detection/lib/mid360_suspended_object_detection/scan_web_server
python3 tests/ros_replay_check.py --binary "$binary" --output "$out" >"$out/ros-replay-check.log" 2>&1
python3 tests/backend_modes_check.py --ros "$binary" --standalone /tmp/mid360-publication-standalone/scan_web_server --output "$out" >"$out/backend-modes.log" 2>&1
python3 tools/config_save_verify.py --binary "$binary" --binary /tmp/mid360-publication-standalone/scan_web_server --output "$out" >"$out/config-save-check.log" 2>&1
printf 'All board checks passed\n'
