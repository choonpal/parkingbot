#!/usr/bin/env bash
set -euo pipefail

if [[ "${ROS_DISTRO:-}" != "humble" ]]; then
  echo "ERROR: source /opt/ros/humble/setup.bash first (ROS_DISTRO=${ROS_DISTRO:-unset})" >&2
  exit 2
fi

# ROS 2 Humble binaries use Ubuntu's Python 3.10 stack. A newer ~/.local
# setuptools can make colcon report success after collecting zero tests.
export PYTHONNOUSERSITE=1

for cmd in ros2 rosdep colcon; do
  command -v "$cmd" >/dev/null 2>&1 || {
    echo "ERROR: required command not found: $cmd" >&2
    exit 2
  }
done

workspace="${1:-$PWD}"
package_dir="$workspace/src/cooperative_parking_robot"
if [[ ! -f "$package_dir/package.xml" ]]; then
  echo "ERROR: workspace not found: $package_dir" >&2
  echo "usage: $0 /absolute/path/to/cooperative_parking_robot_ws" >&2
  exit 2
fi

cd "$workspace"
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select cooperative_parking_robot
# shellcheck disable=SC1091
source install/setup.bash
colcon test --packages-select cooperative_parking_robot --event-handlers console_direct+
test_result="$(colcon test-result --verbose)"
printf '%s\n' "$test_result"
if grep -Eq 'Summary:[[:space:]]+0 tests' <<<"$test_result"; then
  echo "ERROR: colcon collected zero tests" >&2
  exit 1
fi
ros2 pkg executables cooperative_parking_robot

echo "HUMBLE BUILD/TEST: PASS"
