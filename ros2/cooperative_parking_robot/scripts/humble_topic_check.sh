#!/usr/bin/env bash
set -euo pipefail

if [[ "${ROS_DISTRO:-}" != "humble" ]]; then
  echo "ERROR: source /opt/ros/humble/setup.bash first" >&2
  exit 2
fi

role="${1:-}"
case "$role" in
  jetson)
    required=("${CCTV_RAW_TOPIC:-/cctv/image_raw}" "${CCTV_RECT_TOPIC:-/cctv/image_rect}" /parking/map /parking/target_pose /parking/empty_slots /virtual_robot/waypoints)
    ;;
  front)
    required=(/front/odom /front/cmd_vel /front/robot_state /front/ultrasonic_left /front/ultrasonic_right /rear/odom /sync/relative_pose)
    ;;
  rear)
    required=("${REAR_CAMERA_TOPIC:-/rear/marker_camera/image}" /rear/odom /rear/cmd_vel /rear/robot_state /rear/ultrasonic_left /rear/ultrasonic_right /sync/relative_pose)
    ;;
  *)
    echo "usage: $0 {jetson|front|rear}" >&2
    exit 2
    ;;
esac

mapfile -t topics < <(ros2 topic list)
missing=0
for topic in "${required[@]}"; do
  if printf '%s\n' "${topics[@]}" | grep -Fxq "$topic"; then
    echo "PASS $topic"
  else
    echo "MISS $topic"
    missing=1
  fi
done

exit "$missing"
