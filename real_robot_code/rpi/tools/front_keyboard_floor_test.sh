#!/bin/bash
set -eo pipefail

BRIDGE_PID=""
BRIDGE_LOG="/tmp/robot2_keyboard_bridge.log"

cleanup() {
  if [[ -n "${BRIDGE_PID}" ]] && kill -0 "${BRIDGE_PID}" 2>/dev/null; then
    # setsid로 분리된 ros2 CLI와 실제 bridge 노드를 함께 종료한다.
    kill -INT -- "-${BRIDGE_PID}" 2>/dev/null || true
    wait "${BRIDGE_PID}" 2>/dev/null || true
  fi
}

trap cleanup EXIT INT TERM

if pgrep -f '[s]tm32_bridge' >/dev/null; then
  echo "이미 STM32 bridge가 실행 중입니다. 먼저 기존 bridge를 종료하세요."
  exit 1
fi

source /opt/ros/humble/setup.bash
source /home/robot/cooperative_parking_robot_ws/install/setup.bash
export ROS_DOMAIN_ID=42
set -u

: >"${BRIDGE_LOG}"
# 터미널 Ctrl+C는 키보드 노드에만 전달한다. bridge는 마지막 0속도와
# manual-off를 받은 뒤 cleanup에서 별도로 종료한다.
setsid /home/robot/bridge_run.sh >"${BRIDGE_LOG}" 2>&1 &
BRIDGE_PID=$!
sleep 2

if ! kill -0 "${BRIDGE_PID}" 2>/dev/null; then
  echo "STM32 bridge 시작 실패:"
  tail -n 30 "${BRIDGE_LOG}"
  exit 1
fi

echo
echo "robot-2 바닥 키보드 시험"
echo "  W/S : 전진/후진"
echo "  A/D : 좌/우 횡이동"
echo "  Q/E : 좌/우 회전"
echo "  Space : 즉시 정지"
echo "  Ctrl+C : 정지 후 종료"
echo
echo "처음에는 각 키를 짧게 한 번씩만 누르세요. T/G(인양)는 사용하지 마세요."
echo

set +e
ros2 run cooperative_parking_robot keyboard_teleop --ros-args \
  -p role:=front \
  -p linear_speed_mps:=0.0628 \
  -p angular_speed_rps:=0.3142 \
  -p deadman_s:=0.30
TELEOP_STATUS=$?
set -e

# 마지막 0속도와 manual-off 메시지가 bridge에 도달할 시간을 준다.
sleep 0.5
exit "${TELEOP_STATUS}"
