#!/usr/bin/env bash
set -eo pipefail

package_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ -f /opt/ros/humble/setup.bash ]]; then
  # ROS messages and rclpy are provided by the Humble system Python.
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi
if [[ "${ROS_DISTRO:-}" != "humble" ]]; then
  echo "ERROR: ROS 2 Humble 환경을 찾을 수 없습니다." >&2
  exit 2
fi
set -u

# The automated scenarios must never discover a distributed robot.  They
# instantiate no serial bridge, and localhost-only adds a second containment
# boundary in case a developer already has robot nodes on another domain.
export ROS_DOMAIN_ID="${FEATURE_TEST_DOMAIN_ID:-177}"
export ROS_LOCALHOST_ONLY=1
export PYTHONNOUSERSITE=1
export PYTHONPATH="$package_dir${PYTHONPATH:+:$PYTHONPATH}"

# Refuse to inject synthetic data into an already active local ROS graph.
# The DDS scenarios also remap every endpoint, but this catches accidental use
# of a domain that a same-host real launch already occupies.
existing_nodes="$(
  ros2 node list --all --no-daemon --spin-time 0.5 2>/dev/null \
    | sed -n -e '/^\/_ros2cli_/d' -e '/^\//p' || true
)"
if [[ -n "${existing_nodes//[[:space:]]/}" ]]; then
  echo "ERROR: ROS_DOMAIN_ID=$ROS_DOMAIN_ID 에 기존 local node가 있습니다:" >&2
  echo "$existing_nodes" >&2
  echo "실차 launch를 종료하거나 FEATURE_TEST_DOMAIN_ID를 바꾸세요." >&2
  exit 3
fi

feature="${1:-all}"
cd "$package_dir"

case "$feature" in
  perception)
    tests=(
      test/test_dual_cctv_merge.py
      test/test_cctv_vision_hardening.py
      test/test_yolo_camera_switching.py
      test/test_integration_scenarios.py::test_mvp_cctv_merge_receives_dual_detection_envelopes_and_publishes_origin_aware_map
    )
    ;;
  localization)
    tests=(
      test/test_localization_math.py
      test/test_relative_sync_filter.py
      test/test_integration_scenarios.py::test_pose_fusion_receives_dds_inputs_and_publishes_fused_odom
    )
    ;;
  fleet)
    tests=(
      test/test_fleet_retrieval_integration.py
      test/test_retrieval_planning.py
      test/test_integration_scenarios.py::test_fleet_dds_inputs_produce_correlated_registered_slot_plan
    )
    ;;
  entry)
    tests=(
      test/test_vehicle_entry.py
      test/test_ultrasonic_stm32_pipeline.py
      test/test_p1_entry_safety.py
      test/test_integration_scenarios.py::test_ultrasonic_edge_dds_inputs_produce_rear_axle_target
    )
    ;;
  mission)
    tests=(
      test/test_mission_safety.py
      test/test_ui_gate_and_mission_reset.py
      test/test_integration_scenarios.py::test_state_machine_dds_lift_barrier_uses_remapped_actuation_topics
    )
    ;;
  rigid-sync)
    tests=(
      test/test_motion_control.py
      test/test_relative_sync_filter.py
      test/test_rigid_body_p0_policy.py
      test/test_safe_relative_sensor_priority.py
      test/test_integration_scenarios.py::test_mvp_rigid_sync_dds_publishes_only_remapped_paired_commands
    )
    ;;
  integration)
    tests=(test/test_integration_scenarios.py)
    ;;
  all)
    tests=(test)
    ;;
  *)
    echo "usage: $0 {perception|localization|fleet|entry|mission|rigid-sync|integration|all}" >&2
    exit 2
    ;;
esac

echo "FEATURE TEST: $feature (ROS_DOMAIN_ID=$ROS_DOMAIN_ID, localhost only)"
/usr/bin/python3 -m pytest -q "${tests[@]}"
