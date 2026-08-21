"""Mission barrier, freshness, and post-release exit regression tests."""

from pathlib import Path

from cooperative_parking_robot.freshness import (
    NSEC_PER_SEC,
    RequestReplayGuard,
    StampGate,
)


ROOT = Path(__file__).resolve().parents[1]


def source(relative):
    return (ROOT / relative).read_text()


def test_stamp_gate_rejects_zero_stale_future_duplicate_and_out_of_order():
    now = 10 * NSEC_PER_SEC

    gate = StampGate(max_age_s=0.5, future_tolerance_s=0.1)
    assert gate.accept(0, now) == (False, "ZERO_STAMP")
    assert gate.accept(now - NSEC_PER_SEC, now) == (
        False, "STALE_STAMP")
    assert gate.accept(now + NSEC_PER_SEC, now) == (
        False, "FUTURE_STAMP")

    stamp = now - 100_000_000
    assert gate.accept(stamp, now) == (True, "OK")
    assert gate.accept(stamp, now) == (
        False, "DUPLICATE_OR_OUT_OF_ORDER")
    assert gate.accept(stamp - 1, now) == (
        False, "DUPLICATE_OR_OUT_OF_ORDER")
    assert gate.accept(stamp + 1, now) == (True, "OK")


def test_request_replay_guard_scopes_sequence_by_client_and_request_id():
    guard = RequestReplayGuard(max_clients=2, max_request_ids=3)

    assert guard.accept('web-a', 1, 'req-1') == (True, 'OK')
    assert guard.accept('web-a', 1, 'req-2') == (
        False, 'DUPLICATE_SEQUENCE')
    assert guard.accept('web-b', 1, 'req-2') == (True, 'OK')
    assert guard.accept('web-c', 1, 'req-3') == (True, 'OK')
    assert guard.accept('web-a', 2, 'req-1') == (
        False, 'DUPLICATE_REQUEST_ID')


def test_request_replay_guard_keeps_legacy_global_sequence():
    guard = RequestReplayGuard()

    assert guard.accept('', 7, '') == (True, 'OK')
    assert guard.accept('', 7, '') == (False, 'DUPLICATE_SEQUENCE')
    assert guard.accept('', 8, '') == (True, 'OK')


def test_state_machine_uses_mission_scoped_two_phase_barriers():
    state = source(
        "cooperative_parking_robot/robot_state_machine_node.py")
    assert '"mission_id"' in state
    assert '"sequence"' in state
    assert '"stamp_ns"' in state
    assert 'f"/mission/{self.role}/ready"' in state
    assert '"/mission/commit"' in state
    assert "DurabilityPolicy.TRANSIENT_LOCAL" in state

    for stage in ("LIFT", "DRIVE", "RELEASE", "RETURN", "HOME"):
        assert f'self.publish_ready_stage("{stage}")' in state
        assert f'self.maybe_publish_commit("{stage}")' in state
        assert f'"{stage}" in self.committed_stages' in state

    assert 'self.transition("WAIT_RELEASE")' in state
    assert "FLEET_STATE_TIMEOUT" in state
    assert "MISSION_CHANGED_WHILE_ACTIVE" in state


def test_mission_complete_is_gated_by_both_home_ready_commit():
    state = source(
        'cooperative_parking_robot/robot_state_machine_node.py')
    return_block = state.split(
        'elif self.state == "RETURN":', 1)[1].split(
        'elif self.state == "FAULT":', 1)[0]

    assert 'self.publish_ready_stage("HOME")' in return_block
    assert 'self.maybe_publish_commit("HOME")' in return_block
    assert 'if "HOME" in self.committed_stages:' in return_block
    assert return_block.index('if "HOME" in self.committed_stages:') < (
        return_block.index('self.publish_mission_complete()'))


def test_fleet_registry_changes_are_mission_type_and_commit_scoped():
    fleet = source('cooperative_parking_robot/fleet_manager_node.py')
    assert 'self.registry = ParkingRegistry(' in fleet
    assert "self.mission_type == 'park'" in fleet
    assert "self.mission_type == 'retrieve'" in fleet
    assert "stage == 'DRIVE'" in fleet
    assert "stage == 'RETURN'" in fleet
    assert "'RELEASE' in self.active_committed_stages" in fleet
    assert 'self.registry.complete_park(' in fleet
    assert 'self.registry.complete_retrieve(' in fleet
    assert 'self.registry' not in fleet.split(
        'def _finalize_mission', 1)[1].split('def lifted_cb', 1)[0]


def test_fleet_arrival_requires_exact_active_plan_stamp():
    fleet = source('cooperative_parking_robot/fleet_manager_node.py')
    callback = fleet.split('def sync_status_cb', 1)[1].split(
        'def mission_commit_cb', 1)[0]
    assert 'parse_arrival_status(' in callback
    assert 'self.active_plan_stamp_ns' in callback
    assert 'self.pending_final_vehicle_pose = Pose2D(*parsed)' in callback


def test_robot_arrival_requires_current_drive_plan_stamp():
    fleet = source('cooperative_parking_robot/fleet_manager_node.py')
    state = source(
        'cooperative_parking_robot/robot_state_machine_node.py')
    sync_callback = state.split('def sync_cb', 1)[1].split(
        'def transition', 1)[0]

    assert "'plan_stamp_ns': self.active_plan_stamp_ns" in fleet
    assert 'payload.get("plan_stamp_ns", 0)' in state
    assert 'self.state == "DRIVE"' in sync_callback
    assert 'plan_stamp_ns == self.active_plan_stamp_ns' in sync_callback


def test_velocity_command_is_stamped_and_checked_at_bridge():
    individual = source(
        "cooperative_parking_robot/individual_move_node.py")
    sync = source(
        "cooperative_parking_robot/rigid_body_sync_node.py")
    bridge = source(
        "cooperative_parking_robot/stm32_bridge_node.py")

    for text in (individual, sync, bridge):
        assert "TwistStamped" in text
    assert (
        "msg.header.stamp = self.get_clock().now().to_msg()"
        in individual)
    assert (
        "msg.header.stamp = self.get_clock().now().to_msg()"
        in sync)
    assert "self.command_stamp_gate.accept" in bridge
    assert "stamp_to_ns(msg.header.stamp)" in bridge


def test_vehicle_spec_wheelbase_reaches_alignment_and_sync():
    sync = source(
        "cooperative_parking_robot/rigid_body_sync_node.py")
    front_launch = source("launch/front_robot.launch.py")
    rear_launch = source("launch/rear_robot.launch.py")

    assert "'/parking/vehicle_spec', self.vehicle_spec_cb" in sync
    assert "self.kinematics.set_wheelbase(candidate)" in sync
    assert '"use_vehicle_spec_wheelbase": True' in front_launch
    assert '"use_vehicle_spec_wheelbase": True' in rear_launch


def test_three_marker_roles_are_connected_without_id_conflict():
    marker = source(
        "cooperative_parking_robot/cctv_robot_marker_node.py")
    web = source(
        "cooperative_parking_robot/jetson_vision_web_node.py")
    cctv_launch = source("launch/cctv_server.launch.py")
    full_launch = source("launch/full_system.launch.py")

    for text in (marker, web, cctv_launch, full_launch):
        assert "rear_marker_id" in text
    assert "'front': self.get_parameter('front_marker_id').value" in marker
    assert "'rear': self.get_parameter('rear_marker_id').value" in marker
    assert "default_value='10'" in cctv_launch
    assert "default_value='11'" in cctv_launch
    assert "marker_id': 0" in full_launch


def test_drive_marker_fallback_prefers_id0_then_top_pair_then_encoder():
    sync = source(
        "cooperative_parking_robot/rigid_body_sync_node.py")
    assert "ARUCO_DIST_YAW" in sync
    assert "CCTV_ID10_ID11" in sync
    assert "correction = 'ENCODER'" in sync
    assert "MARKER_HOLD" in sync
    marker_stop_block = sync.split(
        "if lost > self.marker_stop:", 1)[1].split(
        "if lost > self.marker_slowdown:", 1)[0]
    assert "self.recoverable_hold(" in marker_stop_block
    assert "self.fatal_stop(" not in marker_stop_block


def test_release_exit_order_is_underbody_then_side_then_home():
    move = source(
        "cooperative_parking_robot/individual_move_node.py")
    return_block = move.split("def run_return", 1)[1]
    underbody = return_block.index('self.phase == "EXIT_UNDERBODY"')
    side = return_block.index('self.phase == "EXIT_TO_SIDE"')
    home = return_block.index('self.phase == "RETURN_HOME"')
    assert underbody < side < home
