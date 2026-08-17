"""Mission barrier, freshness, and post-release exit regression tests."""

from pathlib import Path

from cooperative_parking_robot.freshness import NSEC_PER_SEC, StampGate


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


def test_state_machine_uses_mission_scoped_two_phase_barriers():
    state = source(
        "cooperative_parking_robot/robot_state_machine_node.py")
    assert '"mission_id"' in state
    assert '"sequence"' in state
    assert '"stamp_ns"' in state
    assert 'f"/mission/{self.role}/ready"' in state
    assert '"/mission/commit"' in state
    assert "DurabilityPolicy.TRANSIENT_LOCAL" in state

    for stage in ("LIFT", "DRIVE", "RELEASE", "RETURN"):
        assert f'self.publish_ready_stage("{stage}")' in state
        assert f'self.maybe_publish_commit("{stage}")' in state
        assert f'"{stage}" in self.committed_stages' in state

    assert 'self.transition("WAIT_RELEASE")' in state
    assert "FLEET_STATE_TIMEOUT" in state
    assert "MISSION_CHANGED_WHILE_ACTIVE" in state


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
    assert "MARKER_LOST" in sync


def test_release_exit_order_is_underbody_then_side_then_home():
    move = source(
        "cooperative_parking_robot/individual_move_node.py")
    return_block = move.split("def run_return", 1)[1]
    underbody = return_block.index('self.phase == "EXIT_UNDERBODY"')
    side = return_block.index('self.phase == "EXIT_TO_SIDE"')
    home = return_block.index('self.phase == "RETURN_HOME"')
    assert underbody < side < home
