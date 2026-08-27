"""Latest-value QoS and stale-pose replay regressions."""

from pathlib import Path

from rclpy.qos import DurabilityPolicy, ReliabilityPolicy

from cooperative_parking_robot.latest_qos import (
    SAFETY_STATE_QOS,
    SENSOR_LATEST_QOS,
    STATE_LATEST_QOS,
)


PACKAGE = Path(__file__).parents[1] / 'cooperative_parking_robot'


def test_latest_value_profiles_are_depth_one():
    assert SENSOR_LATEST_QOS.depth == 1
    assert SENSOR_LATEST_QOS.reliability == ReliabilityPolicy.BEST_EFFORT
    assert STATE_LATEST_QOS.depth == 1
    assert STATE_LATEST_QOS.reliability == ReliabilityPolicy.RELIABLE
    assert SAFETY_STATE_QOS.depth == 1
    assert SAFETY_STATE_QOS.durability == DurabilityPolicy.TRANSIENT_LOCAL


def test_high_rate_pose_consumers_use_latest_value_contract():
    expected = (
        'stm32_bridge_node.py',
        'pose_fusion_node.py',
        'fleet_manager_node.py',
        'rigid_body_sync_node.py',
        'rigid_body_sync_safe_node.py',
        'individual_move_node.py',
        'ultrasonic_edge_node.py',
        'cctv_merge_node.py',
        'yolo_bev_map_node.py',
    )
    for name in expected:
        source = (PACKAGE / name).read_text(encoding='utf-8')
        assert 'SENSOR_LATEST_QOS' in source, name
    pose = (PACKAGE / 'pose_fusion_node.py').read_text(encoding='utf-8')
    assert 'depth=20' not in pose
    assert "f'/{self.role}/odom', SENSOR_LATEST_QOS" in pose


def test_duplicate_wheel_stamp_does_not_republish_stale_odom():
    for name in ('pose_fusion_node.py', 'pose_fusion_production_node.py'):
        source = (PACKAGE / name).read_text(encoding='utf-8')
        duplicate_branch = source.split('if raw_dt <= 0.0:', 1)[1].split(
            'dt = min', 1)[0]
        assert 'publish_odom' not in duplicate_branch, name
        assert 'return' in duplicate_branch, name


def test_fleet_and_safety_state_are_latest_and_retained_as_required():
    fleet = (PACKAGE / 'fleet_manager_node.py').read_text(encoding='utf-8')
    state = (PACKAGE / 'robot_state_machine_node.py').read_text(
        encoding='utf-8')
    bridge = (PACKAGE / 'stm32_bridge_node.py').read_text(encoding='utf-8')
    assert "'/fleet/state', STATE_LATEST_QOS" in fleet
    assert 'hardware_status", self.hardware_cb,\n            SAFETY_STATE_QOS' in state
    assert "f'/{self.role}/hardware_ready', SAFETY_STATE_QOS" in bridge
