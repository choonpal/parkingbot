"""Deterministic, hardware-free contracts across real production components.

These tests intentionally inject only upstream boundary messages.  Assertions
are made on state produced by the real node/core under test; no test publisher
is ever checked against its own output.  They complement (rather than replace)
camera and STM32 HIL checks, which need recorded images or physical hardware.
"""

import json
import math
import time

import pytest
import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped, TwistStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, Float64, String

from cooperative_parking_robot.bev_fusion_core import (
    CameraDetection,
    encode_detection_envelope,
)
from cooperative_parking_robot.cctv_observation import CctvObservation
from cooperative_parking_robot.command_qos import CMD_VEL_QOS
from cooperative_parking_robot.fleet_manager_node import FleetManagerNode
from cooperative_parking_robot.latest_qos import SENSOR_LATEST_QOS, STATE_LATEST_QOS
from cooperative_parking_robot.mvp_integration_nodes import (
    OriginAwareCctvMergeNode,
)
from cooperative_parking_robot.mvp_runtime_nodes import MvpRigidBodySyncNode
from cooperative_parking_robot.pose_fusion_production_node import PoseFusionNode
from cooperative_parking_robot.robot_state_machine_node import RobotStateMachineNode
from cooperative_parking_robot.ultrasonic_edge_node import UltrasonicEdgeNode


NSEC = 1_000_000_000
TEST_NAMESPACE = '/parkingbot_test'

# Complete absolute application-topic surface of the production nodes exercised
# below.  Humble does not implement ROS 2 wildcard remap rules, so each
# contract is listed explicitly instead of relying on an unsafe partial rule.
ISOLATED_PRODUCTION_TOPICS = (
    # PoseFusionNode
    '/front/wheel_odom', '/front/cctv_pose', '/front/cctv_marker_visible',
    '/front/cctv_observation', '/front/odom', '/front/localization_status',
    '/front/cctv_fusion_status',
    # OriginAwareCctvMergeNode
    '/cctv0/detections', '/cctv2/detections', '/front/odom', '/rear/odom',
    '/mission/complete', '/fleet/state', '/robot/lifted', '/parking/map',
    '/parking/target_pose', '/parking/empty_slots', '/parking/vehicle_spec',
    '/parking/vehicle_pose_feedback', '/parking/target_ready',
    '/parking/target_status', '/cctv/merge_status',
    # UltrasonicEdgeNode
    '/rear/odom', '/rear/active_target_pose', '/rear/robot_state',
    '/rear/wheel_scan_reset', '/rear/ultrasonic_left',
    '/rear/ultrasonic_right', '/rear/wheel_detected',
    '/rear/wheel_center_s', '/rear/axle_count', '/rear/wheel_center_x',
    '/rear/motion_fault', '/rear/wheel_lateral_offset',
    '/rear/wheel_lateral_valid',
    # RobotStateMachineNode (front role)
    '/front/wheel_aligned', '/front/lift_status', '/front/hardware_status',
    '/front/hardware_ready', '/sync/error_state', '/front/approach_done',
    '/front/return_done', '/front/motion_fault', '/align/rear_done',
    '/release/rear_done', '/mission/rear/ready', '/mission/commit',
    '/front/robot_state', '/front/grip_command', '/front/lifted',
    '/align/front_done', '/release/front_done', '/mission/front/ready',
    '/emergency_stop', '/rear/lifted',
    # MvpRigidBodySyncNode, including inherited and vision-wrapper inputs.
    '/virtual_robot/waypoints', '/sync/relative_pose', '/sync/marker_visible',
    '/rear/wheel_odom', '/rear/cctv_pose', '/front/cctv_marker_visible',
    '/rear/cctv_marker_visible', '/parking/slot_pose',
    '/rear/cctv_observation', '/front/cmd_vel', '/rear/cmd_vel',
    # FleetManagerNode request inputs and outputs.
    '/ui/mission_request',
)


def _stamp(message, stamp_ns, frame_id='map'):
    message.header.frame_id = frame_id
    message.header.stamp.sec = int(stamp_ns // NSEC)
    message.header.stamp.nanosec = int(stamp_ns % NSEC)
    return message


def _odom(stamp_ns, x, y, yaw=0.0, dx=0.0, dy=0.0, dtheta=0.0):
    message = _stamp(Odometry(), stamp_ns)
    message.pose.pose.position.x = x
    message.pose.pose.position.y = y
    message.pose.pose.orientation.z = math.sin(yaw / 2.0)
    message.pose.pose.orientation.w = math.cos(yaw / 2.0)
    message.twist.twist.linear.x = dx
    message.twist.twist.linear.y = dy
    message.twist.twist.angular.z = dtheta
    return message


def _cctv_observation(camera_id, sequence, stamp_ns, x, y, yaw=0.0):
    return CctvObservation(
        role='front', camera_id=camera_id, stamp_ns=stamp_ns,
        sequence=sequence, switch_sequence=sequence,
        source_changed=sequence > 1, handover_validated=False,
        pose=(x, y, yaw), raw_pose=(x, y, yaw),
        source_bias=(0.0, 0.0, 0.0), selection_cost=0.0,
    )


def _spin_until(executor, predicate, timeout_s=3.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)
        if predicate():
            return True
    return False


def _test_topic(topic):
    """Return the only DDS namespace allowed for hardware-free scenarios."""
    assert topic.startswith('/')
    return f'{TEST_NAMESPACE}{topic}'


def _isolated_ros_args(*parameter_assignments):
    """Globally remap every absolute application topic for this rclpy context.

    All feeder/probe nodes use the original production names below; this ROS
    rules resolve those names -- including inputs a particular scenario does
    not exercise -- beneath ``/parkingbot_test``.  This prevents a test node
    from receiving from or publishing to a live robot on the same DDS domain.
    """
    args = ['--ros-args']
    for topic in sorted(set(ISOLATED_PRODUCTION_TOPICS)):
        args.extend(['--remap', f'{topic}:={_test_topic(topic)}'])
    for assignment in parameter_assignments:
        args.extend(['--param', assignment])
    return [*args, '--']


def _assert_scoped(node, *production_topics):
    """Confirm that a node resolves listed production contracts in test scope."""
    for topic in production_topics:
        assert node.resolve_topic_name(topic) == _test_topic(topic)


def test_pose_fusion_receives_dds_inputs_and_publishes_fused_odom():
    """A feeder/probe crosses actual DDS boundaries around production fusion."""
    owns_context = not rclpy.ok()
    if owns_context:
        rclpy.init(args=_isolated_ros_args())
    fusion = PoseFusionNode()
    feeder = Node('pose_fusion_integration_feeder')
    probe = Node('pose_fusion_integration_probe')
    executor = SingleThreadedExecutor()
    received = []
    try:
        pub_wheel = feeder.create_publisher(
            Odometry, '/front/wheel_odom', SENSOR_LATEST_QOS)
        pub_cctv = feeder.create_publisher(
            String, '/front/cctv_observation', 10)
        probe.create_subscription(
            Odometry, '/front/odom', received.append, SENSOR_LATEST_QOS)
        for node in (fusion, feeder, probe):
            executor.add_node(node)

        _assert_scoped(
            fusion, '/front/wheel_odom', '/front/cctv_observation',
            '/front/cctv_pose', '/front/cctv_marker_visible', '/front/odom',
            '/front/localization_status', '/front/cctv_fusion_status')
        assert pub_wheel.topic_name == _test_topic('/front/wheel_odom')
        assert pub_cctv.topic_name == _test_topic('/front/cctv_observation')
        assert fusion.pub_odom.topic_name == _test_topic('/front/odom')

        assert _spin_until(
            executor,
            lambda: (pub_wheel.get_subscription_count() == 1 and
                     pub_cctv.get_subscription_count() == 1 and
                     fusion.pub_odom.get_subscription_count() == 1),
        ), 'DDS discovery did not complete'

        sequence = 0
        def fused_measurement_received():
            nonlocal sequence
            now_ns = feeder.get_clock().now().nanoseconds
            pub_wheel.publish(_odom(now_ns, 0.0, 0.0))
            sequence += 1
            observation = _cctv_observation(
                'cam0', sequence, now_ns, 0.05, 0.0)
            pub_cctv.publish(String(data=observation.to_json()))
            return any(
                message.header.frame_id == 'map' and
                message.child_frame_id == 'front_base' and
                message.pose.pose.position.x == pytest.approx(0.05, abs=0.02)
                for message in received)

        assert _spin_until(executor, fused_measurement_received)
        assert fusion.ekf.initialized
    finally:
        for node in (probe, feeder, fusion):
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        if owns_context:
            rclpy.shutdown()


def test_mvp_cctv_merge_receives_dual_detection_envelopes_and_publishes_origin_aware_map():
    """The deployed merge node deduplicates cameras into an origin-aware map."""
    owns_context = not rclpy.ok()
    if owns_context:
        rclpy.init(args=_isolated_ros_args(
            'map_origin_x_m:=-0.4', 'map_origin_y_m:=-0.8'))
    merge = OriginAwareCctvMergeNode()
    feeder = Node('cctv_merge_integration_feeder')
    probe = Node('cctv_merge_integration_probe')
    executor = SingleThreadedExecutor()
    maps = []
    statuses = []
    target_ready = []
    target_poses = []
    vehicle_specs = []
    try:
        pub_cam0 = feeder.create_publisher(
            String, '/cctv0/detections', SENSOR_LATEST_QOS)
        pub_cam2 = feeder.create_publisher(
            String, '/cctv2/detections', SENSOR_LATEST_QOS)
        probe.create_subscription(OccupancyGrid, '/parking/map', maps.append, 10)
        probe.create_subscription(String, '/cctv/merge_status', statuses.append, 10)
        probe.create_subscription(Bool, '/parking/target_ready', target_ready.append, 10)
        probe.create_subscription(PoseStamped, '/parking/target_pose',
                                  target_poses.append, 10)
        probe.create_subscription(String, '/parking/vehicle_spec',
                                  vehicle_specs.append, 10)
        for node in (merge, feeder, probe):
            executor.add_node(node)

        _assert_scoped(
            merge, '/cctv0/detections', '/cctv2/detections',
            '/front/odom', '/rear/odom', '/mission/complete',
            '/fleet/state', '/robot/lifted', '/parking/map',
            '/parking/target_pose', '/parking/empty_slots',
            '/parking/vehicle_spec', '/parking/vehicle_pose_feedback',
            '/parking/target_ready', '/parking/target_status',
            '/cctv/merge_status')
        assert pub_cam0.topic_name == _test_topic('/cctv0/detections')
        assert pub_cam2.topic_name == _test_topic('/cctv2/detections')
        assert merge.pub_map.topic_name == _test_topic('/parking/map')
        assert merge.pub_status.topic_name == _test_topic('/cctv/merge_status')

        assert _spin_until(
            executor,
            lambda: (pub_cam0.get_subscription_count() == 1 and
                     pub_cam2.get_subscription_count() == 1 and
                     merge.pub_map.get_subscription_count() == 1 and
                     merge.pub_status.get_subscription_count() == 1 and
                     merge.pub_target_ready.get_subscription_count() == 1 and
                     merge.pub_target.get_subscription_count() == 1 and
                     merge.pub_spec.get_subscription_count() == 1),
        ), 'DDS discovery did not complete'

        coverage = [(0.0, 0.0), (4.4, 0.0), (4.4, 3.83), (0.0, 3.83)]
        sequence = 0
        def merged_map_received():
            nonlocal sequence
            sequence += 1
            stamp_ns = feeder.get_clock().now().nanoseconds
            cam0 = CameraDetection(
                'cam0', (2.30, 0.60),
                polygon=[(1.90, 0.40), (2.70, 0.40),
                         (2.70, 0.80), (1.90, 0.80)],
                yaw=0.0, length_m=0.90, width_m=0.35,
                in_waiting=True, confidence=0.9, axis_dist_m=0.2)
            cam2 = CameraDetection(
                'cam2', (2.33, 0.61),
                polygon=[(1.93, 0.41), (2.73, 0.41),
                         (2.73, 0.81), (1.93, 0.81)],
                yaw=0.0, length_m=0.90, width_m=0.35,
                in_waiting=True, confidence=0.9, axis_dist_m=0.8)
            pub_cam0.publish(String(data=encode_detection_envelope(
                'cam0', stamp_ns, sequence, coverage, [cam0])))
            pub_cam2.publish(String(data=encode_detection_envelope(
                'cam2', stamp_ns, sequence, coverage, [cam2])))
            for status in statuses:
                payload = json.loads(status.data)
                if (payload.get('merged_detections') == 1 and
                        payload.get('duplicates_removed') == 1 and
                        all(payload['cameras'][camera]['alive']
                            for camera in ('cam0', 'cam2'))):
                    return (
                        bool(maps) and any(message.data for message in target_ready) and
                        any(message.header.frame_id == 'map' and
                            message.pose.position.x == pytest.approx(2.30, abs=0.03)
                            for message in target_poses) and
                        any(json.loads(message.data).get('dimension_valid')
                            for message in vehicle_specs))
            return False

        assert _spin_until(executor, merged_map_received, timeout_s=4.0)
        latest_map = maps[-1]
        assert latest_map.header.frame_id == 'map'
        assert latest_map.info.width == merge.grid_w
        assert latest_map.info.height == merge.grid_h
        assert latest_map.info.resolution == pytest.approx(merge.resolution)
        assert latest_map.info.origin.position.x == pytest.approx(-0.4)
        assert latest_map.info.origin.position.y == pytest.approx(-0.8)
        assert 0 in latest_map.data

        ready_before_camera_timeout = len(target_ready)
        assert _spin_until(
            executor,
            lambda: any(not message.data
                        for message in target_ready[ready_before_camera_timeout:]),
            timeout_s=2.5), 'camera timeout did not fail closed'
    finally:
        for node in (probe, feeder, merge):
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        if owns_context:
            rclpy.shutdown()


def _target_pose(stamp_ns, x=0.0, y=0.0, yaw=0.0):
    message = _stamp(PoseStamped(), stamp_ns)
    message.pose.position.x = x
    message.pose.position.y = y
    message.pose.orientation.z = math.sin(yaw / 2.0)
    message.pose.orientation.w = math.cos(yaw / 2.0)
    return message


def _range(stamp_ns, value):
    message = _stamp(Range(), stamp_ns)
    message.range = float(value)
    return message


def test_ultrasonic_edge_dds_inputs_produce_rear_axle_target():
    """Fresh ALIGN target/odom/Range streams produce a real wheel target."""
    owns_context = not rclpy.ok()
    if owns_context:
        rclpy.init(args=_isolated_ros_args())
    edge = UltrasonicEdgeNode(parameter_overrides=[
        Parameter('role', value='rear'),
        Parameter('window_size', value=1),
        Parameter('lateral_median_n', value=1),
    ])
    feeder = Node('ultrasonic_edge_integration_feeder')
    probe = Node('ultrasonic_edge_integration_probe')
    executor = SingleThreadedExecutor()
    detected = []
    centers = []
    try:
        pub_state = feeder.create_publisher(
            String, '/rear/robot_state', STATE_LATEST_QOS)
        pub_target = feeder.create_publisher(
            PoseStamped, '/rear/active_target_pose', 10)
        pub_odom = feeder.create_publisher(
            Odometry, '/rear/odom', SENSOR_LATEST_QOS)
        pub_left = feeder.create_publisher(
            Range, '/rear/ultrasonic_left', SENSOR_LATEST_QOS)
        pub_right = feeder.create_publisher(
            Range, '/rear/ultrasonic_right', SENSOR_LATEST_QOS)
        probe.create_subscription(Bool, '/rear/wheel_detected', detected.append, 10)
        probe.create_subscription(Float64, '/rear/wheel_center_s', centers.append, 10)
        for node in (edge, feeder, probe):
            executor.add_node(node)

        _assert_scoped(
            edge, '/rear/odom', '/rear/active_target_pose',
            '/rear/robot_state', '/rear/wheel_scan_reset',
            '/parking/vehicle_spec', '/rear/ultrasonic_left',
            '/rear/ultrasonic_right', '/rear/wheel_detected',
            '/rear/wheel_center_s', '/rear/axle_count',
            '/rear/wheel_center_x', '/rear/motion_fault',
            '/rear/wheel_lateral_offset', '/rear/wheel_lateral_valid')
        assert pub_odom.topic_name == _test_topic('/rear/odom')
        assert edge.pub_detected.topic_name == _test_topic('/rear/wheel_detected')

        assert _spin_until(
            executor,
            lambda: (pub_state.get_subscription_count() == 1 and
                     pub_target.get_subscription_count() == 1 and
                     pub_odom.get_subscription_count() == 1 and
                     pub_left.get_subscription_count() == 1 and
                     pub_right.get_subscription_count() == 1 and
                     edge.pub_detected.get_subscription_count() == 1 and
                     edge.pub_center_s.get_subscription_count() == 1),
        ), 'DDS discovery did not complete'

        phases = [
            ('state_target', 0.0, None),
            ('odom', -0.45, None),
            ('range', -0.45, 0.05),
            ('odom', -0.35, None),
            ('range', -0.35, 0.20),
        ]
        phase = 0
        phase_cycles = 0
        def wheel_target_received():
            nonlocal phase, phase_cycles
            if phase < len(phases) and phase_cycles == 0:
                kind, position_s, distance = phases[phase]
                stamp_ns = feeder.get_clock().now().nanoseconds
                if kind == 'state_target':
                    pub_state.publish(String(data='ALIGN'))
                    pub_target.publish(_target_pose(stamp_ns))
                elif kind == 'odom':
                    pub_odom.publish(_odom(stamp_ns, position_s, 0.0))
                else:
                    pub_left.publish(_range(stamp_ns, distance))
                    pub_right.publish(_range(stamp_ns, distance))
            phase_cycles += 1
            if phase_cycles >= 4:
                phase += 1
                phase_cycles = 0
            return (any(message.data for message in detected) and centers and
                    centers[-1].data == pytest.approx(-0.40, abs=0.03))

        assert _spin_until(executor, wheel_target_received)
        assert edge.published
    finally:
        for node in (probe, feeder, edge):
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        if owns_context:
            rclpy.shutdown()


def test_fleet_dds_inputs_produce_correlated_registered_slot_plan(tmp_path):
    """DDS feeds Fleet's real planner and probes its latched mission outputs.

    The timer/state-machine route to ``plan_and_publish`` is deliberately not
    exercised here: it adds a 0.5 s scheduling dependency unrelated to the
    planning contract.  All plan prerequisites enter through real DDS
    subscriptions; the production planning method is then called directly.
    """
    if rclpy.ok():
        pytest.skip('requires an isolated rclpy context for temporary registry DB')
    registry_db = tmp_path / 'fleet_registry.sqlite3'
    rclpy.init(args=_isolated_ros_args(
        f'parking_registry_db_path:={registry_db}',
        'slot_ids:=[A1]',
        'slot_coords:=[6.0,5.0]',
        'slot_sizes:=[3.0,1.2]',
        'slot_yaws_deg:=[0.0]',
        'use_staged_slot_entry:=false',
        'unknown_is_occupied:=false',
        'odom_timeout_s:=3.0'))
    fleet = FleetManagerNode()
    feeder = Node('fleet_integration_feeder')
    probe = Node('fleet_integration_probe')
    executor = SingleThreadedExecutor()
    paths = []
    slot_poses = []
    try:
        _assert_scoped(
            fleet, '/parking/target_pose', '/parking/empty_slots',
            '/parking/map', '/robot/lifted', '/parking/vehicle_spec',
            '/ui/mission_request', '/mission/complete', '/mission/commit',
            '/sync/error_state', '/front/odom', '/rear/odom',
            '/front/robot_state', '/rear/robot_state',
            '/front/motion_fault', '/rear/motion_fault',
            '/virtual_robot/waypoints', '/fleet/state', '/parking/slot_pose')
        pub_target = feeder.create_publisher(PoseStamped, '/parking/target_pose', 10)
        pub_slots = feeder.create_publisher(PoseArray, '/parking/empty_slots', 10)
        pub_map = feeder.create_publisher(OccupancyGrid, '/parking/map', 10)
        pub_spec = feeder.create_publisher(
            String, '/parking/vehicle_spec', fleet.mission_qos)
        pub_front_odom = feeder.create_publisher(
            Odometry, '/front/odom', SENSOR_LATEST_QOS)
        pub_rear_odom = feeder.create_publisher(
            Odometry, '/rear/odom', SENSOR_LATEST_QOS)
        probe.create_subscription(Path, '/virtual_robot/waypoints', paths.append,
                                  fleet.mission_qos)
        probe.create_subscription(PoseStamped, '/parking/slot_pose',
                                  slot_poses.append, fleet.mission_qos)
        for node in (fleet, feeder, probe):
            executor.add_node(node)

        assert pub_target.topic_name == _test_topic('/parking/target_pose')
        assert pub_slots.topic_name == _test_topic('/parking/empty_slots')
        assert pub_map.topic_name == _test_topic('/parking/map')
        assert pub_spec.topic_name == _test_topic('/parking/vehicle_spec')
        assert pub_front_odom.topic_name == _test_topic('/front/odom')
        assert pub_rear_odom.topic_name == _test_topic('/rear/odom')
        assert fleet.pub_waypoints.topic_name == _test_topic('/virtual_robot/waypoints')
        assert fleet.pub_slot_pose.topic_name == _test_topic('/parking/slot_pose')
        assert _spin_until(
            executor,
            lambda: (
                pub_target.get_subscription_count() == 1 and
                pub_slots.get_subscription_count() == 1 and
                pub_map.get_subscription_count() == 1 and
                pub_spec.get_subscription_count() == 1 and
                pub_front_odom.get_subscription_count() == 1 and
                pub_rear_odom.get_subscription_count() == 1 and
                fleet.pub_waypoints.get_subscription_count() == 1 and
                fleet.pub_slot_pose.get_subscription_count() == 1),
        ), 'DDS discovery did not complete'

        sequence = 0
        def plan_inputs_delivered():
            nonlocal sequence
            sequence += 1
            stamp_ns = feeder.get_clock().now().nanoseconds
            wheelbase = fleet.current_wheelbase
            grid = _stamp(OccupancyGrid(), stamp_ns)
            grid.info.resolution = 0.10
            grid.info.width = 100
            grid.info.height = 100
            grid.data = [0] * (grid.info.width * grid.info.height)
            slots = _stamp(PoseArray(), stamp_ns)
            slots.poses.append(_target_pose(stamp_ns, 6.0, 5.0).pose)
            pub_map.publish(grid)
            pub_slots.publish(slots)
            pub_target.publish(_target_pose(stamp_ns, 2.0, 5.0))
            pub_spec.publish(String(data=json.dumps({
                'stamp_ns': stamp_ns, 'wheelbase': wheelbase,
                'vehicle_length_m': 0.90, 'vehicle_width_m': 0.35,
                'dimension_valid': True, 'sequence': sequence,
            })))
            pub_front_odom.publish(_odom(stamp_ns, 2.0 + wheelbase / 2.0, 5.0))
            pub_rear_odom.publish(_odom(stamp_ns, 2.0 - wheelbase / 2.0, 5.0))
            return (fleet.grid is not None and fleet.empty_slots and
                    fleet.target_pose is not None and
                    fleet.active_vehicle_spec is not None and
                    fleet.current_virtual_start() is not None)

        assert _spin_until(executor, plan_inputs_delivered)
        fleet.mission_type = 'park'
        fleet.mission_id = 'fleet-dds-plan-1'
        fleet.requested_destination_slot_id = 'A1'
        assert fleet.plan_and_publish()
        assert _spin_until(executor, lambda: bool(paths) and bool(slot_poses))

        path = paths[-1]
        destination = slot_poses[-1]
        assert path.header.frame_id == destination.header.frame_id == 'map'
        assert path.poses
        assert path.poses[-1].header.frame_id == 'map'
        assert path.poses[-1].pose.position.x == pytest.approx(6.0, abs=0.11)
        assert destination.pose.position.x == pytest.approx(6.0, abs=0.01)
        assert destination.pose.position.y == pytest.approx(5.0, abs=0.01)
        assert (path.header.stamp.sec, path.header.stamp.nanosec) == (
            destination.header.stamp.sec, destination.header.stamp.nanosec)
        assert fleet.active_plan_stamp_ns > 0
        assert fleet.active_destination_slot_id == 'A1'
        assert fleet.registry.lifecycle('A1').name == 'RESERVED'
    finally:
        for node in (probe, feeder, fleet):
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


def test_state_machine_dds_lift_barrier_uses_remapped_actuation_topics():
    """DDS peer readiness is required before the Front node commits LIFT."""
    if rclpy.ok():
        pytest.skip('requires an isolated rclpy context for actuation remaps')
    rclpy.init(args=_isolated_ros_args())
    front = RobotStateMachineNode(parameter_overrides=[
        Parameter('role', value='front'),
        Parameter('require_hardware_ready', value=False),
    ])
    feeder = Node('state_machine_integration_feeder')
    probe = Node('state_machine_integration_probe')
    executor = SingleThreadedExecutor()
    commits = []
    try:
        _assert_scoped(
            front, '/front/wheel_aligned', '/fleet/state',
            '/front/lift_status', '/front/hardware_status',
            '/front/hardware_ready', '/sync/error_state',
            '/front/approach_done', '/front/return_done',
            '/front/motion_fault', '/align/rear_done', '/release/rear_done',
            '/mission/rear/ready', '/mission/commit', '/front/robot_state',
            '/front/grip_command', '/front/lifted', '/align/front_done',
            '/release/front_done', '/mission/front/ready',
            '/emergency_stop', '/mission/complete', '/robot/lifted',
            '/rear/lifted')
        assert front.pub_grip.topic_name == _test_topic('/front/grip_command')
        assert front.pub_estop.topic_name == _test_topic('/emergency_stop')

        coordination_qos = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        pub_fleet = feeder.create_publisher(
            String, '/fleet/state', STATE_LATEST_QOS)
        pub_approach = feeder.create_publisher(
            Bool, '/front/approach_done', 10)
        pub_aligned = feeder.create_publisher(
            Bool, '/front/wheel_aligned', 10)
        pub_rear_ready = feeder.create_publisher(
            String, '/mission/rear/ready', coordination_qos)
        probe.create_subscription(
            String, '/mission/commit', commits.append, coordination_qos)
        for node in (front, feeder, probe):
            executor.add_node(node)

        assert pub_fleet.topic_name == _test_topic('/fleet/state')
        assert pub_approach.topic_name == _test_topic('/front/approach_done')
        assert pub_aligned.topic_name == _test_topic('/front/wheel_aligned')
        assert pub_rear_ready.topic_name == _test_topic('/mission/rear/ready')
        assert front.pub_commit.topic_name == _test_topic('/mission/commit')

        assert _spin_until(
            executor,
            lambda: (pub_fleet.get_subscription_count() == 1 and
                     pub_approach.get_subscription_count() == 1 and
                     pub_aligned.get_subscription_count() == 1 and
                     pub_rear_ready.get_subscription_count() == 1 and
                     front.pub_commit.get_subscription_count() == 2),
        ), 'DDS discovery did not complete'

        sequence = 0
        mission_id = 'mission-dds-barrier-1'
        def lift_committed():
            nonlocal sequence
            now_ns = feeder.get_clock().now().nanoseconds
            sequence += 1
            pub_fleet.publish(String(data=json.dumps({
                'state': 'WAIT_LIFT', 'mission_id': mission_id,
                'plan_stamp_ns': 0, 'sequence': sequence,
                'stamp_ns': now_ns,
            })))
            pub_approach.publish(Bool(data=True))
            pub_aligned.publish(Bool(data=True))
            pub_rear_ready.publish(String(data=json.dumps({
                'mission_id': mission_id, 'role': 'rear', 'stage': 'LIFT',
                'sequence': sequence, 'stamp_ns': now_ns,
            })))
            return any(json.loads(message.data).get('stage') == 'LIFT'
                       for message in commits)

        assert _spin_until(executor, lift_committed)
        assert front.state == 'LIFT'
        assert any(
            json.loads(message.data)['mission_id'] == mission_id
            for message in commits)
    finally:
        for node in (probe, feeder, front):
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


def test_mvp_rigid_sync_dds_publishes_only_remapped_paired_commands():
    """Fresh mission/pose/ID0 streams drive the deployed P0/vision wrapper."""
    if rclpy.ok():
        pytest.skip('requires an isolated rclpy context for actuation remaps')
    rclpy.init(args=_isolated_ros_args(
        'sync_reference_capture_samples:=3',
        'sync_reference_settle_time_s:=0.0'))
    sync = MvpRigidBodySyncNode()
    feeder = Node('rigid_sync_integration_feeder')
    probe = Node('rigid_sync_integration_probe')
    executor = SingleThreadedExecutor()
    front_commands = []
    rear_commands = []
    try:
        _assert_scoped(
            sync, '/virtual_robot/waypoints', '/parking/slot_pose',
            '/parking/target_pose', '/front/odom', '/rear/odom',
            '/front/wheel_odom', '/rear/wheel_odom', '/sync/relative_pose',
            '/sync/marker_visible', '/robot/lifted', '/front/robot_state',
            '/rear/robot_state', '/parking/vehicle_pose_feedback',
            '/parking/vehicle_spec', '/front/cctv_pose', '/rear/cctv_pose',
            '/front/cctv_marker_visible', '/rear/cctv_marker_visible',
            '/front/cctv_observation', '/rear/cctv_observation',
            '/front/cmd_vel', '/rear/cmd_vel', '/sync/error_state',
            '/emergency_stop')
        assert sync.pub_fc.topic_name == _test_topic('/front/cmd_vel')
        assert sync.pub_rc.topic_name == _test_topic('/rear/cmd_vel')
        assert sync.pub_estop.topic_name == _test_topic('/emergency_stop')

        mission_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        pub_path = feeder.create_publisher(
            Path, '/virtual_robot/waypoints', mission_qos)
        pub_slot = feeder.create_publisher(
            PoseStamped, '/parking/slot_pose', mission_qos)
        pub_target = feeder.create_publisher(
            PoseStamped, '/parking/target_pose', 10)
        pub_front_odom = feeder.create_publisher(
            Odometry, '/front/odom', SENSOR_LATEST_QOS)
        pub_rear_odom = feeder.create_publisher(
            Odometry, '/rear/odom', SENSOR_LATEST_QOS)
        pub_front_wheel = feeder.create_publisher(
            Odometry, '/front/wheel_odom', SENSOR_LATEST_QOS)
        pub_rear_wheel = feeder.create_publisher(
            Odometry, '/rear/wheel_odom', SENSOR_LATEST_QOS)
        pub_relative = feeder.create_publisher(
            PoseStamped, '/sync/relative_pose', SENSOR_LATEST_QOS)
        pub_marker = feeder.create_publisher(
            Bool, '/sync/marker_visible', SENSOR_LATEST_QOS)
        pub_lifted = feeder.create_publisher(Bool, '/robot/lifted', 10)
        pub_front_state = feeder.create_publisher(
            String, '/front/robot_state', STATE_LATEST_QOS)
        pub_rear_state = feeder.create_publisher(
            String, '/rear/robot_state', STATE_LATEST_QOS)
        probe.create_subscription(
            TwistStamped, '/front/cmd_vel',
            front_commands.append, CMD_VEL_QOS)
        probe.create_subscription(
            TwistStamped, '/rear/cmd_vel',
            rear_commands.append, CMD_VEL_QOS)
        for node in (sync, feeder, probe):
            executor.add_node(node)

        assert pub_path.topic_name == _test_topic('/virtual_robot/waypoints')
        assert pub_slot.topic_name == _test_topic('/parking/slot_pose')
        assert pub_target.topic_name == _test_topic('/parking/target_pose')
        assert pub_front_odom.topic_name == _test_topic('/front/odom')
        assert pub_rear_odom.topic_name == _test_topic('/rear/odom')
        assert pub_front_wheel.topic_name == _test_topic('/front/wheel_odom')
        assert pub_rear_wheel.topic_name == _test_topic('/rear/wheel_odom')
        assert pub_relative.topic_name == _test_topic('/sync/relative_pose')
        assert pub_marker.topic_name == _test_topic('/sync/marker_visible')
        assert pub_lifted.topic_name == _test_topic('/robot/lifted')
        assert pub_front_state.topic_name == _test_topic('/front/robot_state')
        assert pub_rear_state.topic_name == _test_topic('/rear/robot_state')

        assert _spin_until(
            executor,
            lambda: (
                pub_path.get_subscription_count() == 1 and
                pub_slot.get_subscription_count() == 1 and
                pub_target.get_subscription_count() == 1 and
                pub_front_odom.get_subscription_count() == 1 and
                pub_rear_odom.get_subscription_count() == 1 and
                pub_front_wheel.get_subscription_count() == 1 and
                pub_rear_wheel.get_subscription_count() == 1 and
                pub_relative.get_subscription_count() == 1 and
                pub_marker.get_subscription_count() == 1 and
                pub_lifted.get_subscription_count() == 1 and
                pub_front_state.get_subscription_count() == 1 and
                pub_rear_state.get_subscription_count() == 1 and
                sync.pub_fc.get_subscription_count() == 1 and
                sync.pub_rc.get_subscription_count() == 1),
        ), 'DDS discovery did not complete'

        mission_sent = False
        relative_sequence = 0
        def paired_command_received():
            nonlocal mission_sent, relative_sequence
            stamp_ns = feeder.get_clock().now().nanoseconds
            if not mission_sent:
                path = _stamp(Path(), stamp_ns)
                path.poses.extend([
                    _target_pose(stamp_ns, 0.3925, 0.0),
                    _target_pose(stamp_ns, 1.20, 0.0),
                ])
                slot = _target_pose(stamp_ns, 1.20, 0.0)
                pub_slot.publish(slot)
                pub_path.publish(path)
                pub_target.publish(_target_pose(stamp_ns, 0.3925, 0.0))
                mission_sent = True

            pub_front_state.publish(String(data='DRIVE'))
            pub_rear_state.publish(String(data='DRIVE'))
            pub_lifted.publish(Bool(data=True))
            pub_marker.publish(Bool(data=True))
            pub_front_odom.publish(_odom(stamp_ns, 0.785, 0.0))
            pub_rear_odom.publish(_odom(stamp_ns, 0.0, 0.0))
            pub_front_wheel.publish(_odom(stamp_ns, 0.785, 0.0))
            pub_rear_wheel.publish(_odom(stamp_ns, 0.0, 0.0))
            relative_sequence += 1
            relative = _target_pose(stamp_ns, 0.215, 0.0)
            relative.header.frame_id = 'rear_base'
            pub_relative.publish(relative)

            return (any(command.twist.linear.x > 1e-4
                        for command in front_commands) and
                    any(command.twist.linear.x > 1e-4
                        for command in rear_commands))

        assert _spin_until(executor, paired_command_received, timeout_s=4.0)
        front = next(command for command in reversed(front_commands)
                     if command.twist.linear.x > 1e-4)
        rear = next(command for command in reversed(rear_commands)
                    if command.twist.linear.x > 1e-4)
        assert front.header.frame_id == 'front_base'
        assert rear.header.frame_id == 'rear_base'
        assert front.twist.linear.x > 0.0
        assert rear.twist.linear.x > 0.0
        assert front.twist.linear.x == pytest.approx(rear.twist.linear.x)
        assert front.twist.linear.y == pytest.approx(rear.twist.linear.y)
        assert front.twist.angular.z == pytest.approx(rear.twist.angular.z)
    finally:
        for node in (probe, feeder, sync):
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
