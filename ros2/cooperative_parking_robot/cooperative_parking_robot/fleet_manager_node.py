#!/usr/bin/env python3
"""
==================================================
fleet_manager_node.py (Jetson Orin Nano)
==================================================
중앙 관제탑. 빈자리 선정 + A* 경로계획 + waypoint 발행.
자체 A*를 사용한다.

입력:
  /parking/target_pose, /parking/empty_slots
  /parking/map (OccupancyGrid)
  /robot/lifted
출력:
  /virtual_robot/waypoints (nav_msgs/Path)
  /fleet/state
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from std_msgs.msg import Bool, String
import math
import json
import time

from cooperative_parking_robot.astar_planner import AStarPlanner
from cooperative_parking_robot.bev_fusion_core import point_in_polygon
from cooperative_parking_robot.freshness import (
    RequestReplayGuard,
    StampGate,
    stamp_to_ns,
)
from cooperative_parking_robot.loaded_footprint import (
    compute_loaded_footprint,
)
from cooperative_parking_robot.latest_qos import (
    SENSOR_LATEST_QOS,
    STATE_LATEST_QOS,
)
from cooperative_parking_robot.vehicle_entry import (
    MIN_INTER_ROBOT_GAP_M,
    approach_longitudinal,
    validate_wheelbase_clearance,
    vehicle_to_world,
)
from cooperative_parking_robot.mission_protocol import parse_arrival_status
from cooperative_parking_robot.sync_faults import is_fatal_sync_error
from cooperative_parking_robot.parking_registry import (
    ParkingCredential,
    ParkingRegistry,
    RegistryTransitionError,
    SlotLifecycle,
    normalize_vehicle_number,
    registered_slots_fingerprint,
)
from cooperative_parking_robot.retrieval_planning import (
    clear_source_vehicle,
    corridor_is_free,
    make_extraction_geometry,
    make_waiting_staging,
    sequential_routes_clear,
    simultaneous_routes_clear,
)
from cooperative_parking_robot.parking_geometry import (
    Pose2D,
    check_slot_fit,
    footprint_extents_in_slot_axes,
    make_approach_candidates,
    parse_registered_slots,
)


PLANNING_VALIDATION_MODES = ('enforce', 'warn_only')


def normalize_planning_validation_mode(value):
    """Normalize the Fleet planning-validation policy parameter."""
    mode = str(value).strip().lower()
    if mode not in PLANNING_VALIDATION_MODES:
        raise ValueError(
            'planning_validation_mode must be enforce or warn_only')
    return mode


class FleetManagerNode(Node):
    def __init__(self):
        super().__init__('fleet_manager_node')

        self.declare_parameter('waiting_x', 2.3)
        self.declare_parameter('waiting_y', 0.6)
        self.declare_parameter('waiting_yaw_deg', 0.0)
        self.declare_parameter(
            'waiting_polygon',
            [2.10, 0.30, 2.50, 0.30, 2.50, 0.90, 2.10, 0.90])
        self.declare_parameter('map_resolution', 0.05)
        self.declare_parameter('target_timeout_s', 2.0)
        self.declare_parameter('target_candidate_timeout_s', 2.0)
        self.declare_parameter('vehicle_spec_timeout_s', 10.0)
        self.declare_parameter('require_valid_vehicle_spec', False)
        self.declare_parameter('odom_timeout_s', 0.5)
        self.declare_parameter('future_tolerance_s', 0.10)
        # 실측된 로봇 외곽. 길이(+x)는 차량 앞뒤, 폭(+y)은 차량 좌우.
        self.declare_parameter('robot_length_m', 0.565)
        self.declare_parameter('robot_width_m', 0.420)
        self.declare_parameter(
            'minimum_inter_robot_gap_m', MIN_INTER_ROBOT_GAP_M)
        self.declare_parameter('default_wheelbase_m', 0.785)
        # 차량 외곽은 아직 실측 전 placeholder이며 config에서 교체한다.
        self.declare_parameter('default_vehicle_length_m', 0.90)
        self.declare_parameter('default_vehicle_width_m', 0.35)
        self.declare_parameter('source_vehicle_fallback_mask_m', 0.90)
        self.declare_parameter('footprint_safety_margin_m', 0.06)
        self.declare_parameter('unknown_is_occupied', True)
        self.declare_parameter('planning_validation_mode', 'enforce')
        self.declare_parameter('layout_registered', False)
        self.declare_parameter('require_registered_layout', False)
        # 브라우저 등록 결과. 같은 index가 하나의 슬롯을 나타낸다.
        self.declare_parameter('slot_ids', ['P1', 'P2', 'P3', 'P4'])
        self.declare_parameter(
            'slot_coords',
            [1.5, 3.5, 2.5, 3.5, 3.5, 3.5, 4.5, 3.5])
        self.declare_parameter(
            'slot_sizes',
            [1.80, 0.70, 1.80, 0.70, 1.80, 0.70, 1.80, 0.70])
        self.declare_parameter('slot_yaws_deg', [90.0, 90.0, 90.0, 90.0])
        self.declare_parameter('slot_match_tolerance_m', 0.10)
        # 최종 주차는 슬롯 앞 정렬점까지 평행이동한 뒤 회전하고 직선 삽입한다.
        self.declare_parameter('use_staged_slot_entry', True)
        self.declare_parameter('parking_direction', 'forward')
        # staging 전환 오차(2cm) + 지역화/외곽 오차를 흡수할 양쪽 여유.
        self.declare_parameter('slot_fit_longitudinal_margin_m', 0.06)
        self.declare_parameter('slot_fit_lateral_margin_m', 0.06)
        self.declare_parameter('slot_staging_gap_m', 0.10)
        self.declare_parameter('rigid_body_lookahead_m', 0.15)
        self.declare_parameter('entry_standoff_m', 0.85)
        self.declare_parameter('approach_speed_mps', 0.035)
        self.declare_parameter('approach_yaw_gain', 1.5)
        self.declare_parameter('approach_max_yaw_rate_rps', 0.15)
        self.declare_parameter('simultaneous_entry', False)
        # 인양 직후에는 차량이 아직 대기위치에 있다. CCTV 차량 중심과
        # 로봇 두 대 중점의 고정 offset을 계획 시작점에도 반영해,
        # Fleet A* 좌표계와 rigid_body_sync의 차량 중심 좌표계를 맞춘다.
        self.declare_parameter('initial_target_offset_gate_m', 0.50)
        # P2: 터치 UI 승인 게이트. false면 v1.9와 동일하게 target 인식 즉시 시작한다.
        self.declare_parameter('require_ui_confirmation', True)
        self.declare_parameter('ui_request_timeout_s', 10.0)
        self.declare_parameter(
            'parking_registry_db_path',
            '~/.ros/adaptive_valet_bot/parking_registry.db')
        self.wait_x = self.get_parameter('waiting_x').value
        self.wait_y = self.get_parameter('waiting_y').value
        self.wait_yaw = math.radians(float(
            self.get_parameter('waiting_yaw_deg').value))
        waiting_flat = list(self.get_parameter('waiting_polygon').value)
        if len(waiting_flat) < 6 or len(waiting_flat) % 2:
            raise ValueError('waiting_polygon must contain x,y pairs')
        self.waiting_polygon = list(zip(
            [float(value) for value in waiting_flat[0::2]],
            [float(value) for value in waiting_flat[1::2]]))
        if not point_in_polygon(
                self.wait_x, self.wait_y, self.waiting_polygon):
            raise ValueError(
                'waiting_x/y must lie inside vehicle-center detection ROI')
        self.resolution = float(self.get_parameter('map_resolution').value)
        self.odom_timeout = float(
            self.get_parameter('odom_timeout_s').value)
        self.future_tolerance = float(
            self.get_parameter('future_tolerance_s').value)
        self.target_candidate_timeout = float(
            self.get_parameter('target_candidate_timeout_s').value)
        if (self.odom_timeout <= 0.0 or self.future_tolerance < 0.0 or
                self.target_candidate_timeout <= 0.0):
            raise ValueError('invalid odom/future timeout')

        self.robot_length = float(
            self.get_parameter('robot_length_m').value)
        self.robot_width = float(
            self.get_parameter('robot_width_m').value)
        self.minimum_inter_robot_gap = float(
            self.get_parameter('minimum_inter_robot_gap_m').value)
        self.current_wheelbase = float(
            self.get_parameter('default_wheelbase_m').value)
        self.vehicle_length = float(
            self.get_parameter('default_vehicle_length_m').value)
        self.vehicle_width = float(
            self.get_parameter('default_vehicle_width_m').value)
        self.source_vehicle_fallback_mask = float(
            self.get_parameter('source_vehicle_fallback_mask_m').value)
        self.footprint_margin = float(
            self.get_parameter('footprint_safety_margin_m').value)
        self.vehicle_center_offset_body = [0.0, 0.0]
        self.unknown_is_occupied = bool(
            self.get_parameter('unknown_is_occupied').value)
        self.planning_validation_mode = normalize_planning_validation_mode(
            self.get_parameter('planning_validation_mode').value)
        if (bool(self.get_parameter('require_registered_layout').value) and
                not bool(self.get_parameter('layout_registered').value)):
            raise RuntimeError(
                '현장 등록 layout이 아닙니다; '
                'BEV 등록 도구로 생성한 YAML을 사용하세요')
        self.registered_slots = parse_registered_slots(
            self.get_parameter('slot_ids').value,
            self.get_parameter('slot_coords').value,
            self.get_parameter('slot_sizes').value,
            self.get_parameter('slot_yaws_deg').value)
        self.registry_database_path = str(
            self.get_parameter('parking_registry_db_path').value).strip()
        if not self.registry_database_path:
            raise ValueError('parking_registry_db_path must not be empty')
        self.slot_match_tolerance = float(
            self.get_parameter('slot_match_tolerance_m').value)
        self.use_staged_slot_entry = bool(
            self.get_parameter('use_staged_slot_entry').value)
        self.parking_direction = str(
            self.get_parameter('parking_direction').value).strip().lower()
        self.slot_fit_long_margin = float(
            self.get_parameter('slot_fit_longitudinal_margin_m').value)
        self.slot_fit_lat_margin = float(
            self.get_parameter('slot_fit_lateral_margin_m').value)
        self.slot_staging_gap = float(
            self.get_parameter('slot_staging_gap_m').value)
        self.rigid_body_lookahead = float(
            self.get_parameter('rigid_body_lookahead_m').value)
        self.entry_standoff = float(
            self.get_parameter('entry_standoff_m').value)
        self.approach_speed = float(
            self.get_parameter('approach_speed_mps').value)
        self.approach_yaw_gain = float(
            self.get_parameter('approach_yaw_gain').value)
        self.approach_max_yaw_rate = float(
            self.get_parameter('approach_max_yaw_rate_rps').value)
        self.simultaneous_entry = bool(
            self.get_parameter('simultaneous_entry').value)
        self.initial_target_offset_gate = float(
            self.get_parameter('initial_target_offset_gate_m').value)
        if (not self.registered_slots or self.slot_match_tolerance <= 0.0 or
                min(self.slot_fit_long_margin, self.slot_fit_lat_margin,
                    self.slot_staging_gap) < 0.0 or
                self.initial_target_offset_gate <= 0.0 or
                self.rigid_body_lookahead <= 0.0 or
                self.entry_standoff <= 0.0 or self.approach_speed <= 0.0 or
                self.approach_yaw_gain <= 0.0 or
                self.approach_max_yaw_rate <= 0.0 or
                self.source_vehicle_fallback_mask <= 0.0):
            raise ValueError('invalid registered-slot parameters')
        if self.parking_direction not in (
                'minimum_rotation', 'forward', 'reverse'):
            raise ValueError(
                'parking_direction must be minimum_rotation, forward, or reverse')

        self.require_ui_confirmation = bool(
            self.get_parameter('require_ui_confirmation').value)
        self.ui_request_timeout = float(
            self.get_parameter('ui_request_timeout_s').value)
        if self.ui_request_timeout <= 0.0:
            raise ValueError('ui_request_timeout_s must be positive')
        self.ui_park_approved = False
        self.ui_approved_time = 0.0
        self.ui_request_id = ''
        self.ui_request_sequence = -1
        self.request_replay = RequestReplayGuard()
        self.loaded_footprint = compute_loaded_footprint(
            self.current_wheelbase,
            self.robot_length,
            self.robot_width,
            self.vehicle_length,
            self.vehicle_width,
            self.footprint_margin,
            self.vehicle_center_offset_body[0],
            self.vehicle_center_offset_body[1],
        )

        # 경로와 최종 슬롯은 임무당 한 번만 발행될 수 있다. 늦게 연결된
        # Front 제어기도 마지막 임무를 받을 수 있도록 latch 성격의 QoS를 쓴다.
        self.mission_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.coordination_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # A* 플래너: 운반 중 yaw를 고정하므로 결합 직사각형을 사용한다.
        self.planner = AStarPlanner(
            resolution=self.resolution,
            footprint_half_length_m=self.loaded_footprint.half_length_m,
            footprint_half_width_m=self.loaded_footprint.half_width_m,
            unknown_is_occupied=bool(
                self.unknown_is_occupied),
        )

        # 상태
        self.target_ready = False
        self.target_ready_since_ns = 0
        self.target_pose = None
        self.target_candidate_receipt_time = None
        self.empty_slots = []
        self.grid = None
        self.grid_w = 0
        self.grid_h = 0
        self.grid_origin_x_m = 0.0
        self.grid_origin_y_m = 0.0
        self.car_lifted = False
        self.state = 'WAIT_TARGET'
        self.path_published = False
        self.mission_id = ''
        self.mission_type = ''
        self.active_source_slot_id = ''
        self.active_destination_slot_id = ''
        self.requested_destination_slot_id = ''
        self.destination_kind = ''
        self.active_parking_direction = ''
        self.active_vehicle_spec = None
        self.active_vehicle_number = ''
        self.active_parking_credential = None
        self.active_plan_stamp_ns = 0
        self.pending_final_vehicle_pose = None
        self.active_committed_stages = set()
        self.last_commit_sequence = -1
        self.pending_completion = None
        self.request_status = None
        self.validation_warnings = []
        self.planning_blocker = None
        self.sync_fault = ''
        self.last_completed = None
        self.completion_sequence = 0
        self.status_sequence = 0
        self.front_odom = None
        self.rear_odom = None
        self.front_robot_state = 'UNKNOWN'
        self.rear_robot_state = 'UNKNOWN'
        self.front_motion_fault = ''
        self.rear_motion_fault = ''
        self.registry = ParkingRegistry(
            [slot.slot_id for slot in self.registered_slots],
            database_path=self.registry_database_path,
            layout_fingerprint=registered_slots_fingerprint(
                self.registered_slots))
        self.target_gate = StampGate(
            self.get_parameter('target_timeout_s').value,
            self.future_tolerance)
        self.spec_gate = StampGate(
            self.get_parameter('vehicle_spec_timeout_s').value,
            self.future_tolerance)
        self.require_valid_vehicle_spec = bool(
            self.get_parameter('require_valid_vehicle_spec').value)
        self.odom_gates = {
            'front': StampGate(self.odom_timeout, self.future_tolerance),
            'rear': StampGate(self.odom_timeout, self.future_tolerance),
        }

        # 구독
        self.create_subscription(PoseStamped, '/parking/target_pose',
                                 self.target_cb, 10)
        self.create_subscription(Bool, '/parking/target_ready',
                                 self.target_ready_cb, 10)
        self.create_subscription(PoseArray, '/parking/empty_slots',
                                 self.slots_cb, 10)
        self.create_subscription(OccupancyGrid, '/parking/map',
                                 self.map_cb, 10)
        self.create_subscription(Bool, '/robot/lifted',
                                 self.lifted_cb, 10)
        self.create_subscription(
            String, '/parking/vehicle_spec', self.vehicle_spec_cb,
            self.mission_qos)
        self.create_subscription(
            String, '/ui/mission_request', self.ui_request_cb, 10)
        self.create_subscription(
            String, '/mission/complete', self.mission_complete_cb, 10)
        self.create_subscription(
            String, '/mission/commit', self.mission_commit_cb,
            self.coordination_qos)
        self.create_subscription(
            String, '/sync/error_state', self.sync_status_cb, 10)
        self.create_subscription(
            Odometry, '/front/odom', self.front_odom_cb, SENSOR_LATEST_QOS)
        self.create_subscription(
            Odometry, '/rear/odom', self.rear_odom_cb, SENSOR_LATEST_QOS)
        self.create_subscription(
            String, '/front/robot_state', self.front_state_cb,
            STATE_LATEST_QOS)
        self.create_subscription(
            String, '/rear/robot_state', self.rear_state_cb,
            STATE_LATEST_QOS)
        self.create_subscription(
            String, '/front/motion_fault', self.front_fault_cb, 10)
        self.create_subscription(
            String, '/rear/motion_fault', self.rear_fault_cb, 10)

        # 발행
        self.pub_waypoints = self.create_publisher(
            Path, '/virtual_robot/waypoints', self.mission_qos)
        self.pub_state = self.create_publisher(
            String, '/fleet/state', STATE_LATEST_QOS)
        self.pub_slot_pose = self.create_publisher(
            PoseStamped, '/parking/slot_pose', self.mission_qos)
        self.pub_target_pose = self.create_publisher(
            PoseStamped, '/parking/target_pose', 10)
        self.pub_vehicle_spec = self.create_publisher(
            String, '/parking/vehicle_spec', self.mission_qos)

        self.create_timer(0.5, self.manage_loop)
        self.create_timer(1.0, self.publish_state)
        self.get_logger().info(
            'fleet_manager_node 시작 (fixed-yaw rectangular-footprint A*) | '
            f'footprint={self.loaded_footprint.length_m:.3f}x'
            f'{self.loaded_footprint.width_m:.3f}m')

    # ===== 콜백 =====
    def target_cb(self, msg):
        accepted, reason = self.target_gate.accept(
            stamp_to_ns(msg.header.stamp),
            self.get_clock().now().nanoseconds)
        if not accepted:
            self.get_logger().warn(
                f'target_pose rejected: {reason}',
                throttle_duration_sec=2.0)
            return
        if msg.header.frame_id not in ('', 'map'):
            self.get_logger().warn('target_pose frame must be map')
            return
        if not self.target_ready:
            return
        if stamp_to_ns(msg.header.stamp) < self.target_ready_since_ns:
            return
        self.target_pose = msg
        if self.state == 'WAIT_TARGET':
            self.target_candidate_receipt_time = time.monotonic()

    def _invalidate_pending_park(self, reason):
        changed = (
            self.target_pose is not None or
            self.target_candidate_receipt_time is not None or
            self.ui_park_approved)
        self.target_pose = None
        self.target_candidate_receipt_time = None
        if self.ui_park_approved:
            self.ui_park_approved = False
            self.requested_destination_slot_id = ''
            self.active_vehicle_number = ''
            self.active_parking_credential = None
            if (self.request_status is not None and
                    self.request_status.get('status') == 'ACCEPTED'):
                self.request_status = dict(self.request_status)
                self.request_status['status'] = 'REJECTED'
                self.request_status['reason'] = str(reason)
        if changed:
            self.publish_state()
        return changed

    def target_ready_cb(self, msg):
        previous = self.target_ready
        self.target_ready = bool(msg.data)
        if self.target_ready and not previous:
            self.target_ready_since_ns = self.get_clock().now().nanoseconds
        if not self.target_ready and self.state == 'WAIT_TARGET':
            if self._invalidate_pending_park('TARGET_NOT_READY'):
                self.get_logger().warn(
                    'target_ready=false — pending park candidate cancelled')

    def slots_cb(self, msg):
        """빈 Pose를 등록 슬롯과 다시 연결해 크기와 진입 Yaw를 보존한다."""
        matched = []
        for pose in msg.poses:
            x_m = float(pose.position.x)
            y_m = float(pose.position.y)
            candidate = min(
                self.registered_slots,
                key=lambda slot: math.hypot(
                    slot.center_x_m - x_m, slot.center_y_m - y_m))
            error = math.hypot(
                candidate.center_x_m - x_m,
                candidate.center_y_m - y_m)
            if error > self.slot_match_tolerance:
                self.get_logger().warn(
                    f'미등록 empty-slot pose ({x_m:.2f},{y_m:.2f}) 무시',
                    throttle_duration_sec=2.0)
                continue
            if candidate not in matched:
                matched.append(candidate)
        self.empty_slots = matched

    def map_cb(self, msg):
        width = int(msg.info.width)
        height = int(msg.info.height)
        resolution = float(msg.info.resolution)
        origin_x = float(msg.info.origin.position.x)
        origin_y = float(msg.info.origin.position.y)
        if (width <= 0 or height <= 0 or resolution <= 0.0 or
                not all(math.isfinite(value) for value in (
                    resolution, origin_x, origin_y))):
            self.get_logger().warn('잘못된 OccupancyGrid 메타데이터 무시')
            return
        if len(msg.data) != width * height:
            self.get_logger().warn(
                f'OccupancyGrid 크기 불일치: data={len(msg.data)}, '
                f'expected={width * height}')
            return

        self.grid = list(msg.data)
        self.grid_w = width
        self.grid_h = height
        self.resolution = resolution
        self.grid_origin_x_m = origin_x
        self.grid_origin_y_m = origin_y
        # AStarPlanner의 월드↔격자 변환을 수신 맵 geometry와 동기화한다.
        self.planner.set_map_geometry(resolution, origin_x, origin_y)

    def _set_request_status(self, payload, status, reason=''):
        self.request_status = {
            'request_id': str(payload.get('request_id', '')),
            'type': str(payload.get('type', '')),
            'source_slot_id': str(payload.get('source_slot_id', '')),
            'destination_slot_id': str(
                payload.get('destination_slot_id', '')),
            'status': str(status),
            'reason': str(reason),
        }
        self.publish_state()

    def _robots_accepting_mission(self):
        return (
            self.front_robot_state == 'IDLE' and
            self.rear_robot_state == 'IDLE' and
            not self.front_motion_fault and
            not self.rear_motion_fault)

    def ui_request_cb(self, msg):
        """UI intent를 검증한다. 실제 ACCEPTED/REJECTED 권한은 Fleet에 있다."""
        try:
            payload = json.loads(msg.data)
            request_type = str(payload['type'])
            request_id = str(payload.get('request_id', ''))
            client_id = str(payload.get('client_id', ''))
            sequence = int(payload['sequence'])
            stamp_ns = int(payload['stamp_ns'])
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warn(
                f'invalid ui/mission_request ignored: {exc}',
                throttle_duration_sec=2.0)
            self._set_request_status({}, 'REJECTED', 'INVALID_REQUEST')
            return
        accepted, replay_reason = self.request_replay.accept(
            client_id, sequence, request_id)
        if not accepted:
            self._set_request_status(
                payload, 'REJECTED', replay_reason)
            return
        age_s = (self.get_clock().now().nanoseconds - stamp_ns) * 1e-9
        if not -0.5 <= age_s <= self.ui_request_timeout:
            self.get_logger().warn(
                f'stale/future ui request rejected (age={age_s:.2f}s)')
            self._set_request_status(
                payload, 'REJECTED', 'STALE_REQUEST')
            return
        self.ui_request_sequence = sequence
        if request_type == 'park':
            self._handle_park_request(payload)
            return
        if request_type == 'retrieve':
            self._handle_retrieve_request(payload)
            return
        self._set_request_status(payload, 'REJECTED', 'INVALID_REQUEST')

    def _handle_park_request(self, payload):
        if (self.state != 'WAIT_TARGET' or
                self.mission_type not in ('', 'park') or
                self.ui_park_approved):
            self._set_request_status(
                payload, 'REJECTED', 'MISSION_ALREADY_ACTIVE')
            return
        if not self._robots_accepting_mission():
            self._set_request_status(
                payload, 'REJECTED', 'ROBOT_NOT_IDLE')
            return
        if not self.target_ready:
            self._set_request_status(
                payload, 'REJECTED', 'TARGET_NOT_READY')
            return
        if (self.require_valid_vehicle_spec and
                not self._vehicle_spec_ready()):
            self._set_request_status(
                payload, 'REJECTED', 'WAITING_VEHICLE_DIMENSION')
            return
        identity_keys = ('vehicle_number', 'password')
        has_identity = any(key in payload for key in identity_keys)
        has_identity = has_identity or 'destination_slot_id' in payload
        if has_identity:
            if not all(key in payload for key in identity_keys):
                self._set_request_status(
                    payload, 'REJECTED', 'INVALID_REQUEST')
                return
            try:
                vehicle_number = normalize_vehicle_number(
                    payload.get('vehicle_number'))
            except ValueError:
                self._set_request_status(
                    payload, 'REJECTED', 'INVALID_VEHICLE_NUMBER')
                return
            try:
                credential = ParkingCredential.create(
                    payload.get('password'))
            except ValueError:
                self._set_request_status(
                    payload, 'REJECTED', 'INVALID_PASSWORD')
                return
            if self.registry.find_by_vehicle_number(vehicle_number) is not None:
                self._set_request_status(
                    payload, 'REJECTED', 'VEHICLE_ALREADY_PARKED')
                return
            destination_slot_id = str(
                payload.get('destination_slot_id', '')).strip()
            perceived_empty = {
                candidate.slot_id for candidate in self.empty_slots}
            if destination_slot_id:
                slot = self._slot_by_id(destination_slot_id)
                if slot is None:
                    self._set_request_status(
                        payload, 'REJECTED', 'DESTINATION_SLOT_NOT_FOUND')
                    return
                if self.registry.lifecycle(
                        destination_slot_id) is not SlotLifecycle.EMPTY:
                    self._set_request_status(
                        payload, 'REJECTED', 'DESTINATION_SLOT_NOT_EMPTY')
                    return
                if destination_slot_id not in perceived_empty:
                    self._set_request_status(
                        payload, 'REJECTED', 'DESTINATION_SLOT_UNAVAILABLE')
                    return
            else:
                if not self._available_park_slot_ids():
                    self._set_request_status(
                        payload, 'REJECTED', 'DESTINATION_SLOT_UNAVAILABLE')
                    return
            self.requested_destination_slot_id = destination_slot_id
            self.active_vehicle_number = vehicle_number
            self.active_parking_credential = credential
        self.ui_park_approved = True
        self.ui_approved_time = time.monotonic()
        self.ui_request_id = str(payload.get('request_id', ''))
        self._set_request_status(payload, 'ACCEPTED')
        self.get_logger().info(f'UI 입차 승인 수신: {self.ui_request_id}')

    def _handle_retrieve_request(self, payload):
        if (self.state != 'WAIT_TARGET' or self.mission_id or
                self.target_pose is not None):
            self._set_request_status(
                payload, 'REJECTED', 'MISSION_ALREADY_ACTIVE')
            return
        if not self._robots_accepting_mission():
            self._set_request_status(
                payload, 'REJECTED', 'ROBOT_NOT_IDLE')
            return
        self._reset_planning_diagnostics()
        status_payload = dict(payload)
        if ('vehicle_number' not in payload or 'password' not in payload):
            self._set_request_status(
                status_payload, 'REJECTED',
                'VEHICLE_OR_PASSWORD_INVALID')
            return
        record = self.registry.authenticate_vehicle(
            payload.get('vehicle_number'), payload.get('password'))
        if record is None:
            self._set_request_status(
                status_payload, 'REJECTED',
                'VEHICLE_OR_PASSWORD_INVALID')
            return
        source_slot_id = record.slot_id
        supplied_slot = str(payload.get('source_slot_id', '')).strip()
        if supplied_slot and supplied_slot != source_slot_id:
            self._set_request_status(
                status_payload, 'REJECTED',
                'VEHICLE_OR_PASSWORD_INVALID')
            return
        status_payload['source_slot_id'] = source_slot_id
        if record.lifecycle is not SlotLifecycle.OCCUPIED:
            self._set_request_status(
                status_payload, 'REJECTED', 'SOURCE_SLOT_NOT_OCCUPIED')
            return
        if record.parking_direction != 'forward':
            self._set_request_status(
                status_payload, 'REJECTED',
                'UNSUPPORTED_PARKING_DIRECTION')
            return
        if record.final_vehicle_pose is None or record.vehicle_spec is None:
            self._set_request_status(
                status_payload, 'REJECTED', 'MISSING_VEHICLE_RECORD')
            return
        # 맵/현재 odometry는 모델 기반 corridor 판정이 아니라 경로 명령을
        # 만들기 위한 필수 입력이다. WARN_ONLY에서도 절대로 우회하지 않는다.
        if self.grid is None:
            self._set_request_status(
                status_payload, 'REJECTED', 'MAP_MISSING')
            return
        if self.current_virtual_start() is None:
            self._set_request_status(
                status_payload, 'REJECTED', 'ODOM_MISSING_OR_STALE')
            return
        approach_clear = self._retrieve_approach_preflight(record)
        if not self._validation_allows(
                approach_clear,
                'APPROACH_CORRIDOR_BLOCKED',
                mission_phase='REQUEST'):
            self._set_request_status(
                status_payload, 'REJECTED', 'APPROACH_CORRIDOR_BLOCKED')
            return

        mission_id = f'mission-{self.get_clock().now().nanoseconds}'
        try:
            self.registry.reserve_retrieve(source_slot_id, mission_id)
        except (KeyError, RegistryTransitionError):
            self._set_request_status(
                status_payload, 'REJECTED', 'SOURCE_SLOT_NOT_OCCUPIED')
            return
        self.mission_id = mission_id
        self.mission_type = 'retrieve'
        self.active_source_slot_id = source_slot_id
        self.destination_kind = 'WAITING'
        self.active_parking_direction = record.parking_direction
        self.active_vehicle_spec = dict(record.vehicle_spec)
        self._apply_active_vehicle_spec()
        self.target_pose = self._publish_retrieve_target(
            record.final_vehicle_pose)
        self.state = 'WAIT_LIFT'
        self.ui_request_id = str(status_payload.get('request_id', ''))
        self._set_request_status(status_payload, 'ACCEPTED')
        self.get_logger().info(
            f'UI 출차 승인: {source_slot_id} ({mission_id})')

    def _validation_allows(self, passed, code, mission_phase):
        """Apply the configured policy to a model-based planning finding.

        WARN_ONLY never hides the finding: it records a bounded, UI-visible
        warning and emits a ROS warning, but permits the existing mission
        algorithm to continue.  Missing data, identity/lifecycle checks and
        runtime emergency stops do not call this helper.
        """
        if passed:
            return True
        if getattr(self, 'planning_validation_mode', 'enforce') != 'warn_only':
            return False
        warning = {
            'code': str(code),
            'mission_phase': str(mission_phase),
        }
        warnings = getattr(self, 'validation_warnings', [])
        is_new = warning not in warnings
        if is_new:
            warnings.append(warning)
            del warnings[:-16]
        self.validation_warnings = warnings
        if is_new:
            self.get_logger().warn(
                f'WARN_ONLY planning validation: {code} '
                f'(phase={mission_phase})')
        return True

    def _reset_planning_diagnostics(self):
        self.validation_warnings = []
        self.planning_blocker = None

    def _set_planning_blocker(self, code, mission_phase='PLAN_PATH'):
        """Publish a stable reason when no executable command can be built."""
        blocker = {
            'code': str(code),
            'mission_phase': str(mission_phase),
        }
        if getattr(self, 'planning_blocker', None) != blocker:
            self.planning_blocker = blocker
            publish_state = getattr(self, 'publish_state', None)
            if callable(publish_state):
                publish_state()
        return False

    def sync_status_cb(self, msg):
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        error = str(payload.get('error', 'OK')).strip()
        if (self.mission_id and self.state in ('PLAN_PATH', 'NAVIGATING') and
                is_fatal_sync_error(error)):
            self.sync_fault = error
            self.state = 'FAULT'
            self.path_published = False
            self.publish_state()
            self.get_logger().error(
                f'fleet mission stopped by rigid-body fault: {error}')
            return
        if (not self.mission_id or self.active_plan_stamp_ns <= 0 or
                self.state not in ('PLAN_PATH', 'NAVIGATING')):
            return
        parsed = parse_arrival_status(
            payload, self.active_plan_stamp_ns)
        if parsed is None:
            return
        self.pending_final_vehicle_pose = Pose2D(*parsed)
        # ARRIVED와 RETURN commit은 서로 다른 topic이므로 어느 쪽이 먼저
        # 도착해도 같은 idempotent Registry 확정을 재시도한다.
        FleetManagerNode._try_complete_park_registry(self)

    def _try_complete_park_registry(self):
        if (self.mission_type != 'park' or
                not self.active_destination_slot_id or
                self.pending_final_vehicle_pose is None or
                self.active_vehicle_spec is None or
                not {'RELEASE', 'RETURN'}.issubset(
                    self.active_committed_stages)):
            return False
        try:
            lifecycle = self.registry.lifecycle(
                self.active_destination_slot_id)
            if lifecycle is SlotLifecycle.OCCUPIED:
                return True
            if lifecycle is not SlotLifecycle.RESERVED:
                return False
            self.registry.complete_park(
                self.active_destination_slot_id,
                self.mission_id,
                self.pending_final_vehicle_pose,
                self.active_parking_direction,
                self.active_vehicle_spec)
        except (KeyError, RegistryTransitionError) as exc:
            self.get_logger().error(
                f'park registry completion rejected: {exc}')
            return False
        self.publish_state()
        return True

    def mission_commit_cb(self, msg):
        try:
            payload = json.loads(msg.data)
            mission_id = str(payload['mission_id'])
            role = str(payload['role'])
            stage = str(payload['stage'])
            sequence = int(payload['sequence'])
            stamp_ns = int(payload['stamp_ns'])
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            return
        age_s = (self.get_clock().now().nanoseconds - stamp_ns) * 1e-9
        if (not self.mission_id or mission_id != self.mission_id or
                role != 'front' or sequence <= self.last_commit_sequence or
                stage not in ('LIFT', 'DRIVE', 'RELEASE', 'RETURN', 'HOME') or
                not -0.5 <= age_s <= 10.0):
            return
        self.last_commit_sequence = sequence
        self.active_committed_stages.add(stage)

        try:
            if (stage == 'DRIVE' and self.mission_type == 'retrieve' and
                    self.active_source_slot_id):
                self.registry.mark_retrieve_exiting(
                    self.active_source_slot_id, self.mission_id)
                self.publish_state()
            elif stage == 'RETURN' and 'RELEASE' in self.active_committed_stages:
                if self.mission_type == 'park':
                    FleetManagerNode._try_complete_park_registry(self)
                elif (self.mission_type == 'retrieve' and
                      self.destination_kind == 'WAITING' and
                      self.active_source_slot_id):
                    self.registry.complete_retrieve(
                        self.active_source_slot_id, self.mission_id)
                    self.publish_state()
            elif (stage == 'HOME' and self.pending_completion is not None and
                  FleetManagerNode._registry_completion_ready(self)):
                pending = self.pending_completion
                self.pending_completion = None
                self._finalize_mission(pending)
        except RegistryTransitionError as exc:
            self.get_logger().error(f'registry transition rejected: {exc}')

    def mission_complete_cb(self, msg):
        try:
            payload = json.loads(msg.data)
            mission_id = str(payload['mission_id'])
            stamp_ns = int(payload['stamp_ns'])
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            self.get_logger().warn('invalid mission/complete envelope')
            return
        age_s = (self.get_clock().now().nanoseconds - stamp_ns) * 1e-9
        if (not -0.5 <= age_s <= 10.0 or not self.mission_id or
                mission_id != self.mission_id):
            return
        if 'HOME' not in self.active_committed_stages:
            # commit과 complete는 서로 다른 topic이므로 DDS 도착 순서가 바뀔 수 있다.
            self.pending_completion = payload
            return
        if not FleetManagerNode._registry_completion_ready(self):
            self.pending_completion = payload
            self.get_logger().error(
                'mission complete blocked: parking registry transition '
                'is not complete')
            return
        self._finalize_mission(payload)

    def _registry_completion_ready(self):
        """완료 표시 전에 차량 배치 lifecycle이 확정됐는지 검사한다."""
        try:
            if self.mission_type == 'park':
                return bool(
                    self.active_destination_slot_id and
                    self.registry.lifecycle(
                        self.active_destination_slot_id) is
                    SlotLifecycle.OCCUPIED)
            if self.mission_type == 'retrieve':
                return bool(
                    self.active_source_slot_id and
                    self.registry.lifecycle(
                        self.active_source_slot_id) is SlotLifecycle.EMPTY)
        except KeyError:
            return False
        return False

    def _finalize_mission(self, payload):
        mission_id = self.mission_id
        self.completion_sequence += 1
        self.last_completed = {
            'completion_sequence': self.completion_sequence,
            'mission_id': mission_id,
            'mission_type': self.mission_type,
            'source_slot_id': self.active_source_slot_id,
            'stamp_ns': int(payload['stamp_ns']),
        }
        if (self.request_status is not None and
                self.request_status.get('request_id') == self.ui_request_id and
                self.request_status.get('status') == 'ACCEPTED'):
            self.request_status = dict(self.request_status)
            self.request_status['status'] = 'COMPLETED'
            self.request_status['reason'] = ''

        self.state = 'WAIT_TARGET'
        self.mission_id = ''
        self.mission_type = ''
        self.active_source_slot_id = ''
        self.active_destination_slot_id = ''
        self.requested_destination_slot_id = ''
        self.destination_kind = ''
        self.active_parking_direction = ''
        self.active_vehicle_spec = None
        self.active_vehicle_number = ''
        self.active_parking_credential = None
        self.active_plan_stamp_ns = 0
        self.pending_final_vehicle_pose = None
        self.active_committed_stages.clear()
        self.last_commit_sequence = -1
        self.pending_completion = None
        self._reset_planning_diagnostics()
        self.car_lifted = False
        self.target_pose = None
        self.empty_slots = []
        self.path_published = False
        self.ui_park_approved = False
        self.ui_request_id = ''
        # 차량 중심 offset은 임무별로 다시 잡는다. 남겨 두면 다음 A* 시작점이
        # 지난 임무의 offset만큼 틀어진다.
        self.vehicle_center_offset_body = [0.0, 0.0]
        self.loaded_footprint = compute_loaded_footprint(
            self.current_wheelbase,
            self.robot_length,
            self.robot_width,
            self.vehicle_length,
            self.vehicle_width,
            self.footprint_margin,
            0.0,
            0.0,
        )
        self.planner.set_footprint(
            self.loaded_footprint.half_length_m,
            self.loaded_footprint.half_width_m)
        # StampGate는 마지막 stamp를 기억하므로 새 임무 target을 받으려면
        # 반드시 함께 초기화해야 한다.
        self.target_gate.reset()
        self.spec_gate.reset()
        self.publish_state()
        self.get_logger().info(
            f'임무 {mission_id} 완료 — WAIT_TARGET 복귀')

    def lifted_cb(self, msg):
        if msg.data and not self.car_lifted:
            self.car_lifted = True
            self.get_logger().info('차량 들림 신호 수신')

    @staticmethod
    def _optional_dimension(payload, keys, default):
        for key in keys:
            if key in payload:
                return float(payload[key])
        return default

    def _vehicle_spec_ready(self):
        """True only for a current, explicitly dimension-valid observation."""
        spec = self.active_vehicle_spec
        if spec is None or not bool(spec.get('dimension_valid', False)):
            return False
        stamp_ns = self.spec_gate.last_stamp_ns
        now_ns = self.get_clock().now().nanoseconds
        age_ns = now_ns - stamp_ns
        return (-self.spec_gate.future_tolerance_ns <= age_ns <=
                self.spec_gate.max_age_ns)

    def vehicle_spec_cb(self, msg):
        try:
            payload = json.loads(msg.data)
            if self.require_valid_vehicle_spec and not bool(
                    payload.get('dimension_valid', False)):
                raise ValueError('vehicle_spec dimension invalid')
            accepted, reason = self.spec_gate.accept(
                int(payload['stamp_ns']),
                self.get_clock().now().nanoseconds)
            if not accepted:
                raise ValueError(f'vehicle_spec {reason}')
            wheelbase = float(payload['wheelbase'])
            validate_wheelbase_clearance(
                wheelbase, self.robot_length,
                self.minimum_inter_robot_gap)
            vehicle_length = self._optional_dimension(
                payload,
                ('vehicle_length_m', 'length_m', 'vehicle_length'),
                self.vehicle_length)
            vehicle_width = self._optional_dimension(
                payload,
                ('vehicle_width_m', 'width_m', 'vehicle_width'),
                self.vehicle_width)
            footprint = compute_loaded_footprint(
                wheelbase,
                self.robot_length,
                self.robot_width,
                vehicle_length,
                vehicle_width,
                self.footprint_margin,
                self.vehicle_center_offset_body[0],
                self.vehicle_center_offset_body[1],
            )
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warn(
                f'invalid vehicle_spec ignored: {exc}',
                throttle_duration_sec=2.0)
            return

        if self.car_lifted or self.state in ('PLAN_PATH', 'NAVIGATING'):
            changed = (
                abs(wheelbase - self.current_wheelbase) > 1e-6 or
                abs(vehicle_length - self.vehicle_length) > 1e-6 or
                abs(vehicle_width - self.vehicle_width) > 1e-6)
            if changed:
                self.get_logger().error(
                    'vehicle geometry changed after lift; ignored for safety')
            return

        self.current_wheelbase = wheelbase
        self.vehicle_length = vehicle_length
        self.vehicle_width = vehicle_width
        self.active_vehicle_spec = {
            'wheelbase': wheelbase,
            'vehicle_length_m': vehicle_length,
            'vehicle_width_m': vehicle_width,
            'dimension_valid': bool(payload.get('dimension_valid', False)),
        }
        self.loaded_footprint = footprint
        self.planner.set_footprint(
            footprint.half_length_m, footprint.half_width_m)
        cells = footprint.half_extent_cells(self.resolution)
        self.get_logger().info(
            f'mission footprint={footprint.length_m:.3f}x'
            f'{footprint.width_m:.3f}m, half_cells={cells}, '
            f'wheelbase={wheelbase:.3f}m')

    def front_odom_cb(self, msg):
        self._odom_cb('front', msg)

    def rear_odom_cb(self, msg):
        self._odom_cb('rear', msg)

    def front_state_cb(self, msg):
        self.front_robot_state = str(msg.data)

    def rear_state_cb(self, msg):
        self.rear_robot_state = str(msg.data)

    def front_fault_cb(self, msg):
        self.front_motion_fault = str(msg.data).strip()

    def rear_fault_cb(self, msg):
        self.rear_motion_fault = str(msg.data).strip()

    def _odom_cb(self, role, msg):
        if msg.header.frame_id not in ('', 'map'):
            return
        accepted, _ = self.odom_gates[role].accept(
            stamp_to_ns(msg.header.stamp),
            self.get_clock().now().nanoseconds)
        if not accepted:
            return
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            return
        value = {'x': x, 'y': y, 'yaw': yaw, 'receipt': time.monotonic()}
        if role == 'front':
            self.front_odom = value
        else:
            self.rear_odom = value

    def current_virtual_start(self):
        """Front/Rear 중점과 두 yaw의 원형 평균으로 현재 강체 Pose를 만든다."""
        if self.front_odom is None or self.rear_odom is None:
            return None
        now = time.monotonic()
        if (now - self.front_odom['receipt'] > self.odom_timeout or
                now - self.rear_odom['receipt'] > self.odom_timeout):
            return None
        yaw = math.atan2(
            math.sin(self.front_odom['yaw']) + math.sin(self.rear_odom['yaw']),
            math.cos(self.front_odom['yaw']) + math.cos(self.rear_odom['yaw']))
        return Pose2D(
            (self.front_odom['x'] + self.rear_odom['x']) / 2.0,
            (self.front_odom['y'] + self.rear_odom['y']) / 2.0,
            yaw)

    def _slot_by_id(self, slot_id):
        return next((slot for slot in self.registered_slots
                     if slot.slot_id == slot_id), None)

    def _retrieve_approach_preflight(self, record):
        target = record.final_vehicle_pose
        routes = {}
        for role, odom in (
                ('front', self.front_odom), ('rear', self.rear_odom)):
            longitudinal = approach_longitudinal(
                role, self.entry_standoff,
                float(record.vehicle_spec['wheelbase']))
            goal = vehicle_to_world(
                longitudinal, 0.0,
                target.x_m, target.y_m, target.yaw_rad)
            start = (odom['x'], odom['y'])
            translation_goal_yaw = (
                target.yaw_rad if self.simultaneous_entry else None)
            if not corridor_is_free(
                    self.grid, self.grid_w, self.grid_h, self.resolution,
                    start, goal, odom['yaw'],
                    self.robot_length, self.robot_width,
                    margin_m=0.0,
                    unknown_is_occupied=self.unknown_is_occupied,
                    goal_yaw_rad=translation_goal_yaw,
                    speed_mps=self.approach_speed,
                    yaw_gain=self.approach_yaw_gain,
                    max_yaw_rate=self.approach_max_yaw_rate,
                    origin_x_m=getattr(self, 'grid_origin_x_m', 0.0),
                    origin_y_m=getattr(self, 'grid_origin_y_m', 0.0)):
                return False
            if (not self.simultaneous_entry and
                    not corridor_is_free(
                        self.grid, self.grid_w, self.grid_h,
                        self.resolution,
                        goal, goal, odom['yaw'],
                        self.robot_length, self.robot_width,
                        margin_m=0.0,
                        unknown_is_occupied=self.unknown_is_occupied,
                        goal_yaw_rad=target.yaw_rad,
                        speed_mps=self.approach_speed,
                        yaw_gain=self.approach_yaw_gain,
                        max_yaw_rate=self.approach_max_yaw_rate,
                        origin_x_m=getattr(
                            self, 'grid_origin_x_m', 0.0),
                        origin_y_m=getattr(
                            self, 'grid_origin_y_m', 0.0))):
                return False
            routes[role] = (start, goal)
        route_clearance = (simultaneous_routes_clear
                           if self.simultaneous_entry
                           else sequential_routes_clear)
        if not route_clearance(
                routes['front'], routes['rear'], self.approach_speed,
                self.robot_length, self.robot_width,
                self.minimum_inter_robot_gap,
                self.front_odom['yaw'], self.rear_odom['yaw'],
                front_goal_yaw_rad=target.yaw_rad,
                rear_goal_yaw_rad=target.yaw_rad,
                yaw_gain=self.approach_yaw_gain,
                max_yaw_rate=self.approach_max_yaw_rate):
            return False
        return True

    def _apply_active_vehicle_spec(self):
        spec = self.active_vehicle_spec
        self.current_wheelbase = float(spec['wheelbase'])
        self.vehicle_length = float(spec['vehicle_length_m'])
        self.vehicle_width = float(spec['vehicle_width_m'])
        self.vehicle_center_offset_body = [0.0, 0.0]
        self.loaded_footprint = compute_loaded_footprint(
            self.current_wheelbase,
            self.robot_length,
            self.robot_width,
            self.vehicle_length,
            self.vehicle_width,
            self.footprint_margin,
            0.0,
            0.0,
        )
        self.planner.set_footprint(
            self.loaded_footprint.half_length_m,
            self.loaded_footprint.half_width_m)

    def _publish_retrieve_target(self, pose):
        now = self.get_clock().now()
        stamp = now.to_msg()
        target = PoseStamped()
        target.header.stamp = stamp
        target.header.frame_id = 'map'
        target.pose.position.x = pose.x_m
        target.pose.position.y = pose.y_m
        target.pose.orientation.z = math.sin(pose.yaw_rad / 2.0)
        target.pose.orientation.w = math.cos(pose.yaw_rad / 2.0)
        spec = dict(self.active_vehicle_spec)
        spec['stamp_ns'] = now.nanoseconds
        self.pub_target_pose.publish(target)
        self.pub_vehicle_spec.publish(String(data=json.dumps(spec)))
        return target

    def _grid_occupied(self, gx, gy):
        """맵 밖과 설정상 unknown을 점유로 포함하는 단일 셀 판정."""
        if gx < 0 or gy < 0 or gx >= self.grid_w or gy >= self.grid_h:
            return True
        value = int(self.grid[gy * self.grid_w + gx])
        return value >= 50 or (value < 0 and self.unknown_is_occupied)

    def _oriented_footprint_free(self, x_m, y_m, yaw_rad):
        """한 자세에서 회전된 loaded rectangle가 점유 셀과 겹치는지 검사."""
        origin_x = getattr(self, 'grid_origin_x_m', 0.0)
        origin_y = getattr(self, 'grid_origin_y_m', 0.0)
        half_length = (
            self.loaded_footprint.half_length_m + self.slot_fit_long_margin)
        half_width = (
            self.loaded_footprint.half_width_m + self.slot_fit_lat_margin)
        radius = math.hypot(half_length, half_width)
        min_gx = int(math.floor(
            (x_m - radius - origin_x) / self.resolution))
        max_gx = int(math.floor(
            (x_m + radius - origin_x) / self.resolution))
        min_gy = int(math.floor(
            (y_m - radius - origin_y) / self.resolution))
        max_gy = int(math.floor(
            (y_m + radius - origin_y) / self.resolution))
        c, s = math.cos(yaw_rad), math.sin(yaw_rad)
        # 셀 중심만 검사할 때 모서리 접촉을 놓치지 않도록 셀 반대각을 더한다.
        cell_padding = self.resolution / math.sqrt(2.0)
        for gy in range(min_gy, max_gy + 1):
            for gx in range(min_gx, max_gx + 1):
                if not self._grid_occupied(gx, gy):
                    continue
                cell_x = origin_x + (gx + 0.5) * self.resolution
                cell_y = origin_y + (gy + 0.5) * self.resolution
                dx, dy = cell_x - x_m, cell_y - y_m
                local_x = dx * c + dy * s
                local_y = -dx * s + dy * c
                if (abs(local_x) <= half_length + cell_padding and
                        abs(local_y) <= half_width + cell_padding):
                    return False
        return True

    def _rotation_space_free(self, x_m, y_m):
        """정렬점에서 어떤 중간 Yaw로 돌아도 되는 반대각 원 공간 검사."""
        origin_x = getattr(self, 'grid_origin_x_m', 0.0)
        origin_y = getattr(self, 'grid_origin_y_m', 0.0)
        radius = 0.5 * math.hypot(
            self.loaded_footprint.length_m + 2.0 * self.slot_fit_long_margin,
            self.loaded_footprint.width_m + 2.0 * self.slot_fit_lat_margin)
        cell_padding = self.resolution / math.sqrt(2.0)
        min_gx = int(math.floor(
            (x_m - radius - origin_x) / self.resolution))
        max_gx = int(math.floor(
            (x_m + radius - origin_x) / self.resolution))
        min_gy = int(math.floor(
            (y_m - radius - origin_y) / self.resolution))
        max_gy = int(math.floor(
            (y_m + radius - origin_y) / self.resolution))
        for gy in range(min_gy, max_gy + 1):
            for gx in range(min_gx, max_gx + 1):
                cell_x = origin_x + (gx + 0.5) * self.resolution
                cell_y = origin_y + (gy + 0.5) * self.resolution
                if (math.hypot(cell_x - x_m, cell_y - y_m) <=
                        radius + cell_padding and self._grid_occupied(gx, gy)):
                    return False
        return True

    def _insertion_corridor_free(self, start, goal):
        """정렬점부터 슬롯 중심까지 목표 Yaw 직사각형을 일정 간격으로 검사."""
        dx = goal.x_m - start.x_m
        dy = goal.y_m - start.y_m
        distance = math.hypot(dx, dy)
        sample_count = max(1, int(math.ceil(
            distance / max(self.resolution * 0.5, 1e-3))))
        for index in range(sample_count + 1):
            ratio = index / sample_count
            if not self._oriented_footprint_free(
                    start.x_m + ratio * dx,
                    start.y_m + ratio * dy,
                    goal.yaw_rad):
                return False
        return True

    # ===== 관제 로직 =====
    def manage_loop(self):
        if self.state == 'WAIT_TARGET':
            if not self.target_ready:
                self._invalidate_pending_park('TARGET_NOT_READY')
                return
            if (self.target_pose is not None and
                    self.target_candidate_receipt_time is not None and
                    time.monotonic() - self.target_candidate_receipt_time >
                    self.target_candidate_timeout):
                self.target_pose = None
                self.target_candidate_receipt_time = None
                self.active_vehicle_spec = None
                if self.ui_park_approved:
                    self.ui_park_approved = False
                    self.requested_destination_slot_id = ''
                    self.active_vehicle_number = ''
                    self.active_parking_credential = None
                    if self.request_status is not None:
                        self.request_status = dict(self.request_status)
                        self.request_status['status'] = 'REJECTED'
                        self.request_status['reason'] = 'TARGET_TIMEOUT'
                self.publish_state()
                self.get_logger().warn('stale candidate target cleared')
            # 차가 아직 없는데 버튼이 먼저 눌린 경우, 승인이 무기한 남아 있으면
            # 한참 뒤에 들어온 차가 예고 없이 실려 나간다. 반드시 만료시킨다.
            if (self.ui_park_approved and
                    time.monotonic() - self.ui_approved_time >
                    self.ui_request_timeout):
                self.ui_park_approved = False
                self.requested_destination_slot_id = ''
                self.active_vehicle_number = ''
                self.active_parking_credential = None
                if (self.request_status is not None and
                        self.request_status.get('request_id') ==
                        self.ui_request_id and
                        self.request_status.get('status') == 'ACCEPTED'):
                    self.request_status = dict(self.request_status)
                    self.request_status['status'] = 'REJECTED'
                    self.request_status['reason'] = 'TARGET_TIMEOUT'
                    self.publish_state()
                self.get_logger().warn('UI 입차 승인 만료 — 다시 눌러야 합니다')
            if self.target_pose is not None:
                if (self.require_valid_vehicle_spec and
                        not self._vehicle_spec_ready()):
                    return
                if not self.require_ui_confirmation:
                    self.mission_id = (
                        f'mission-{self.get_clock().now().nanoseconds}')
                    self.mission_type = 'park'
                    self._reset_planning_diagnostics()
                    self.state = 'WAIT_LIFT'
                    self.publish_state()
                elif self.ui_park_approved:
                    # 승인은 1회성이다. 진입 순간 즉시 소비해야 다음 임무가
                    # 버튼 없이 자동 시작되는 일을 막을 수 있다.
                    self.ui_park_approved = False
                    self.mission_id = (
                        f'mission-{self.get_clock().now().nanoseconds}')
                    self.mission_type = 'park'
                    self._reset_planning_diagnostics()
                    self.state = 'WAIT_LIFT'
                    self.publish_state()
                    self.get_logger().info(
                        f'UI 승인으로 임무 시작 ({self.ui_request_id})')

        elif self.state == 'WAIT_LIFT':
            if self.mission_type == 'retrieve' and not self.car_lifted:
                try:
                    record = self.registry.get(self.active_source_slot_id)
                except KeyError:
                    record = None
                if (
                        record is not None and
                        record.lifecycle is SlotLifecycle.EXIT_RESERVED and
                        record.reservation_mission_id == self.mission_id and
                        record.final_vehicle_pose is not None and
                        record.vehicle_spec is not None):
                    # Front-first에서는 Rear가 WAIT_FRONT_STAGED에서 target
                    # freshness 창보다 오래 기다릴 수 있다. Registry pose는
                    # 이 미션 동안 고정된 권위값이므로 기존 topic을 fresh
                    # stamp로 유지해 Rear도 같은 target을 latch하게 한다.
                    self.target_pose = self._publish_retrieve_target(
                        record.final_vehicle_pose)
            if self.car_lifted:
                self.state = 'PLAN_PATH'
                self.publish_state()

        elif self.state == 'PLAN_PATH':
            if self.plan_and_publish():
                self.state = 'NAVIGATING'
                self.publish_state()

        elif self.state == 'NAVIGATING':
            pass  # rigid_body_sync가 주행

    def _available_park_slot_ids(self):
        '''Return slots that both Registry and CCTV currently consider empty.'''
        perceived_empty = {slot.slot_id for slot in self.empty_slots}
        return [
            slot_id for slot_id in self.registry.empty_slot_ids()
            if slot_id in perceived_empty
        ]

    def _eligible_park_slots(self):
        '''Apply an optional operator selection to currently available slots.'''
        available = set(self._available_park_slot_ids())
        requested = getattr(self, 'requested_destination_slot_id', '')
        return [
            slot for slot in self.empty_slots
            if slot.slot_id in available and
            (not requested or slot.slot_id == requested)
        ]

    def plan_and_publish(self):
        """크기가 맞는 빈자리 선택 -> 정렬점 A* -> 슬롯 목표 자세 발행."""
        if self.mission_type == 'retrieve':
            return self.plan_retrieve_and_publish()
        if self.grid is None:
            self.get_logger().warn('맵 미수신 — 대기', throttle_duration_sec=2.0)
            return self._set_planning_blocker('MAP_MISSING')
        if not self.empty_slots:
            self.get_logger().warn('빈자리 없음', throttle_duration_sec=2.0)
            return self._set_planning_blocker('NO_EMPTY_SLOT')

        raw_start = self.current_virtual_start()
        if raw_start is None:
            self.get_logger().warn(
                'Front/Rear 최신 odom 없음 — 실제 base_virtual 시작점 대기',
                throttle_duration_sec=2.0)
            return self._set_planning_blocker('ODOM_MISSING_OR_STALE')
        start = raw_start

        # rigid_body_sync는 인양 직후 target_pose로 차량중심 offset을 초기화한다.
        # Fleet도 같은 기준으로 A* 시작점을 잡아야 계획과 추종 좌표가
        # 서로 어긋나지 않는다. gate 밖이면 오인식으로 보고 odom 중점을 쓴다.
        if self.target_pose is not None:
            target_x = float(self.target_pose.pose.position.x)
            target_y = float(self.target_pose.pose.position.y)
            world_dx = target_x - raw_start.x_m
            world_dy = target_y - raw_start.y_m
            offset = math.hypot(world_dx, world_dy)
            if offset <= self.initial_target_offset_gate:
                c, s = math.cos(raw_start.yaw_rad), math.sin(raw_start.yaw_rad)
                self.vehicle_center_offset_body = [
                    c * world_dx + s * world_dy,
                    -s * world_dx + c * world_dy,
                ]
                start = Pose2D(target_x, target_y, raw_start.yaw_rad)
                self.get_logger().info(
                    f'A* 시작점에 CCTV 차량중심 body-offset '
                    f'({self.vehicle_center_offset_body[0]:+.3f},'
                    f'{self.vehicle_center_offset_body[1]:+.3f})m 반영')
            else:
                self.vehicle_center_offset_body = [0.0, 0.0]
                self.get_logger().warn(
                    f'CCTV 차량중심 offset gate 초과({offset:.3f}m) — '
                    'odom 중점 사용')

        # 차량 중심을 기준으로 회전하므로, 로봇 중점과의 body offset까지
        # 포함한 대칭 외접 footprint로 fit/A*/sweep를 다시 설정한다.
        self.loaded_footprint = compute_loaded_footprint(
            self.current_wheelbase,
            self.robot_length,
            self.robot_width,
            self.vehicle_length,
            self.vehicle_width,
            self.footprint_margin,
            self.vehicle_center_offset_body[0],
            self.vehicle_center_offset_body[1],
        )

        # A* 구간에서는 초기 yaw를 유지한다. 해당 yaw의 직사각형을 map x/y축에
        # 투영한 envelope를 사용해야 0도가 아닌 차량도 작게 취급하지 않는다.
        path_length, path_width = footprint_extents_in_slot_axes(
            self.loaded_footprint.length_m,
            self.loaded_footprint.width_m,
            start.yaw_rad)
        self.planner.set_footprint(path_length / 2.0, path_width / 2.0)

        # 차량만이 아니라 Front+차량+Rear loaded footprint가 들어가는 슬롯만 남긴다.
        compatible = []
        for slot in FleetManagerNode._eligible_park_slots(self):
            fit = check_slot_fit(
                slot,
                self.loaded_footprint.length_m,
                self.loaded_footprint.width_m,
                self.slot_fit_long_margin,
                self.slot_fit_lat_margin)
            if not fit.fits:
                allowed = self._validation_allows(
                    False, fit.reason, mission_phase='PLAN_PATH')
                self.get_logger().warn(
                    f'슬롯 {slot.slot_id} '
                    f'{"경고 후 사용" if allowed else "제외"}: '
                    f'{fit.reason} | '
                    f'clearance L={fit.length_clearance_m:+.3f}m '
                    f'W={fit.width_clearance_m:+.3f}m',
                    throttle_duration_sec=2.0)
                if not allowed:
                    continue
            compatible.append((slot, fit))
        if not compatible:
            self.get_logger().warn(
                '빈 슬롯은 있지만 loaded footprint가 들어갈 슬롯이 없음',
                throttle_duration_sec=2.0)
            return self._set_planning_blocker('PARK_SLOT_FIT_BLOCKED')

        compatible.sort(key=lambda item: math.hypot(
            item[0].center_x_m - start.x_m,
            item[0].center_y_m - start.y_m))
        selected_slot = None
        selected_fit = None
        selected_approach = None
        waypoints = None
        saw_astar_failure = False
        saw_validation_failure = False
        for slot, fit in compatible:
            if self.use_staged_slot_entry:
                approach_candidates = make_approach_candidates(
                    slot,
                    self.loaded_footprint.length_m,
                    self.slot_staging_gap,
                    start.yaw_rad)
                if self.parking_direction in ('forward', 'reverse'):
                    approach_candidates = [
                        candidate for candidate in approach_candidates
                        if candidate.parking_direction == self.parking_direction]
            else:
                # 레거시 모드도 슬롯 yaw는 보존하지만 A* 목표가 바로 중심이다.
                approach_candidates = make_approach_candidates(
                    slot, self.loaded_footprint.length_m, 0.0, start.yaw_rad)

            for approach in approach_candidates:
                path_goal = (approach.staging_pose.position
                             if self.use_staged_slot_entry else slot.center)
                candidate_path = self.planner.plan(
                    self.grid, self.grid_w, self.grid_h,
                    start.position, path_goal)
                if candidate_path is None:
                    saw_astar_failure = True
                    continue
                if self.use_staged_slot_entry:
                    # 2D A*는 회전/삽입을 모른다. 정렬점 회전 원과 슬롯 축
                    # 직선 삽입 corridor를 별도로 검사한다. WARN_ONLY에서는
                    # 진단 결과를 남기되 기존 trajectory 생성을 계속한다.
                    rotation_free = self._rotation_space_free(
                        *approach.staging_pose.position)
                    if not self._validation_allows(
                            rotation_free,
                            'PARK_ROTATION_SPACE_BLOCKED',
                            mission_phase='PLAN_PATH'):
                        saw_validation_failure = True
                        continue
                    insertion_free = self._insertion_corridor_free(
                        approach.staging_pose, approach.target_pose)
                    if not self._validation_allows(
                            insertion_free,
                            'PARK_INSERTION_CORRIDOR_BLOCKED',
                            mission_phase='PLAN_PATH'):
                        saw_validation_failure = True
                        continue
                    if math.hypot(
                            candidate_path[-1][0] - path_goal[0],
                            candidate_path[-1][1] - path_goal[1]) > 1e-6:
                        candidate_path.append(path_goal)
                selected_slot = slot
                selected_fit = fit
                selected_approach = approach
                waypoints = candidate_path
                break
            if waypoints is not None:
                break

        if selected_slot is None or selected_approach is None or waypoints is None:
            self.get_logger().error(
                '모든 적합 슬롯의 A*/회전공간/삽입경로 검사 실패 — 재계획')
            return self._set_planning_blocker(
                'PARK_PATH_VALIDATION_BLOCKED'
                if saw_validation_failure else
                ('ASTAR_NO_PATH' if saw_astar_failure
                 else 'PARK_PATH_VALIDATION_BLOCKED'))

        if self.active_vehicle_spec is None:
            self.active_vehicle_spec = {
                'wheelbase': self.current_wheelbase,
                'vehicle_length_m': self.vehicle_length,
                'vehicle_width_m': self.vehicle_width,
            }
        try:
            self.registry.reserve_park(
                selected_slot.slot_id,
                self.mission_id,
                getattr(self, 'active_vehicle_number', ''),
                getattr(self, 'active_parking_credential', None),
            )
        except (KeyError, RegistryTransitionError) as exc:
            self.get_logger().error(f'park reservation failed: {exc}')
            return False
        self.active_destination_slot_id = selected_slot.slot_id
        self.destination_kind = 'PARKING_SLOT'
        self.active_parking_direction = selected_approach.parking_direction

        # Registry reservation이 성공한 뒤 path와 destination을 같은 stamp로 발행한다.
        path = Path()
        mission_stamp = self.get_clock().now().to_msg()
        path.header.stamp = mission_stamp
        path.header.frame_id = 'map'
        for wx, wy in waypoints:
            ps = PoseStamped()
            ps.header.stamp = mission_stamp
            ps.header.frame_id = 'map'
            ps.pose.position.x = wx
            ps.pose.position.y = wy
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        # 목표 주차칸 자세 발행 (FINAL_APPROACH 정밀정렬용)
        sp = PoseStamped()
        sp.header.stamp = mission_stamp
        sp.header.frame_id = 'map'
        sp.pose.position.x = selected_approach.target_pose.x_m
        sp.pose.position.y = selected_approach.target_pose.y_m
        slot_yaw = selected_approach.target_pose.yaw_rad
        sp.pose.orientation.z = math.sin(slot_yaw / 2.0)
        sp.pose.orientation.w = math.cos(slot_yaw / 2.0)
        try:
            self.pub_slot_pose.publish(sp)
            self.pub_waypoints.publish(path)
        except Exception as exc:
            self.registry.rollback_unpublished_park(
                selected_slot.slot_id, self.mission_id)
            self.active_destination_slot_id = ''
            self.destination_kind = ''
            self.active_parking_direction = ''
            self.get_logger().error(f'park plan publish failed: {exc}')
            return False
        self.path_published = True
        self.planning_blocker = None
        self.active_plan_stamp_ns = stamp_to_ns(mission_stamp)
        self.publish_state()

        self.get_logger().info(
            f'A* 경로 생성: start={start.position}, '
            f'footprint={self.loaded_footprint.length_m:.3f}x'
            f'{self.loaded_footprint.width_m:.3f}m, '
            f'{len(waypoints)}개 waypoint → stage='
            f'{selected_approach.staging_pose.position} → '
            f'슬롯 {selected_slot.slot_id} '
            f'({selected_approach.parking_direction}, '
            f'yaw={math.degrees(slot_yaw):.1f}deg, '
            f'clearance={selected_fit.length_clearance_m:.3f}/'
            f'{selected_fit.width_clearance_m:.3f}m)')
        return True

    def plan_retrieve_and_publish(self):
        if self.grid is None or not self.active_source_slot_id:
            return False
        try:
            record = self.registry.get(self.active_source_slot_id)
        except KeyError:
            return False
        if (record.lifecycle not in (
                SlotLifecycle.EXIT_RESERVED, SlotLifecycle.EXITING) or
                record.final_vehicle_pose is None or
                record.vehicle_spec is None):
            return False
        source_slot = self._slot_by_id(self.active_source_slot_id)
        if source_slot is None:
            return False

        self.active_vehicle_spec = dict(record.vehicle_spec)
        self._apply_active_vehicle_spec()
        final_pose = record.final_vehicle_pose
        extraction = make_extraction_geometry(
            source_slot, final_pose,
            self.loaded_footprint.length_m,
            self.slot_staging_gap,
            self.rigid_body_lookahead,
            self.slot_fit_long_margin)
        planning_grid = clear_source_vehicle(
            self.grid, self.grid_w, self.grid_h, self.resolution,
            final_pose,
            self.vehicle_length,
            self.vehicle_width,
            self.source_vehicle_fallback_mask,
            origin_x_m=getattr(self, 'grid_origin_x_m', 0.0),
            origin_y_m=getattr(self, 'grid_origin_y_m', 0.0))
        extraction_clear = corridor_is_free(
                planning_grid, self.grid_w, self.grid_h, self.resolution,
                final_pose.position, extraction.clear_pose.position,
                final_pose.yaw_rad,
                self.loaded_footprint.length_m,
                self.loaded_footprint.width_m,
                margin_m=max(
                    self.slot_fit_long_margin, self.slot_fit_lat_margin),
                unknown_is_occupied=self.unknown_is_occupied,
                origin_x_m=getattr(self, 'grid_origin_x_m', 0.0),
                origin_y_m=getattr(self, 'grid_origin_y_m', 0.0))
        if not self._validation_allows(
                extraction_clear,
                'RETRIEVE_EXTRACTION_CORRIDOR_BLOCKED',
                mission_phase='PLAN_PATH'):
            self.get_logger().error('retrieve extraction corridor blocked')
            return self._set_planning_blocker(
                'RETRIEVE_EXTRACTION_CORRIDOR_BLOCKED')

        waiting_pose = Pose2D(self.wait_x, self.wait_y, self.wait_yaw)
        waiting_staging = make_waiting_staging(
            waiting_pose,
            self.loaded_footprint.length_m,
            self.slot_staging_gap)
        waiting_rotation_clear = self._rotation_space_free(
            *waiting_staging.position)
        if not self._validation_allows(
                waiting_rotation_clear,
                'WAITING_ROTATION_SPACE_BLOCKED',
                mission_phase='PLAN_PATH'):
            self.get_logger().error('waiting staging rotation space blocked')
            return self._set_planning_blocker(
                'WAITING_ROTATION_SPACE_BLOCKED')
        waiting_insertion_clear = corridor_is_free(
                planning_grid, self.grid_w, self.grid_h, self.resolution,
                waiting_staging.position, waiting_pose.position,
                waiting_pose.yaw_rad,
                self.loaded_footprint.length_m,
                self.loaded_footprint.width_m,
                margin_m=max(
                    self.slot_fit_long_margin, self.slot_fit_lat_margin),
                unknown_is_occupied=self.unknown_is_occupied,
                origin_x_m=getattr(self, 'grid_origin_x_m', 0.0),
                origin_y_m=getattr(self, 'grid_origin_y_m', 0.0))
        if not self._validation_allows(
                waiting_insertion_clear,
                'WAITING_INSERTION_CORRIDOR_BLOCKED',
                mission_phase='PLAN_PATH'):
            self.get_logger().error('waiting insertion corridor blocked')
            return self._set_planning_blocker(
                'WAITING_INSERTION_CORRIDOR_BLOCKED')

        path_length, path_width = footprint_extents_in_slot_axes(
            self.loaded_footprint.length_m,
            self.loaded_footprint.width_m,
            final_pose.yaw_rad)
        self.planner.set_footprint(path_length / 2.0, path_width / 2.0)
        astar_path = self.planner.plan(
            planning_grid, self.grid_w, self.grid_h,
            extraction.clear_pose.position,
            waiting_staging.position)
        if astar_path is None:
            self.get_logger().error('retrieve clear-to-waiting A* failed')
            return self._set_planning_blocker('ASTAR_NO_PATH')

        waypoints = [
            final_pose.position,
            extraction.source_staging.position,
            extraction.clear_pose.position,
        ]
        # exact clear와 A* start-cell 중심이 다르면 짧은 연결 segment를 보존한다.
        # 같은 점일 때만 제거하여 discontinuity와 zero-length를 모두 피한다.
        for point in astar_path:
            if math.dist(waypoints[-1], point) > 1e-6:
                waypoints.append(point)
        if math.dist(waypoints[-1], waiting_staging.position) > 1e-6:
            waypoints.append(waiting_staging.position)

        path = Path()
        mission_stamp = self.get_clock().now().to_msg()
        path.header.stamp = mission_stamp
        path.header.frame_id = 'map'
        for wx, wy in waypoints:
            ps = PoseStamped()
            ps.header.stamp = mission_stamp
            ps.header.frame_id = 'map'
            ps.pose.position.x = wx
            ps.pose.position.y = wy
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        destination = PoseStamped()
        destination.header.stamp = mission_stamp
        destination.header.frame_id = 'map'
        destination.pose.position.x = waiting_pose.x_m
        destination.pose.position.y = waiting_pose.y_m
        destination.pose.orientation.z = math.sin(waiting_pose.yaw_rad / 2.0)
        destination.pose.orientation.w = math.cos(waiting_pose.yaw_rad / 2.0)
        try:
            # slot을 먼저 보내도 RigidBodySync가 stamp별 pending으로 보관한다.
            # path가 실패하면 운반 명령은 시작되지 않아 같은 mission이 재시도 가능하다.
            self.pub_slot_pose.publish(destination)
            self.pub_waypoints.publish(path)
        except Exception as exc:
            self.get_logger().error(f'retrieve plan publish failed: {exc}')
            return False
        self.path_published = True
        self.planning_blocker = None
        self.active_plan_stamp_ns = stamp_to_ns(mission_stamp)
        self.publish_state()
        self.get_logger().info(
            f'retrieve path: {self.active_source_slot_id} -> WAITING, '
            f'{len(waypoints)} waypoints')
        return True

    def publish_state(self):
        self.status_sequence += 1
        available_slot_ids = self._available_park_slot_ids()
        msg = String()
        msg.data = json.dumps({
            'state': self.state,
            'mission_id': self.mission_id,
            'mission_type': self.mission_type,
            'sequence': self.status_sequence,
            'stamp_ns': self.get_clock().now().nanoseconds,
            'empty_count': len(available_slot_ids),
            'available_slot_ids': available_slot_ids,
            'lifted': self.car_lifted,
            'has_map': self.grid is not None,
            'has_target': self.target_pose is not None,
            'vehicle_spec_ready': self._vehicle_spec_ready(),
            'require_ui_confirmation': self.require_ui_confirmation,
            'ui_approved': self.ui_park_approved,
            'ui_request_id': self.ui_request_id,
            'parking_slots': self.registry.summaries(('forward',)),
            'active_source_slot_id': self.active_source_slot_id,
            'active_destination_slot_id': self.active_destination_slot_id,
            'destination_kind': self.destination_kind,
            'plan_stamp_ns': self.active_plan_stamp_ns,
            'request_status': self.request_status,
            'planning_validation_mode': self.planning_validation_mode,
            'validation_warnings': list(self.validation_warnings),
            'planning_blocker': self.planning_blocker,
            'sync_fault': self.sync_fault,
            'last_completed': self.last_completed,
            'footprint_length_m': round(
                self.loaded_footprint.length_m, 4),
            'footprint_width_m': round(
                self.loaded_footprint.width_m, 4),
        })
        self.pub_state.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FleetManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
