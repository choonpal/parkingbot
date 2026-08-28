#!/usr/bin/env python3
"""
==================================================
cctv_merge_node (Jetson — 천장 CCTV 2대 병합)
==================================================
카메라별 ``yolo_bev_map_node`` sensor 인스턴스가 발행한 검출 envelope을
받아서 하나의 주차장 상태로 합치고, 기존 단일 카메라 시절과 **완전히 같은
이름의 하류 토픽**을 발행한다. 덕분에 fleet_manager_node, individual_move,
rigid_body_sync, UI는 코드를 한 줄도 바꾸지 않는다.

입력:
  /cctv0/detections (std_msgs/String, JSON) — cam0 sensor 인스턴스
  /cctv2/detections (std_msgs/String, JSON) — cam2 sensor 인스턴스
  /front/odom, /rear/odom (nav_msgs/Odometry) — 운반 차량 association, self-mask
  /mission/complete (std_msgs/String) — 타겟 latch 해제

출력 (단일 카메라 시절과 동일):
  /parking/map (nav_msgs/OccupancyGrid)
  /parking/target_pose (geometry_msgs/PoseStamped)
  /parking/empty_slots (geometry_msgs/PoseArray)
  /parking/vehicle_spec (std_msgs/String, TRANSIENT_LOCAL)
  /parking/vehicle_pose_feedback (geometry_msgs/PoseStamped)
  /parking/target_ready (std_msgs/Bool)
  /parking/target_status (std_msgs/String, JSON)
출력 (신규 진단):
  /cctv/merge_status (std_msgs/String, JSON)

왜 별도 노드인가
----------------
카메라 2대를 한 노드 안에서 처리하면 단일 카메라 경로와 코드가 뒤엉켜
"어느 쪽이 지금 도는 건지" 알 수 없게 된다. sensor(카메라별 인지) /
merge(통합 판단)로 나누면 카메라를 3대로 늘려도 launch만 늘리면 되고,
한 대가 죽어도 merge가 그 사실을 알고 해당 시야의 슬롯만 unknown 처리한다.
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, QoSProfile, ReliabilityPolicy,
)
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Bool, String

from cooperative_parking_robot.bev_fusion_core import (
    SlotOccupancyTracker,
    TargetLatchTracker,
    VehicleDimensionTracker,
    coverage_grid_values,
    decode_detection_envelope,
    merge_detections,
    perception_is_available,
    point_in_polygon,
    slot_observability,
    summarize_merge,
    target_presence_state,
)
from cooperative_parking_robot.parking_geometry import (
    parse_registered_slots,
    slot_polygon,
)
from cooperative_parking_robot.latest_qos import (
    SENSOR_LATEST_QOS,
    STATE_LATEST_QOS,
)
from cooperative_parking_robot.vision_utils import directed_axis_yaw

try:
    import cv2
    import numpy as np
    DEPS_OK = True
except ImportError:
    DEPS_OK = False


class CctvMergeNode(Node):
    def __init__(self):
        super().__init__('cctv_merge_node')

        # ===== 입력 구성 =====
        # detection_topics와 camera_ids는 같은 길이여야 한다. camera_ids는
        # envelope 안의 camera_id와 대조해 배선 실수를 초기에 잡는 용도다.
        self.declare_parameter(
            'detection_topics', ['/cctv0/detections', '/cctv2/detections'])
        self.declare_parameter('camera_ids', ['cam0', 'cam2'])
        # 이 시간 동안 새 envelope이 없으면 그 카메라는 죽은 것으로 본다.
        self.declare_parameter('camera_timeout_s', 1.0)
        self.declare_parameter('merge_rate_hz', 10.0)
        self.declare_parameter('require_all_cameras', True)

        # ===== 중복 제거 =====
        # 모형차 전장 0.9m 기준, 서로 다른 두 차량의 중심이 0.35m 안에 들어올
        # 수 없다. 반대로 같은 차량을 두 카메라가 보면 parallax 때문에 최대
        # 10cm 수준까지 벌어질 수 있으므로 gate는 그보다 넉넉해야 한다.
        self.declare_parameter('duplicate_center_gate_m', 0.35)
        self.declare_parameter('duplicate_overlap_ratio', 0.30)
        # 0.0이면 광축에 가까운 카메라 값을 그대로 채택(권장).
        self.declare_parameter('duplicate_center_blend', 0.0)

        # ===== 맵 =====
        self.declare_parameter('map_resolution', 0.05)
        self.declare_parameter('map_width_m', 4.40)
        self.declare_parameter('map_height_m', 3.83)
        self.declare_parameter('map_origin_x_m', 0.0)
        self.declare_parameter('map_origin_y_m', 0.0)
        self.declare_parameter('robot_odom_freshness_s', 0.5)
        # JSON keeps arbitrary polygon vertex counts representable as one ROS
        # parameter. Example: '[[[1,1],[2,1],[2,2],[1,2]]]'.
        self.declare_parameter('static_obstacle_polygons_json', '[]')
        self.declare_parameter('car_size_m', 0.90)
        self.declare_parameter('target_mask_radius_m', 0.30)
        self.declare_parameter('robot_mask_radius_m', 0.32)

        # ===== 슬롯/대기영역 (parking_layout.yaml과 동일 스키마) =====
        self.declare_parameter('layout_registered', False)
        self.declare_parameter('require_registered_layout', False)
        self.declare_parameter('slot_ids', ['P1', 'P2', 'P3', 'P4'])
        self.declare_parameter('slot_coords',
                               [1.5, 3.5, 2.5, 3.5, 3.5, 3.5, 4.5, 3.5])
        self.declare_parameter(
            'slot_sizes', [1.80, 0.70, 1.80, 0.70, 1.80, 0.70, 1.80, 0.70])
        self.declare_parameter('slot_yaws_deg', [90.0, 90.0, 90.0, 90.0])
        self.declare_parameter('slot_polygons', [0.0])
        self.declare_parameter(
            'waiting_polygon',
            [2.10, 0.30, 2.50, 0.30, 2.50, 0.90, 2.10, 0.90])
        self.declare_parameter('slot_occupancy_overlap_ratio', 0.10)
        self.declare_parameter('slot_empty_confirm_frames', 5)
        self.declare_parameter('slot_occupied_hold_s', 0.75)
        # True면 슬롯 네 모서리가 모두 한 카메라 안에 들어와야 판정한다.
        self.declare_parameter('require_full_slot_coverage', False)

        # ===== 타겟/제원 =====
        # 실측 YOLO 중심 노이즈(p90 약 2.3cm)보다 여유 있게 잡되, 움직이는
        # 차량은 즉시 READY를 해제한다. launch/YAML에서 현장별 조정 가능하다.
        self.declare_parameter('stationary_tolerance_m', 0.04)
        self.declare_parameter('stationary_hold_s', 2.0)
        self.declare_parameter('target_detection_timeout_s', 1.2)
        self.declare_parameter('target_presence_timeout_s', 1.2)
        self.declare_parameter('target_position_filter_window', 3)
        self.declare_parameter('vehicle_feedback_association_gate_m', 0.45)
        self.declare_parameter('use_fixed_wheelbase', True)
        self.declare_parameter('fixed_wheelbase_m', 0.785)
        self.declare_parameter('default_vehicle_length_m', 0.90)
        self.declare_parameter('default_vehicle_width_m', 0.35)
        self.declare_parameter('vehicle_dimension_padding_m', 0.03)
        self.declare_parameter('vehicle_length_range_m', [0.30, 6.50])
        self.declare_parameter('vehicle_width_range_m', [0.20, 2.80])
        self.declare_parameter('vehicle_dimension_ema_alpha', 0.20)
        self.declare_parameter('vehicle_spec_republish_s', 1.0)
        self.declare_parameter('yaw_ema_alpha', 0.15)
        self.declare_parameter('waiting_yaw_deg', 0.0)

        if not DEPS_OK:
            raise RuntimeError(
                'cctv_merge_node requires numpy and OpenCV (cv2)')

        self._read_parameters()
        self._build_state()
        self._setup_ros_interfaces()

        self.get_logger().info(
            'cctv_merge_node 시작 | '
            f'cameras={self.camera_ids} | topics={self.detection_topics} | '
            f'slots={[slot.slot_id for slot in self.slots]} | '
            f'rate={self.merge_rate_hz:.1f}Hz')

    # ------------------------------------------------------------------
    # 초기화
    # ------------------------------------------------------------------
    def _read_parameters(self):
        self.detection_topics = [
            str(value) for value in
            self.get_parameter('detection_topics').value]
        self.camera_ids = [
            str(value) for value in self.get_parameter('camera_ids').value]
        if not self.detection_topics:
            raise ValueError('detection_topics must not be empty')
        if len(self.detection_topics) != len(self.camera_ids):
            raise ValueError(
                'detection_topics and camera_ids must have the same length')
        if len(set(self.camera_ids)) != len(self.camera_ids):
            raise ValueError('camera_ids must be unique')
        if len(set(self.detection_topics)) != len(self.detection_topics):
            raise ValueError('detection_topics must be unique')

        self.camera_timeout_s = float(
            self.get_parameter('camera_timeout_s').value)
        self.merge_rate_hz = float(self.get_parameter('merge_rate_hz').value)
        self.require_all_cameras = bool(
            self.get_parameter('require_all_cameras').value)
        if self.camera_timeout_s <= 0.0 or self.merge_rate_hz <= 0.0:
            raise ValueError('camera_timeout_s and merge_rate_hz must be > 0')

        self.duplicate_center_gate = float(
            self.get_parameter('duplicate_center_gate_m').value)
        self.duplicate_overlap = float(
            self.get_parameter('duplicate_overlap_ratio').value)
        self.duplicate_blend = float(
            self.get_parameter('duplicate_center_blend').value)
        if self.duplicate_center_gate < 0.0:
            raise ValueError('duplicate_center_gate_m must be non-negative')
        if not 0.0 <= self.duplicate_overlap <= 1.0:
            raise ValueError('duplicate_overlap_ratio must be in [0,1]')
        if not 0.0 <= self.duplicate_blend <= 0.5:
            raise ValueError('duplicate_center_blend must be in [0,0.5]')

        self.resolution = float(self.get_parameter('map_resolution').value)
        self.map_w_m = float(self.get_parameter('map_width_m').value)
        self.map_h_m = float(self.get_parameter('map_height_m').value)
        if self.resolution <= 0.0 or self.map_w_m <= 0.0 or self.map_h_m <= 0.0:
            raise ValueError('map resolution/width/height must be positive')
        self.grid_w = int(math.ceil(self.map_w_m / self.resolution))
        self.grid_h = int(math.ceil(self.map_h_m / self.resolution))
        self.map_origin_x = float(
            self.get_parameter('map_origin_x_m').value)
        self.map_origin_y = float(
            self.get_parameter('map_origin_y_m').value)
        self.robot_odom_freshness = float(
            self.get_parameter('robot_odom_freshness_s').value)
        if self.robot_odom_freshness <= 0.0:
            raise ValueError('robot_odom_freshness_s must be positive')
        try:
            self.static_obstacle_polygons = json.loads(str(
                self.get_parameter('static_obstacle_polygons_json').value))
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            raise ValueError('invalid static_obstacle_polygons_json') from exc
        if not isinstance(self.static_obstacle_polygons, list):
            raise ValueError('static_obstacle_polygons_json must be a list')
        for polygon in self.static_obstacle_polygons:
            if (not isinstance(polygon, list) or len(polygon) < 3 or
                    any(not isinstance(point, list) or len(point) != 2
                        for point in polygon)):
                raise ValueError('each static obstacle needs at least 3 [x,y] points')
        self.car_size = float(self.get_parameter('car_size_m').value)
        self.target_mask_radius = float(
            self.get_parameter('target_mask_radius_m').value)
        self.robot_mask_radius = float(
            self.get_parameter('robot_mask_radius_m').value)
        if min(self.car_size, self.target_mask_radius,
               self.robot_mask_radius) <= 0.0:
            raise ValueError('mask radii and car_size_m must be positive')

        if (bool(self.get_parameter('require_registered_layout').value) and
                not bool(self.get_parameter('layout_registered').value)):
            raise RuntimeError(
                '현장 등록 layout이 아닙니다; '
                'bev_layout_calibration.launch.py를 먼저 실행하세요')

        self.slots = parse_registered_slots(
            self.get_parameter('slot_ids').value,
            self.get_parameter('slot_coords').value,
            self.get_parameter('slot_sizes').value,
            self.get_parameter('slot_yaws_deg').value)
        if not self.slots:
            raise ValueError('at least one registered parking slot is required')

        polygon_flat = list(self.get_parameter('slot_polygons').value)
        if len(polygon_flat) == 1 and float(polygon_flat[0]) == 0.0:
            polygons = [slot_polygon(slot) for slot in self.slots]
            self.get_logger().warn(
                'slot_polygons 미등록 — fitted rectangle 호환 폴백')
        elif len(polygon_flat) == 8 * len(self.slots):
            polygons = [
                tuple((float(polygon_flat[8 * i + 2 * c]),
                       float(polygon_flat[8 * i + 2 * c + 1]))
                      for c in range(4))
                for i in range(len(self.slots))
            ]
        else:
            raise ValueError('slot_polygons must contain eight values per slot')
        self.slot_polygons = {
            slot.slot_id: polygon
            for slot, polygon in zip(self.slots, polygons)
        }
        self.slot_by_id = {slot.slot_id: slot for slot in self.slots}

        waiting_flat = list(self.get_parameter('waiting_polygon').value)
        if len(waiting_flat) != 8:
            raise ValueError('waiting_polygon must contain four x,y points')
        self.waiting_polygon = [
            (float(waiting_flat[i]), float(waiting_flat[i + 1]))
            for i in range(0, 8, 2)]

        self.feedback_gate = float(
            self.get_parameter('vehicle_feedback_association_gate_m').value)
        if self.feedback_gate <= 0.0:
            raise ValueError(
                'vehicle_feedback_association_gate_m must be positive')
        self.use_fixed_wheelbase = bool(
            self.get_parameter('use_fixed_wheelbase').value)
        self.fixed_wheelbase = float(
            self.get_parameter('fixed_wheelbase_m').value)
        if self.fixed_wheelbase <= 0.0:
            raise ValueError('fixed_wheelbase_m must be positive')
        self.require_full_slot_coverage = bool(
            self.get_parameter('require_full_slot_coverage').value)
        self.target_presence_timeout_s = float(
            self.get_parameter('target_presence_timeout_s').value)
        self.target_position_filter_window = int(
            self.get_parameter('target_position_filter_window').value)
        self.waiting_yaw = math.radians(float(
            self.get_parameter('waiting_yaw_deg').value))
        if self.target_presence_timeout_s <= 0.0:
            raise ValueError('target_presence_timeout_s must be positive')
        if self.target_position_filter_window <= 0:
            raise ValueError('target_position_filter_window must be positive')

    def _build_state(self):
        now = time.monotonic()
        self.slot_tracker = SlotOccupancyTracker(
            [slot.slot_id for slot in self.slots],
            overlap_threshold=float(
                self.get_parameter('slot_occupancy_overlap_ratio').value),
            empty_confirm_frames=int(
                self.get_parameter('slot_empty_confirm_frames').value),
            occupied_hold_s=float(
                self.get_parameter('slot_occupied_hold_s').value),
            now=now)
        self.target_tracker = TargetLatchTracker(
            stationary_tolerance_m=float(
                self.get_parameter('stationary_tolerance_m').value),
            stationary_hold_s=float(
                self.get_parameter('stationary_hold_s').value),
            detection_timeout_s=float(
                self.get_parameter('target_detection_timeout_s').value),
            position_filter_window=self.target_position_filter_window)
        self.dimension_tracker = VehicleDimensionTracker(
            default_length_m=float(
                self.get_parameter('default_vehicle_length_m').value),
            default_width_m=float(
                self.get_parameter('default_vehicle_width_m').value),
            padding_m=float(
                self.get_parameter('vehicle_dimension_padding_m').value),
            length_range_m=list(
                self.get_parameter('vehicle_length_range_m').value),
            width_range_m=list(
                self.get_parameter('vehicle_width_range_m').value),
            dimension_alpha=float(
                self.get_parameter('vehicle_dimension_ema_alpha').value),
            yaw_alpha=float(self.get_parameter('yaw_ema_alpha').value))

        # 카메라별 최신 envelope. 값이 None이면 아직 한 번도 못 받은 상태다.
        self.latest = {camera_id: None for camera_id in self.camera_ids}
        self.latest_wall = {camera_id: 0.0 for camera_id in self.camera_ids}
        self.robot_pose = {'front': None, 'rear': None}
        self.spec_sent = False
        self.spec_last_publish_wall = None
        self.spec_republish_s = float(
            self.get_parameter('vehicle_spec_republish_s').value)
        if self.spec_republish_s <= 0.0:
            raise ValueError('vehicle_spec_republish_s must be positive')
        self.latest_vehicle_class = 'default'
        self.latest_classified_wheelbase = self.fixed_wheelbase
        self._last_target_ready = None
        self._last_target_state = None
        self._camera_alive = {camera_id: None for camera_id in self.camera_ids}
        self._perception_available = None
        self._target_last_observed_wall = 0.0
        self.fleet_state = 'UNKNOWN'
        self.vehicle_lifted = False

    def _setup_ros_interfaces(self):
        for camera_id, topic in zip(self.camera_ids, self.detection_topics):
            self.create_subscription(
                String, topic,
                lambda msg, c=camera_id: self.detection_cb(c, msg),
                SENSOR_LATEST_QOS)
        for role in ('front', 'rear'):
            self.create_subscription(
                Odometry, f'/{role}/odom',
                lambda msg, r=role: self.odom_cb(r, msg),
                SENSOR_LATEST_QOS)
        self.create_subscription(
            String, '/mission/complete', self.mission_complete_cb, 10)
        self.create_subscription(
            String, '/fleet/state', self.fleet_state_cb, STATE_LATEST_QOS)
        self.create_subscription(
            Bool, '/robot/lifted', self.lifted_cb, 10)

        mission_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_map = self.create_publisher(
            OccupancyGrid, '/parking/map', 10)
        self.pub_target = self.create_publisher(
            PoseStamped, '/parking/target_pose', 10)
        self.pub_empty = self.create_publisher(
            PoseArray, '/parking/empty_slots', 10)
        self.pub_spec = self.create_publisher(
            String, '/parking/vehicle_spec', mission_qos)
        self.pub_vehicle_fb = self.create_publisher(
            PoseStamped, '/parking/vehicle_pose_feedback',
            SENSOR_LATEST_QOS)
        self.pub_target_ready = self.create_publisher(
            Bool, '/parking/target_ready', 10)
        self.pub_target_status = self.create_publisher(
            String, '/parking/target_status', 10)
        self.pub_status = self.create_publisher(
            String, '/cctv/merge_status', 10)

        self.create_timer(1.0 / self.merge_rate_hz, self.merge_cycle)

    # ------------------------------------------------------------------
    # 콜백
    # ------------------------------------------------------------------
    def detection_cb(self, camera_id, msg):
        try:
            envelope = decode_detection_envelope(msg.data)
        except ValueError as exc:
            self.get_logger().warn(
                f'[{camera_id}] detection envelope 해석 실패: {exc}',
                throttle_duration_sec=5.0)
            return
        if envelope['camera_id'] != camera_id:
            # 토픽 remap을 잘못해 다른 카메라 데이터가 들어오면 좌표계가
            # 조용히 섞인다. 절대 받아들이지 않는다.
            self.get_logger().error(
                f'[{camera_id}] envelope camera_id 불일치: '
                f"{envelope['camera_id']} — 배선/remap 확인 필요",
                throttle_duration_sec=5.0)
            return
        if not envelope['homography_ok']:
            self.get_logger().error(
                f'[{camera_id}] homography 미로드 상태의 envelope — 폐기',
                throttle_duration_sec=5.0)
            return
        self.latest[camera_id] = envelope
        self.latest_wall[camera_id] = time.monotonic()

    def odom_cb(self, role, msg):
        self.robot_pose[role] = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y), time.monotonic())

    def fleet_state_cb(self, msg):
        try:
            payload = json.loads(msg.data)
            self.fleet_state = str(payload['state'])
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            self.get_logger().warn(
                'invalid fleet/state envelope', throttle_duration_sec=5.0)

    def lifted_cb(self, msg):
        self.vehicle_lifted = bool(msg.data)

    def mission_complete_cb(self, msg):
        """임무 종료 — 타겟 latch와 제원 캐시를 해제한다.

        단일 카메라 노드(yolo_bev_map_node.mission_complete_cb)와 동일한
        envelope 검증을 그대로 쓴다.
        """
        try:
            payload = json.loads(msg.data)
            mission_id = str(payload['mission_id'])
            stamp_ns = int(payload['stamp_ns'])
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            self.get_logger().warn('invalid mission/complete envelope')
            return
        age_s = (self.get_clock().now().nanoseconds - stamp_ns) * 1e-9
        if not -0.5 <= age_s <= 10.0:
            return
        if self.target_tracker.latched is None:
            return
        self.target_tracker.reset()
        self.dimension_tracker.reset()
        self.spec_sent = False
        self.spec_last_publish_wall = None
        self.latest_vehicle_class = 'default'
        self.latest_classified_wheelbase = self.fixed_wheelbase
        self.pub_target_ready.publish(Bool(data=False))
        self._target_last_observed_wall = 0.0
        self._publish_target_status(
            time.monotonic(), current_visible=False, ready=False,
            reason='MISSION_COMPLETE')
        self.get_logger().info(f'임무 {mission_id} 완료 — 타겟 latch 해제')

    # ------------------------------------------------------------------
    # 메인 병합 사이클
    # ------------------------------------------------------------------
    def merge_cycle(self):
        now = time.monotonic()
        alive_envelopes = {}
        camera_states = {}
        for camera_id in self.camera_ids:
            envelope = self.latest.get(camera_id)
            age = now - self.latest_wall.get(camera_id, 0.0)
            alive = envelope is not None and age <= self.camera_timeout_s
            camera_states[camera_id] = {
                'alive': alive,
                'age_s': age if envelope is not None else float('inf'),
                'detections': 0 if envelope is None else len(
                    envelope['detections']),
                'coverage_ready': (
                    envelope is not None and
                    envelope['coverage_polygon'] is not None),
            }
            if alive:
                alive_envelopes[camera_id] = envelope

        perception_available = perception_is_available(
            camera_states, self.require_all_cameras)
        self._log_perception_health(camera_states, perception_available)

        if not alive_envelopes:
            self.get_logger().error(
                '살아있는 CCTV sensor 노드가 없습니다 — 맵/빈자리 발행 중단',
                throttle_duration_sec=5.0)
            self._publish_fail_closed(camera_states)
            return
        if self.require_all_cameras and len(alive_envelopes) < len(
                self.camera_ids):
            missing = [c for c in self.camera_ids if c not in alive_envelopes]
            self.get_logger().error(
                f'require_all_cameras=true인데 {missing} 미수신 — 발행 중단',
                throttle_duration_sec=5.0)
            self._publish_fail_closed(camera_states)
            return

        # 1) 검출 병합
        all_detections = []
        coverage_polygons = {}
        newest_stamp_ns = 0
        for camera_id, envelope in alive_envelopes.items():
            all_detections.extend(envelope['detections'])
            coverage_polygons[camera_id] = envelope['coverage_polygon']
            newest_stamp_ns = max(newest_stamp_ns, envelope['stamp_ns'])
            for detection in envelope['detections']:
                if detection.vehicle_class:
                    self.latest_vehicle_class = detection.vehicle_class
                if detection.classified_wheelbase_m:
                    self.latest_classified_wheelbase = (
                        detection.classified_wheelbase_m)
        merged = merge_detections(
            all_detections,
            duplicate_center_gate_m=self.duplicate_center_gate,
            duplicate_overlap_ratio=self.duplicate_overlap,
            center_blend=self.duplicate_blend)

        # 2) 타겟(대기영역 차량) 확정
        #    sensor 노드가 이미 in_waiting을 계산해 오지만, 병합 후 위치가
        #    조금 바뀔 수 있으므로 최종 좌표로 한 번 더 확인한다.
        target_detection = None
        for detection in merged:
            if detection.in_waiting or point_in_polygon(
                    detection.center[0], detection.center[1],
                    self.waiting_polygon):
                target_detection = detection
                break
        if target_detection is not None:
            self._target_last_observed_wall = now
            self.dimension_tracker.update_yaw(target_detection.yaw)
            self.dimension_tracker.update_dimensions(
                target_detection.length_m, target_detection.width_m)
        else:
            # 타겟이 사라지고 timeout이 지나면 치수/각도 캐시도 초기화한다.
            if (self.target_tracker.latched is None and
                    now - self.target_tracker.last_seen >
                    self.target_tracker.timeout_s):
                self.dimension_tracker.reset()

        mission_active = (
            self.fleet_state in ('PLAN_PATH', 'NAVIGATING') or
            (self.fleet_state == 'WAIT_LIFT' and self.vehicle_lifted))
        latched = self.target_tracker.update(
            None if target_detection is None else target_detection.center,
            now,
            preserve_latched=mission_active)
        if self.target_tracker.just_latched:
            self.get_logger().info(
                '타겟 정차 확인 — target_ready latch (merge)')
            self.pub_target_ready.publish(Bool(data=True))
        if latched is not None:
            self._publish_target(latched)
            if (not self.spec_sent or self.spec_last_publish_wall is None or
                    now - self.spec_last_publish_wall >=
                    self.spec_republish_s):
                self._publish_vehicle_spec()

        # 3) 슬롯 점유 — 관측 자격이 있는 카메라가 있는 슬롯만 판정
        observable = slot_observability(
            self.slot_polygons, coverage_polygons,
            require_full_slot=self.require_full_slot_coverage)
        slot_cars = [
            detection for detection in merged
            if not (detection.in_waiting or point_in_polygon(
                detection.center[0], detection.center[1],
                self.waiting_polygon))
        ]
        slot_state = self.slot_tracker.update(
            self.slot_polygons, slot_cars, observable, now)
        self._publish_empty_slots()

        # 4) 운반 중 차량 피드백
        if target_detection is None:
            self._publish_vehicle_feedback(slot_cars, newest_stamp_ns)

        # 5) 맵
        self._publish_map(merged, latched, coverage_polygons)

        # 6) 진단
        self._publish_status(camera_states, merged, newest_stamp_ns, slot_state)

        ready = latched is not None
        if ready != self._last_target_ready:
            self._last_target_ready = ready
        self.pub_target_ready.publish(Bool(data=ready))
        self._publish_target_status(
            now,
            current_visible=target_detection is not None,
            ready=ready,
        )

    # ------------------------------------------------------------------
    # 발행 helper
    # ------------------------------------------------------------------
    def _log_perception_health(self, camera_states, available):
        for camera_id, state in camera_states.items():
            alive = bool(state['alive'])
            previous = self._camera_alive[camera_id]
            if (previous is None and not alive) or (
                    previous is not None and alive != previous):
                if alive:
                    self.get_logger().info(
                        f'PERCEPTION_RECOVERED camera={camera_id}')
                else:
                    self.get_logger().warn(
                        'PERCEPTION_UNAVAILABLE '
                        f'camera={camera_id} age_s={state["age_s"]:.2f} '
                        f'timeout_s={self.camera_timeout_s:.2f}')
            self._camera_alive[camera_id] = alive
        if ((self._perception_available is None and not available) or
                (self._perception_available is not None and
                 available != self._perception_available)):
            self.get_logger().info(
                'PERCEPTION_AVAILABLE — detection 병합 재개'
                if available else
                'PERCEPTION_UNAVAILABLE — 안전 fail-close 진입')
        self._perception_available = available

    def _publish_fail_closed(self, camera_states):
        self.target_tracker.reset()
        self.dimension_tracker.reset()
        self.spec_sent = False
        self.spec_last_publish_wall = None
        self.latest_vehicle_class = 'default'
        self.latest_classified_wheelbase = self.fixed_wheelbase
        self.pub_target_ready.publish(Bool(data=False))
        self._last_target_ready = False
        self._target_last_observed_wall = 0.0
        self._publish_target_status(
            time.monotonic(), current_visible=False, ready=False,
            perception_available=False,
            reason='PERCEPTION_UNAVAILABLE')
        self._publish_empty_slots(force_empty=True)
        self._publish_map([], None, {})
        self._publish_status(camera_states, [], 0)

    def _publish_target(self, center):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(center[0])
        msg.pose.position.y = float(center[1])
        half_yaw = directed_axis_yaw(
            self.dimension_tracker.yaw, self.waiting_yaw) / 2.0
        msg.pose.orientation.z = math.sin(half_yaw)
        msg.pose.orientation.w = math.cos(half_yaw)
        self.pub_target.publish(msg)

    def _publish_target_status(
            self, now, current_visible, ready, perception_available=True,
            reason=''):
        """Publish customer-facing presence separately from the safety gate."""
        if self._target_last_observed_wall > 0.0:
            last_seen_age_s = max(
                0.0, float(now) - self._target_last_observed_wall)
            observed_recently = (
                last_seen_age_s <= self.target_presence_timeout_s)
        else:
            last_seen_age_s = None
            observed_recently = False
        state = target_presence_state(
            ready, observed_recently,
            perception_available=perception_available)
        if state != self._last_target_state:
            if state == 'ABSENT' and not reason:
                self.get_logger().info(
                    'TARGET_ABSENT — perception 정상, 대기영역 차량 미검출')
            self._last_target_state = state
        stable_for_s = 0.0
        if self.target_tracker.stable_since is not None:
            stable_for_s = max(
                0.0, float(now) - self.target_tracker.stable_since)
        payload = {
            'version': 1,
            'state': state,
            'visible': bool(current_visible),
            'observed_recently': bool(observed_recently),
            'ready': bool(ready),
            'stable_for_s': round(stable_for_s, 3),
            'last_seen_age_s': (
                None if last_seen_age_s is None
                else round(last_seen_age_s, 3)),
            'stationary_tolerance_m': self.target_tracker.tolerance,
            'stationary_hold_s': self.target_tracker.hold_s,
            'detection_timeout_s': self.target_tracker.timeout_s,
        }
        if reason:
            payload['reason'] = str(reason)
        self.pub_target_status.publish(String(
            data=json.dumps(payload, ensure_ascii=False)))

    def _publish_vehicle_spec(self):
        if not self.dimension_tracker.dimension_valid:
            return False
        first_publish = not self.spec_sent
        wheelbase = (
            self.fixed_wheelbase if self.use_fixed_wheelbase
            else self.latest_classified_wheelbase)
        msg = String()
        msg.data = json.dumps({
            'class': self.latest_vehicle_class,
            'wheelbase': wheelbase,
            'wheelbase_mode': (
                'fixed' if self.use_fixed_wheelbase else 'classified'),
            'vehicle_length_m': self.dimension_tracker.length_m,
            'vehicle_width_m': self.dimension_tracker.width_m,
            'dimension_source': 'segmentation_mask',
            'dimension_valid': True,
            'sequence': 1,
            'stamp_ns': self.get_clock().now().nanoseconds,
        })
        self.pub_spec.publish(msg)
        self.spec_sent = True
        self.spec_last_publish_wall = time.monotonic()
        if first_publish:
            self.get_logger().info(
                f'차종={self.latest_vehicle_class}, '
                f'wheelbase={wheelbase:.3f}m, '
                f'size={self.dimension_tracker.length_m:.3f}x'
                f'{self.dimension_tracker.width_m:.3f}m (merge)')
        return True

    def _publish_empty_slots(self, force_empty=False):
        pa = PoseArray()
        pa.header.stamp = self.get_clock().now().to_msg()
        pa.header.frame_id = 'map'
        slot_ids = [] if force_empty else self.slot_tracker.empty_slot_ids()
        for slot_id in slot_ids:
            slot = self.slot_by_id[slot_id]
            pose = Pose()
            pose.position.x = slot.center_x_m
            pose.position.y = slot.center_y_m
            pose.orientation.z = math.sin(slot.entry_yaw_rad / 2.0)
            pose.orientation.w = math.cos(slot.entry_yaw_rad / 2.0)
            pa.poses.append(pose)
        self.pub_empty.publish(pa)

    def _publish_vehicle_feedback(self, cars, stamp_ns):
        front = self.robot_pose.get('front')
        rear = self.robot_pose.get('rear')
        if not cars or front is None or rear is None:
            return
        predicted = ((front[0] + rear[0]) / 2.0, (front[1] + rear[1]) / 2.0)
        candidate = min(cars, key=lambda car: math.hypot(
            car.center[0] - predicted[0], car.center[1] - predicted[1]))
        distance = math.hypot(
            candidate.center[0] - predicted[0],
            candidate.center[1] - predicted[1])
        if distance > self.feedback_gate:
            self.get_logger().warn(
                f'운반 차량 feedback association gate 초과: {distance:.3f}m',
                throttle_duration_sec=2.0)
            return
        msg = PoseStamped()
        # 처리 지연을 숨기지 않도록 CCTV 촬영시각을 그대로 전파한다.
        if stamp_ns > 0:
            msg.header.stamp.sec = int(stamp_ns // 1_000_000_000)
            msg.header.stamp.nanosec = int(stamp_ns % 1_000_000_000)
        else:
            msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = candidate.center[0]
        msg.pose.position.y = candidate.center[1]
        if candidate.yaw is None:
            msg.pose.orientation.w = 1.0
        else:
            msg.pose.orientation.z = math.sin(candidate.yaw / 2.0)
            msg.pose.orientation.w = math.cos(candidate.yaw / 2.0)
        self.pub_vehicle_fb.publish(msg)

    def _publish_map(self, merged, latched, coverage_polygons):
        grid = np.asarray(coverage_grid_values(
            self.grid_w, self.grid_h, self.resolution,
            coverage_polygons, self.map_origin_x, self.map_origin_y),
            dtype=np.int8).reshape(
                (self.grid_h, self.grid_w))
        static_mask_u8 = np.zeros(
            (self.grid_h, self.grid_w), dtype=np.uint8)
        for polygon in self.static_obstacle_polygons:
            contour = np.asarray([[
                int(math.floor((float(x) - self.map_origin_x) /
                               self.resolution)),
                int(math.floor((float(y) - self.map_origin_y) /
                               self.resolution))]
                for x, y in polygon], dtype=np.int32)
            cv2.fillPoly(static_mask_u8, [contour], 1)
        static_mask = static_mask_u8.astype(bool)
        grid[static_mask] = 100
        car_px = max(1, int(math.ceil(self.car_size / self.resolution)))
        for detection in merged:
            # 운반 대상은 A* 장애물에서 제거해 시작점이 막히지 않게 한다.
            if latched is not None and math.hypot(
                    detection.center[0] - latched[0],
                    detection.center[1] - latched[1]
            ) <= self.target_mask_radius:
                continue
            if detection.polygon is not None and len(detection.polygon) >= 3:
                contour = np.asarray([
                    [int(round((x - self.map_origin_x) / self.resolution)),
                     int(round((y - self.map_origin_y) / self.resolution))]
                    for x, y in detection.polygon
                ], dtype=np.int32)
                cv2.fillPoly(grid, [contour], 100)
                continue
            gx = int(math.floor(
                (detection.center[0] - self.map_origin_x) / self.resolution))
            gy = int(math.floor(
                (detection.center[1] - self.map_origin_y) / self.resolution))
            half = car_px // 2
            y1 = max(0, gy - half)
            y2 = min(self.grid_h, gy + half)
            x1 = max(0, gx - half)
            x2 = min(self.grid_w, gx + half)
            if x1 < x2 and y1 < y2:
                grid[y1:y2, x1:x2] = 100

        # YOLO가 로봇을 차량으로 오검출해도 A* 시작점을 막지 않도록 self-mask.
        mask_cells = max(1, int(self.robot_mask_radius / self.resolution))
        now = time.monotonic()
        for pose in self.robot_pose.values():
            if pose is None or now - pose[2] > self.robot_odom_freshness:
                continue
            gx = int(math.floor(
                (pose[0] - self.map_origin_x) / self.resolution))
            gy = int(math.floor(
                (pose[1] - self.map_origin_y) / self.resolution))
            y1 = max(0, gy - mask_cells)
            y2 = min(self.grid_h, gy + mask_cells + 1)
            x1 = max(0, gx - mask_cells)
            x2 = min(self.grid_w, gx + mask_cells + 1)
            if x1 >= x2 or y1 >= y2:
                continue
            region = grid[y1:y2, x1:x2]
            region_static = static_mask[y1:y2, x1:x2]
            region[(region >= 0) & ~region_static] = 0

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = self.resolution
        msg.info.width = self.grid_w
        msg.info.height = self.grid_h
        msg.info.origin.position.x = self.map_origin_x
        msg.info.origin.position.y = self.map_origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.flatten().tolist()
        self.pub_map.publish(msg)

    def _publish_status(self, camera_states, merged, stamp_ns, slot_state=None):
        message = String()
        message.data = summarize_merge(
            camera_states, merged,
            self.slot_tracker.state if slot_state is None else slot_state,
            stamp_ns)
        self.pub_status.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = CctvMergeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
