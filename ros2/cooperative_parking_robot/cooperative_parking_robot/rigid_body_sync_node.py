#!/usr/bin/env python3
"""
두 로봇을 하나의 강체(base_virtual)로 제어하는 Master 노드.

핵심 원칙
- 메카넘 평행이동을 사용하므로 목표가 뒤면 후진하고 옆이면 횡이동한다.
- 목표 방향을 보기 위해 차량을 돌리지 않는다. 경로 수신 시 yaw를 유지한다.
- 일반 경로와 FINAL_APPROACH 모두 같은 상대거리/yaw 융합·PID·fail-safe를 거친다.
- ArUco raw 거리는 camera->marker 거리이므로 실측 offset 없이는 중심거리 보정에
  사용하지 않는다. offset 실측 전에도 ArUco 상대 yaw는 계속 사용한다.
"""

import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, QoSProfile, ReliabilityPolicy,
)
from std_msgs.msg import Bool, String

from cooperative_parking_robot.fault_policy import classify_fault

from cooperative_parking_robot.command_qos import CMD_VEL_QOS
from cooperative_parking_robot.latest_qos import (
    SENSOR_LATEST_QOS,
    STATE_LATEST_QOS,
)
from cooperative_parking_robot.kalman_filter import ScalarKalman
from cooperative_parking_robot.freshness import StampGate, stamp_to_ns
from cooperative_parking_robot.mission_protocol import make_arrival_status
from cooperative_parking_robot.pid_controller import PID
from cooperative_parking_robot.pure_pursuit import PurePursuit
from cooperative_parking_robot.rigid_body_kinematics import RigidBodyKinematics
from cooperative_parking_robot.vehicle_entry import (
    DEFAULT_WHEELBASE_M, MIN_INTER_ROBOT_GAP_M, ROBOT_LENGTH_M,
    validate_wheelbase_clearance,
)


class RigidBodySyncNode(Node):
    def __init__(self):
        super().__init__('rigid_body_sync_node')

        # ===== 기본/주행 =====
        self.declare_parameter('wheelbase', DEFAULT_WHEELBASE_M)
        self.declare_parameter('max_speed', 0.08)
        self.declare_parameter('max_omega', 0.30)
        self.declare_parameter('lookahead', 0.15)
        self.declare_parameter('path_goal_tolerance', 0.01)
        self.declare_parameter('hold_initial_yaw', True)
        self.declare_parameter('yaw_hold_kp', 1.0)
        self.declare_parameter('use_vehicle_spec_wheelbase', True)

        # ===== 상대동기/안전 =====
        self.declare_parameter('dist_error_limit', 0.03)       # 감속 시작
        self.declare_parameter('dist_stop_limit', 0.08)        # 즉시 정지
        self.declare_parameter('dist_error_timeout_s', 2.0)    # 감속오차 지속 정지
        self.declare_parameter('yaw_error_limit', 0.15)
        self.declare_parameter('odom_timeout_s', 0.50)
        self.declare_parameter('marker_slowdown_s', 1.0)
        self.declare_parameter('marker_stop_s', 2.0)

        # Calibration is supplied by config/id0_calibration.yaml. A direct
        # node run starts distance fusion disabled instead of guessing from
        # the robot envelope length.
        self.declare_parameter('aruco_distance_offset_m', 0.0)
        self.declare_parameter('use_aruco_distance', False)
        self.declare_parameter('aruco_min_distance_m', 0.05)
        self.declare_parameter('aruco_max_distance_m', 1.50)
        self.declare_parameter('aruco_timeout_s', 0.30)
        self.declare_parameter('cctv_marker_timeout_s', 0.50)
        self.declare_parameter('robot_length_m', ROBOT_LENGTH_M)
        self.declare_parameter(
            'minimum_inter_robot_gap_m', MIN_INTER_ROBOT_GAP_M)

        # CCTV 차량 중심 보정은 top-marker odom과 중복될 수 있으므로 outlier gate 사용
        self.declare_parameter('cctv_feedback_gate_m', 0.25)
        self.declare_parameter('cctv_offset_alpha', 0.30)
        self.declare_parameter('cctv_feedback_timeout_s', 0.50)
        self.declare_parameter('path_timeout_s', 5.0)
        self.declare_parameter('target_timeout_s', 2.0)
        self.declare_parameter('slot_timeout_s', 5.0)
        self.declare_parameter('slot_pose_wait_timeout_s', 1.0)
        self.declare_parameter('future_tolerance_s', 0.10)
        # 로봇 두 대의 기하학적 중점과 실제 차량 중심이 다를 수 있으므로,
        # 인양 직후 YOLO가 고정한 target_pose로 초기 오프셋을 잡는다.
        self.declare_parameter('initialize_offset_from_target_pose', True)
        self.declare_parameter('initial_target_offset_gate_m', 0.50)

        # ===== 최종 접근 =====
        # A*의 마지막 점은 슬롯 중심이 아니라 슬롯 밖 staging point다.
        # 이 점에 충분히 도착한 뒤에만 회전/삽입 단계로 넘어간다.
        self.declare_parameter('final_approach_dist', 0.02)
        self.declare_parameter('final_pos_tol', 0.02)
        self.declare_parameter('final_yaw_tol', 0.052)  # 3 deg
        self.declare_parameter('final_lateral_tol', 0.01)
        self.declare_parameter('final_speed_ratio', 0.30)
        self.declare_parameter('align_to_slot_yaw', True)

        gp = self.get_parameter
        self.wheelbase = float(gp('wheelbase').value)
        self.max_speed = float(gp('max_speed').value)
        self.max_omega = float(gp('max_omega').value)
        self.path_goal_tolerance = float(
            gp('path_goal_tolerance').value)
        self.hold_initial_yaw = bool(gp('hold_initial_yaw').value)
        self.yaw_hold_kp = float(gp('yaw_hold_kp').value)
        self.use_vehicle_spec_wheelbase = bool(
            gp('use_vehicle_spec_wheelbase').value)

        self.dist_limit = float(gp('dist_error_limit').value)
        self.dist_stop_limit = float(gp('dist_stop_limit').value)
        self.dist_error_timeout = float(gp('dist_error_timeout_s').value)
        self.yaw_limit = float(gp('yaw_error_limit').value)
        self.odom_timeout = float(gp('odom_timeout_s').value)
        self.marker_slowdown = float(gp('marker_slowdown_s').value)
        self.marker_stop = float(gp('marker_stop_s').value)

        self.aruco_distance_offset = float(gp('aruco_distance_offset_m').value)
        self.use_aruco_distance = bool(gp('use_aruco_distance').value)
        if self.use_aruco_distance and self.aruco_distance_offset <= 0.0:
            raise ValueError(
                'use_aruco_distance=true requires ID0 calibration')
        self.aruco_min_distance = float(gp('aruco_min_distance_m').value)
        self.aruco_max_distance = float(gp('aruco_max_distance_m').value)
        self.aruco_timeout = float(gp('aruco_timeout_s').value)
        self.cctv_marker_timeout = float(gp('cctv_marker_timeout_s').value)
        self.robot_length = float(gp('robot_length_m').value)
        self.minimum_inter_robot_gap = float(
            gp('minimum_inter_robot_gap_m').value)

        self.cctv_feedback_gate = float(gp('cctv_feedback_gate_m').value)
        self.cctv_offset_alpha = float(gp('cctv_offset_alpha').value)
        self.cctv_feedback_timeout = float(
            gp('cctv_feedback_timeout_s').value)
        self.path_timeout = float(gp('path_timeout_s').value)
        self.target_timeout = float(gp('target_timeout_s').value)
        self.slot_timeout = float(gp('slot_timeout_s').value)
        self.slot_pose_wait_timeout = float(
            gp('slot_pose_wait_timeout_s').value)
        self.future_tolerance = float(gp('future_tolerance_s').value)
        self.initialize_offset_from_target = bool(
            gp('initialize_offset_from_target_pose').value)
        self.initial_target_offset_gate = float(
            gp('initial_target_offset_gate_m').value)
        self.final_speed_ratio = float(gp('final_speed_ratio').value)
        self.align_to_slot_yaw = bool(gp('align_to_slot_yaw').value)
        self.final_approach_dist = float(gp('final_approach_dist').value)
        self.final_pos_tol = float(gp('final_pos_tol').value)
        self.final_yaw_tol = float(gp('final_yaw_tol').value)
        self.final_lateral_tol = float(gp('final_lateral_tol').value)

        self._validate_parameters()
        self.stamp_gates = {
            'path': StampGate(self.path_timeout, self.future_tolerance),
            'front_odom': StampGate(self.odom_timeout, self.future_tolerance),
            'rear_odom': StampGate(self.odom_timeout, self.future_tolerance),
            'aruco': StampGate(self.aruco_timeout, self.future_tolerance),
            'cctv_feedback': StampGate(
                self.cctv_feedback_timeout, self.future_tolerance),
            'target': StampGate(self.target_timeout, self.future_tolerance),
            'slot': StampGate(self.slot_timeout, self.future_tolerance),
            'vehicle_spec': StampGate(
                self.slot_timeout, self.future_tolerance),
        }

        # fleet_manager가 한 번 발행한 경로/슬롯을 늦게 연결된 Master도
        # 수신하도록 publisher와 동일한 transient-local QoS를 사용한다.
        self.mission_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # ===== 모듈 =====
        self.kinematics = RigidBodyKinematics(self.wheelbase)
        self.pursuit = PurePursuit(
            lookahead=float(gp('lookahead').value),
            max_speed=self.max_speed,
            max_omega=self.max_omega,
            goal_tolerance=self.path_goal_tolerance,
            rotate_to_path=not self.hold_initial_yaw,
        )
        self.dist_kalman = ScalarKalman(init=self.wheelbase)
        self.yaw_kalman = ScalarKalman(init=0.0, R=0.01)
        self.dist_pid = PID(1.2, 0.1, 0.05, out_limit=self.max_speed)
        self.yaw_pid = PID(1.0, 0.05, 0.03, out_limit=0.20)

        # ===== 상태 =====
        self.front = {'x': 0.0, 'y': 0.0, 'theta': 0.0, 't': 0.0}
        self.rear = {'x': 0.0, 'y': 0.0, 'theta': 0.0, 't': 0.0}
        self.front_ready = False
        self.rear_ready = False

        self.aruco_raw_dist = None
        self.aruco_dist = None
        self.aruco_yaw = None
        self.aruco_receipt_time = None  # local monotonic: host clock skew 영향 없음
        self.marker_visible = False
        self.marker_lost_since = None
        self.front_top_marker_visible = False
        self.rear_top_marker_visible = False
        self.front_top_marker_time = None
        self.rear_top_marker_time = None
        self.dist_error_since = None

        self.has_path = False
        # transient-local 경로를 재수신해도 차량을 든 DRIVE 상태가 아니면
        # 절대 구동하지 않는다. 노드 재시작/오래된 임무 replay 방지용 gate.
        self.vehicle_lifted = False
        self.front_robot_state = 'IDLE'
        self.rear_robot_state = 'IDLE'
        self.estop = False
        self.yaw_reference = None
        self.final_mode = False
        self.slot_pose = None
        self.path_mission_stamp_ns = None
        self.slot_mission_stamp_ns = None
        self.pending_slot_pose = None
        self.pending_slot_mission_stamp_ns = None
        self.slot_pose_missing_since = None
        self.target_pose = None
        self.target_offset_initialized = False
        self.sync_filters_initialized = False

        # 차량 중심이 Front/Rear 중점에서 떨어진 양을 body frame에
        # 저장한다. world frame 고정 벡터로 두면 회전 후 중심이 틀린다.
        self.vehicle_offset_body = [0.0, 0.0]
        self.cctv_time = 0.0

        self._err = 'OK'
        self._info = {}

        # ===== 구독 =====
        self.create_subscription(
            Path, '/virtual_robot/waypoints', self.path_cb, self.mission_qos)
        self.create_subscription(
            Odometry, '/front/odom', self.front_cb, SENSOR_LATEST_QOS)
        self.create_subscription(
            Odometry, '/rear/odom', self.rear_cb, SENSOR_LATEST_QOS)
        self.create_subscription(
            PoseStamped, '/sync/relative_pose', self.aruco_cb,
            SENSOR_LATEST_QOS)
        self.create_subscription(
            Bool, '/sync/marker_visible', self.marker_cb,
            SENSOR_LATEST_QOS)
        self.create_subscription(
            Bool, '/front/cctv_marker_visible', self.front_top_marker_cb,
            SENSOR_LATEST_QOS)
        self.create_subscription(
            Bool, '/rear/cctv_marker_visible', self.rear_top_marker_cb,
            SENSOR_LATEST_QOS)
        self.create_subscription(
            Bool, '/emergency_stop', self.estop_cb, STATE_LATEST_QOS)
        self.create_subscription(
            Bool, '/robot/lifted', self.vehicle_lifted_cb, 10)
        self.create_subscription(
            String, '/front/robot_state', self.front_state_cb,
            STATE_LATEST_QOS)
        self.create_subscription(
            String, '/rear/robot_state', self.rear_state_cb,
            STATE_LATEST_QOS)
        self.create_subscription(
            PoseStamped, '/parking/vehicle_pose_feedback',
            self.cctv_feedback_cb, SENSOR_LATEST_QOS)
        self.create_subscription(
            PoseStamped, '/parking/target_pose', self.target_cb, 10)
        self.create_subscription(
            PoseStamped, '/parking/slot_pose', self.slot_cb, self.mission_qos)
        self.create_subscription(
            String, '/parking/vehicle_spec', self.vehicle_spec_cb,
            self.mission_qos)

        # ===== 발행 =====
        self.pub_fc = self.create_publisher(
            TwistStamped, '/front/cmd_vel', CMD_VEL_QOS)
        self.pub_rc = self.create_publisher(
            TwistStamped, '/rear/cmd_vel', CMD_VEL_QOS)
        self.pub_err = self.create_publisher(String, '/sync/error_state', 10)
        self.pub_estop = self.create_publisher(Bool, '/emergency_stop', 10)

        self.create_timer(0.02, self.control_loop)  # 50 Hz
        self.create_timer(1.0, self.log_status)

        self.get_logger().info(
            'rigid_body_sync 시작 | '
            f'wheelbase={self.wheelbase:.3f}m | '
            f'yaw_hold={self.hold_initial_yaw} | '
            f'aruco_offset={self.aruco_distance_offset:+.3f}m | '
            f'aruco_dist={self.use_aruco_distance}')
        if not self.use_aruco_distance:
            self.get_logger().warn(
                'use_aruco_distance=false: 장착 offset 실측 전이므로 ArUco는 '
                '상대 yaw에만 사용하고 중심거리는 encoder로 유지합니다.')

    def _validate_parameters(self):
        validate_wheelbase_clearance(
            self.wheelbase, self.robot_length,
            self.minimum_inter_robot_gap)
        if self.max_speed <= 0.0 or self.max_omega <= 0.0:
            raise ValueError('max_speed/max_omega must be positive')
        if not 0.0 < self.dist_limit < self.dist_stop_limit:
            raise ValueError('need 0 < dist_error_limit < dist_stop_limit')
        if self.dist_error_timeout <= 0.0 or self.odom_timeout <= 0.0:
            raise ValueError('timeouts must be positive')
        if not 0.0 < self.marker_slowdown < self.marker_stop:
            raise ValueError('need 0 < marker_slowdown_s < marker_stop_s')
        if not 0.0 < self.aruco_timeout < self.marker_stop:
            raise ValueError('aruco_timeout_s must be in (0, marker_stop_s)')
        if self.cctv_marker_timeout <= 0.0:
            raise ValueError('cctv_marker_timeout_s must be positive')
        if self.aruco_min_distance < 0.0 or \
                self.aruco_max_distance <= self.aruco_min_distance:
            raise ValueError('invalid ArUco distance bounds')
        if self.cctv_feedback_gate <= 0.0:
            raise ValueError('cctv_feedback_gate_m must be positive')
        if self.initial_target_offset_gate <= 0.0:
            raise ValueError('initial_target_offset_gate_m must be positive')
        if not 0.0 < self.cctv_offset_alpha <= 1.0:
            raise ValueError('cctv_offset_alpha must be in (0,1]')
        if not 0.0 < self.final_speed_ratio <= 1.0:
            raise ValueError('final_speed_ratio must be in (0,1]')
        if not (0.0 < self.path_goal_tolerance < self.final_approach_dist):
            raise ValueError(
                'need 0 < path_goal_tolerance < final_approach_dist')
        if (self.final_pos_tol <= 0.0 or self.final_yaw_tol <= 0.0 or
                not 0.0 < self.final_lateral_tol <= self.final_approach_dist):
            raise ValueError('invalid final pose/lateral tolerances')
        if any(value <= 0.0 for value in (
                self.path_timeout, self.target_timeout, self.slot_timeout,
                self.slot_pose_wait_timeout)):
            raise ValueError('path/target/slot timeouts must be positive')
        if self.future_tolerance < 0.0:
            raise ValueError('future_tolerance_s must be non-negative')

    def _accept_stamped(self, stream, msg):
        accepted, reason = self.stamp_gates[stream].accept(
            stamp_to_ns(msg.header.stamp),
            self.get_clock().now().nanoseconds)
        if not accepted:
            self.get_logger().warn(
                f'{stream} rejected: {reason}',
                throttle_duration_sec=2.0)
        return accepted

    # ===== 콜백 =====
    def path_cb(self, msg):
        if msg.header.frame_id not in ('', 'map'):
            self.get_logger().warn(
                f'waypoint frame={msg.header.frame_id!r}: map 경로만 허용')
            return
        if not self._accept_stamped('path', msg):
            return
        waypoints = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if not waypoints:
            return
        if not all(math.isfinite(v) for point in waypoints for v in point):
            self.get_logger().warn('비정상 waypoint(NaN/Inf) 폐기')
            return

        self.pursuit.set_path(waypoints)
        path_stamp_ns = stamp_to_ns(msg.header.stamp)
        self.path_mission_stamp_ns = path_stamp_ns
        if self.pending_slot_mission_stamp_ns == path_stamp_ns:
            # DDS는 서로 다른 topic 순서를 보장하지 않으므로 slot이 path보다
            # 먼저 도착한 경우를 pending으로 보관했다가 같은 stamp에 연결한다.
            self.slot_pose = self.pending_slot_pose
            self.slot_mission_stamp_ns = path_stamp_ns
            self.pending_slot_pose = None
            self.pending_slot_mission_stamp_ns = None
        elif self.slot_mission_stamp_ns != path_stamp_ns:
            # transient-local에 남은 이전 임무 슬롯을 새 경로와 섞지 않는다.
            self.slot_pose = None
            self.slot_mission_stamp_ns = None
        self.slot_pose_missing_since = None
        self.has_path = True
        self.final_mode = False
        self.marker_lost_since = None
        self.dist_error_since = None
        self.dist_pid.reset()
        self.yaw_pid.reset()
        self._err = 'OK'

        self.yaw_reference = None
        self.target_offset_initialized = False
        self.vehicle_offset_body[:] = [0.0, 0.0]
        self.sync_filters_initialized = False
        if self.front_ready and self.rear_ready:
            cx, cy, self.yaw_reference = self.kinematics.virtual_pose(
                self.front, self.rear)
            self._initialize_sync_filters()
            self._initialize_target_offset(cx, cy, self.yaw_reference)
        ref = ('pending' if self.yaw_reference is None else
               f'{math.degrees(self.yaw_reference):.1f}deg')
        offset_text = (
            f'body({self.vehicle_offset_body[0]:+.3f},'
            f'{self.vehicle_offset_body[1]:+.3f})m'
            if self.target_offset_initialized else 'pending/unchanged')
        self.get_logger().info(
            f'waypoint {len(waypoints)}개 수신 | yaw_reference={ref} | '
            f'initial_vehicle_offset={offset_text}')

    def front_cb(self, msg):
        pose = self._parse_odom(msg, 'front_odom')
        if pose is not None:
            self.front = pose
            self.front_ready = True

    def rear_cb(self, msg):
        pose = self._parse_odom(msg, 'rear_odom')
        if pose is not None:
            self.rear = pose
            self.rear_ready = True

    def _parse_odom(self, msg, stream):
        if msg.header.frame_id != 'map':
            self.get_logger().warn(
                f'{stream} frame={msg.header.frame_id!r}: map odom required',
                throttle_duration_sec=2.0)
            return None
        if not self._accept_stamped(stream, msg):
            return None
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        theta = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                           1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        if not all(math.isfinite(v) for v in (p.x, p.y, theta)):
            self.get_logger().warn('비정상 odom(NaN/Inf) 폐기')
            return None
        return {'x': float(p.x), 'y': float(p.y),
                'theta': theta, 't': time.monotonic()}

    def _initialize_sync_filters(self):
        """경로 시작 시 실제 현재 간격/yaw를 Kalman 절대 초기값으로 사용한다.

        ScalarKalman은 이후 raw delta만 누적하므로 첫 값을 wheelbase=고정값으로
        두면 시작 순간의 정렬 오차가 영원히 보이지 않는다. 현재 odom의 절대값과
        raw 기준을 동시에 초기화해 그 오차도 즉시 fail-safe/PID에 반영한다.
        """
        if not (self.front_ready and self.rear_ready):
            return False
        enc_dist = self.kinematics.encoder_distance(self.front, self.rear)
        enc_yaw = self.angle_norm(self.front['theta'] - self.rear['theta'])
        if not all(math.isfinite(v) for v in (enc_dist, enc_yaw)):
            return False
        self.dist_kalman.reset(enc_dist, raw_value=enc_dist)
        self.yaw_kalman.reset(enc_yaw, raw_value=enc_yaw)
        self.sync_filters_initialized = True
        self.get_logger().info(
            f'상대필터 초기화: distance={enc_dist:.3f}m, '
            f'yaw={math.degrees(enc_yaw):+.2f}deg')
        return True

    def aruco_cb(self, msg):
        if msg.header.frame_id != 'rear_base':
            self.get_logger().warn(
                'ArUco pose rejected: WRONG_FRAME',
                throttle_duration_sec=2.0)
            return
        raw_dist = float(msg.pose.position.x)
        corrected = raw_dist + self.aruco_distance_offset
        q = msg.pose.orientation
        quaternion = (float(q.x), float(q.y), float(q.z), float(q.w))
        if not all(math.isfinite(value) for value in quaternion):
            self.get_logger().warn(
                'ArUco pose rejected: INVALID_QUATERNION',
                throttle_duration_sec=2.0)
            return
        norm = math.sqrt(sum(value * value for value in quaternion))
        if norm < 1e-6:
            self.get_logger().warn(
                'ArUco pose rejected: INVALID_QUATERNION',
                throttle_duration_sec=2.0)
            return
        qx, qy, qz, qw = (value / norm for value in quaternion)
        yaw = math.atan2(2.0 * (qw * qz + qx * qy),
                         1.0 - 2.0 * (qy * qy + qz * qz))
        required_values = ((raw_dist, corrected, yaw)
                           if self.use_aruco_distance else (yaw,))
        if not all(math.isfinite(v) for v in required_values):
            self.get_logger().warn(
                '비정상 ArUco pose 폐기', throttle_duration_sec=2.0)
            return
        if (self.use_aruco_distance and not
                self.aruco_min_distance <= corrected <= self.aruco_max_distance):
            self.get_logger().warn(
                f'ArUco 중심거리 범위 밖: raw={raw_dist:.3f}m, '
                f'corrected={corrected:.3f}m',
                throttle_duration_sec=2.0)
            return
        # Validate the payload envelope before advancing the ordering gate. A
        # wrong-frame packet must not poison the next legitimate observation.
        if not self._accept_stamped('aruco', msg):
            return

        self.aruco_raw_dist = raw_dist if math.isfinite(raw_dist) else None
        self.aruco_dist = corrected if math.isfinite(corrected) else None
        self.aruco_yaw = yaw
        # source timestamp 대신 local receive freshness를 사용해 Front/Rear 시스템
        # 시계 오차로 정상 ArUco가 stale 처리되는 문제를 막는다.
        self.aruco_receipt_time = time.monotonic()
        self.marker_visible = True

    def marker_cb(self, msg):
        self.marker_visible = bool(msg.data)

    def front_top_marker_cb(self, msg):
        self.front_top_marker_visible = bool(msg.data)
        self.front_top_marker_time = time.monotonic()

    def rear_top_marker_cb(self, msg):
        self.rear_top_marker_visible = bool(msg.data)
        self.rear_top_marker_time = time.monotonic()

    def top_marker_pair_fresh(self, now):
        if not (self.front_top_marker_visible and
                self.rear_top_marker_visible and
                self.front_top_marker_time is not None and
                self.rear_top_marker_time is not None):
            return False
        return (
            0.0 <= now - self.front_top_marker_time <
            self.cctv_marker_timeout and
            0.0 <= now - self.rear_top_marker_time <
            self.cctv_marker_timeout)

    def estop_cb(self, msg):
        if msg.data:
            self.estop = True
            self.send_stop()

    def vehicle_lifted_cb(self, msg):
        self.vehicle_lifted = bool(msg.data)
        if not self.vehicle_lifted and self.has_path:
            self.send_stop()

    def front_state_cb(self, msg):
        self.front_robot_state = str(msg.data)
        if self.front_robot_state != 'DRIVE' and self.has_path:
            self.send_stop()

    def rear_state_cb(self, msg):
        self.rear_robot_state = str(msg.data)
        if self.rear_robot_state != 'DRIVE' and self.has_path:
            self.send_stop()

    def target_cb(self, msg):
        if msg.header.frame_id not in ('', 'map'):
            return
        if not self._accept_stamped('target', msg):
            return
        values = (float(msg.pose.position.x), float(msg.pose.position.y))
        if all(math.isfinite(v) for v in values):
            self.target_pose = values
            if self.has_path and self.front_ready and self.rear_ready:
                cx, cy, ct = self.kinematics.virtual_pose(
                    self.front, self.rear)
                self._initialize_target_offset(cx, cy, ct)

    def _initialize_target_offset(self, cx, cy, yaw):
        if self.target_offset_initialized:
            return True
        if not self.initialize_offset_from_target or self.target_pose is None:
            return False
        world_x = self.target_pose[0] - cx
        world_y = self.target_pose[1] - cy
        magnitude = math.hypot(world_x, world_y)
        if magnitude > self.initial_target_offset_gate:
            self.get_logger().warn(
                f'초기 target offset gate 초과: {magnitude:.3f}m',
                throttle_duration_sec=2.0)
            return False
        body_x, body_y = self.kinematics.world_offset_to_body(
            world_x, world_y, yaw)
        self.vehicle_offset_body[:] = [body_x, body_y]
        self.cctv_time = time.monotonic()
        self.target_offset_initialized = True
        self.get_logger().info(
            '차량 중심 초기 body-offset 적용: '
            f'({body_x:+.3f},{body_y:+.3f})m')
        return True

    def cctv_feedback_cb(self, msg):
        # 하차 후 차량은 슬롯에 남고 로봇만 복귀한다. 이때도 YOLO 차량
        # feedback을 계속 받아 offset을 갱신하면 로봇 좌표가 주차된 차량을
        # 따라가는 값으로 오염된다. 실제 운반 중에만 절대보정을 허용한다.
        if (not self.has_path or not self.vehicle_lifted or
                self.front_robot_state != 'DRIVE' or
                self.rear_robot_state != 'DRIVE'):
            return
        if not (self.front_ready and self.rear_ready):
            return
        if msg.header.frame_id != 'map':
            return
        if not self._accept_stamped('cctv_feedback', msg):
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        ros_now = self.get_clock().now().nanoseconds * 1e-9
        age = ros_now - stamp if stamp > 0.0 else 0.0
        if age < -0.1 or age > self.cctv_feedback_timeout:
            return
        if not all(math.isfinite(v) for v in
                   (msg.pose.position.x, msg.pose.position.y)):
            return

        cx_enc, cy_enc, ct = self.kinematics.virtual_pose(
            self.front, self.rear)
        desired_world_x = msg.pose.position.x - cx_enc
        desired_world_y = msg.pose.position.y - cy_enc
        desired_body_x, desired_body_y = self.kinematics.world_offset_to_body(
            desired_world_x, desired_world_y, ct)
        residual_x = desired_body_x - self.vehicle_offset_body[0]
        residual_y = desired_body_y - self.vehicle_offset_body[1]
        residual = math.hypot(residual_x, residual_y)
        if residual > self.cctv_feedback_gate:
            self.get_logger().warn(
                f'CCTV vehicle feedback gate 초과: {residual:.3f}m',
                throttle_duration_sec=2.0)
            return

        alpha = self.cctv_offset_alpha
        self.vehicle_offset_body[0] += alpha * residual_x
        self.vehicle_offset_body[1] += alpha * residual_y
        self.cctv_time = time.monotonic()

    def slot_cb(self, msg):
        if msg.header.frame_id not in ('', 'map'):
            return
        if not self._accept_stamped('slot', msg):
            return
        slot_stamp_ns = stamp_to_ns(msg.header.stamp)
        q = msg.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        values = (float(msg.pose.position.x), float(msg.pose.position.y), yaw)
        if not all(math.isfinite(v) for v in values):
            return
        if (self.path_mission_stamp_ns is not None and
                slot_stamp_ns != self.path_mission_stamp_ns):
            self.pending_slot_pose = values
            self.pending_slot_mission_stamp_ns = slot_stamp_ns
            self.get_logger().warn(
                'slot_pose가 현재 waypoint과 다른 stamp — '
                '다음 path 순서 대기')
        else:
            self.slot_pose = values
            self.slot_mission_stamp_ns = slot_stamp_ns
            self.slot_pose_missing_since = None

    def vehicle_spec_cb(self, msg):
        if not self.use_vehicle_spec_wheelbase:
            return
        try:
            payload = json.loads(msg.data)
            candidate = float(payload['wheelbase'])
            stamp_ns = int(payload['stamp_ns'])
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            self.get_logger().warn('invalid vehicle_spec envelope')
            return
        accepted, reason = self.stamp_gates['vehicle_spec'].accept(
            stamp_ns, self.get_clock().now().nanoseconds)
        if not accepted:
            self.get_logger().warn(
                f'vehicle_spec rejected: {reason}',
                throttle_duration_sec=2.0)
            return
        if not math.isfinite(candidate) or candidate <= 0.0:
            self.get_logger().warn('invalid vehicle_spec wheelbase')
            return
        try:
            validate_wheelbase_clearance(
                candidate, self.robot_length,
                self.minimum_inter_robot_gap)
        except ValueError as exc:
            self.get_logger().warn(
                f'unsafe vehicle_spec wheelbase rejected: {exc}')
            return
        if (self.vehicle_lifted or self.front_robot_state in
                ('DRIVE', 'WAIT_RELEASE', 'RELEASE') or
                self.rear_robot_state in
                ('DRIVE', 'WAIT_RELEASE', 'RELEASE')):
            self.get_logger().warn(
                'active transport 중 wheelbase 변경 거부')
            return
        if math.isclose(candidate, self.wheelbase, abs_tol=1e-6):
            return
        self.wheelbase = candidate
        self.kinematics.set_wheelbase(candidate)
        self.dist_kalman.reset(candidate)
        self.sync_filters_initialized = False
        self.get_logger().info(
            f'vehicle_spec wheelbase={candidate:.3f}m applied')

    # ===== 제어 =====
    def control_loop(self):
        if self.estop:
            self.send_stop()
            return
        if not (self.front_ready and self.rear_ready):
            return
        if not self.has_path:
            return
        # 경로는 transient-local이라 노드 재시작 시 과거 임무가 다시 올 수
        # 있다. 실제 운반 상태를 두 조건으로 확인하기 전에는 명령하지 않는다.
        if (not self.vehicle_lifted or
                self.front_robot_state != 'DRIVE' or
                self.rear_robot_state != 'DRIVE'):
            self.send_stop()
            return

        now = time.monotonic()
        if now - self.front['t'] > self.odom_timeout or \
                now - self.rear['t'] > self.odom_timeout:
            self.fatal_stop('ODOM_TIMEOUT')
            return

        cx_raw, cy_raw, ct = self.kinematics.virtual_pose(
            self.front, self.rear)
        if not self.target_offset_initialized:
            self._initialize_target_offset(cx_raw, cy_raw, ct)
        cx, cy, _ = self.kinematics.control_point_pose(
            cx_raw, cy_raw, ct,
            self.vehicle_offset_body[0], self.vehicle_offset_body[1])

        if self.hold_initial_yaw and self.yaw_reference is None:
            self.yaw_reference = ct
            self.get_logger().info(
                f'yaw_reference 캡처: {math.degrees(ct):.1f}deg')
        if not self.sync_filters_initialized and not self._initialize_sync_filters():
            self.fatal_stop('SYNC_FILTER_INIT_FAILED')
            return

        if self.pursuit.waypoints:
            gx, gy = self.pursuit.waypoints[-1]
            if math.hypot(gx - cx, gy - cy) < self.final_approach_dist and \
                    self.slot_pose is not None:
                self.final_mode = True

        if self.final_mode:
            done, command, info = self.compute_final_command(cx, cy, ct)
            if done:
                self.finish_path('정밀 정렬 완료 — 도착', cx, cy, ct)
                return
            self.apply_sync_and_publish(
                *command, now,
                mode='FINAL_APPROACH',
                # 차량 중심 평행은 final_speed_ratio로 이미 저속화된다.
                # 로봇별 선속도 상한은 회전 반경 속도(ω·L/2) +
                # offset 보상을 자르지 않도록 정상 max_speed를 사용한다.
                linear_limit=self.max_speed,
                angular_limit=min(self.max_omega, 0.15),
                extra_info=info)
            return

        result = self.pursuit.compute(cx, cy, ct)
        if result is None:
            self.send_stop()
            return
        vx, vy, path_omega = result
        if self.pursuit.is_finished(cx, cy):
            if self.align_to_slot_yaw:
                # 슬롯 pose가 없으면 외부 staging을 최종 도착으로 처리해
                # 차량을 조기 release하면 안 된다. 잠시 대기 후 fail-stop.
                if self.slot_pose is None:
                    self.send_stop()
                    if self.slot_pose_missing_since is None:
                        self.slot_pose_missing_since = now
                    elif (now - self.slot_pose_missing_since >=
                          self.slot_pose_wait_timeout):
                        self.fatal_stop('SLOT_POSE_MISSING')
                    return
                self.final_mode = True
                return
            self.finish_path('waypoint 추종 완료 — 도착', cx, cy, ct)
            return

        if self.hold_initial_yaw:
            omega, yaw_hold_error = self.yaw_hold_command(ct, self.max_omega)
        else:
            omega, yaw_hold_error = path_omega, 0.0

        self.apply_sync_and_publish(
            vx, vy, omega, now,
            mode='PATH_TRACKING',
            linear_limit=self.max_speed,
            angular_limit=self.max_omega,
            extra_info={
                'yaw_hold_err_deg': round(math.degrees(yaw_hold_error), 2),
            })

    def compute_final_command(self, cx, cy, ct):
        """staging에서 슬롯 Yaw 정렬 후 슬롯 중심으로 저속 직선 삽입한다."""
        sx, sy, slot_yaw = self.slot_pose
        ex, ey = sx - cx, sy - cy
        pos_err = math.hypot(ex, ey)
        yaw_tol = self.final_yaw_tol

        if self.align_to_slot_yaw:
            yaw_error = self.angle_norm(slot_yaw - ct)
            rotation_radius = (
                self.kinematics.half_L + math.hypot(
                    self.vehicle_offset_body[0],
                    self.vehicle_offset_body[1]))
            kinematic_omega_limit = min(
                self.max_omega, 0.10,
                0.8 * self.max_speed / max(rotation_radius, 1e-6))
            omega = self.clamp(
                self.yaw_hold_kp * yaw_error,
                kinematic_omega_limit)
        elif self.hold_initial_yaw:
            omega, yaw_error = self.yaw_hold_command(
                ct, min(self.max_omega, 0.10))
        else:
            yaw_error = self.angle_norm(slot_yaw - ct)
            omega = self.clamp(
                self.yaw_hold_kp * yaw_error,
                min(self.max_omega, 0.10))

        pos_tol = self.final_pos_tol
        axis_c, axis_s = math.cos(slot_yaw), math.sin(slot_yaw)
        longitudinal_error = ex * axis_c + ey * axis_s
        lateral_error = -ex * axis_s + ey * axis_c
        info = {
            'goal_pos_err_cm': round(pos_err * 100.0, 2),
            'yaw_hold_err_deg': round(math.degrees(yaw_error), 2),
            'slot_longitudinal_err_cm': round(
                longitudinal_error * 100.0, 2),
            'slot_lateral_err_cm': round(lateral_error * 100.0, 2),
            'final_phase': (
                'ALIGN_SLOT_YAW' if abs(yaw_error) >= yaw_tol
                else ('ALIGN_SLOT_CENTERLINE'
                      if abs(lateral_error) > self.final_lateral_tol
                      else 'INSERT_ALONG_SLOT_AXIS')),
        }
        if (pos_err < pos_tol and abs(yaw_error) < yaw_tol and
                abs(lateral_error) <= self.final_lateral_tol):
            return True, (0.0, 0.0, 0.0), info

        # 슬롯 축이 맞기 전에는 평행이동하지 않는다. 좁은 슬롯 안에서 돌지 않고
        # Fleet Manager가 계산한 외부 staging point에서 제자리 회전한다.
        if self.align_to_slot_yaw and abs(yaw_error) >= yaw_tol:
            return False, (0.0, 0.0, omega), info

        slow = self.max_speed * self.final_speed_ratio
        # 슬롯 밖에서 중심선 횡오차를 먼저 닫고, 그 다음에는
        # 슬롯 축 속도만 허용한다. 목표 중심으로 대각선 진입해
        # 검증한 insertion corridor 밖으로 나가는 것을 막는다.
        if abs(lateral_error) > self.final_lateral_tol:
            lateral_speed = self.clamp(
                1.5 * lateral_error, slow)
            world_vx = -axis_s * lateral_speed
            world_vy = axis_c * lateral_speed
        else:
            longitudinal_speed = self.clamp(
                1.0 * longitudinal_error, slow)
            world_vx = axis_c * longitudinal_speed
            world_vy = axis_s * longitudinal_speed

        c, s = math.cos(ct), math.sin(ct)
        vx = world_vx * c + world_vy * s
        vy = -world_vx * s + world_vy * c
        return False, (vx, vy, omega), info

    def yaw_hold_command(self, current_yaw, omega_limit):
        if self.yaw_reference is None:
            return 0.0, 0.0
        error = self.angle_norm(self.yaw_reference - current_yaw)
        return self.clamp(self.yaw_hold_kp * error, omega_limit), error

    def apply_sync_and_publish(self, vx, vy, omega, now, *, mode,
                               linear_limit, angular_limit, extra_info=None):
        # vx/vy는 차량 중심(control point) 명령이다. body offset이
        # 있으면 회전 시 로봇 중점에 반대 평행속도를 주어 차량
        # 중심이 staging 위치에서 표류하지 않게 한다.
        centre_vx, centre_vy, centre_omega = (
            self.kinematics.control_point_twist_to_centre(
                vx, vy, omega,
                self.vehicle_offset_body[0], self.vehicle_offset_body[1]))
        front_vel, rear_vel = self.kinematics.split(
            centre_vx, centre_vy, centre_omega)

        enc_dist = self.kinematics.encoder_distance(self.front, self.rear)
        self.dist_kalman.predict(enc_dist)
        enc_yaw = self.angle_norm(self.front['theta'] - self.rear['theta'])
        self.yaw_kalman.predict(enc_yaw)

        aruco_age = (None if self.aruco_receipt_time is None else
                     now - self.aruco_receipt_time)
        aruco_fresh = (aruco_age is not None and
                       0.0 <= aruco_age < self.aruco_timeout and
                       self.aruco_dist is not None)
        correction = 'ENCODER'
        if aruco_fresh:
            if self.use_aruco_distance:
                self.dist_kalman.update(self.aruco_dist)
            if self.aruco_yaw is not None:
                self.yaw_kalman.update(self.aruco_yaw)
            correction = ('ARUCO_DIST_YAW' if self.use_aruco_distance
                          else 'ARUCO_YAW')
        elif self.top_marker_pair_fresh(now):
            # The configured top markers provide an absolute pair observation.
            # inputs are already pose-fused, so use their relative measurement
            # to anchor both filters while ID0 is occluded.
            self.dist_kalman.update(enc_dist)
            self.yaw_kalman.update(enc_yaw)
            correction = 'CCTV_TOP_MARKERS'

        fused_dist = self.dist_kalman.x
        relative_yaw_error = self.angle_norm(self.yaw_kalman.x)
        dist_error = fused_dist - self.wheelbase

        if correction == 'ENCODER':
            if self.marker_lost_since is None:
                self.marker_lost_since = now
        else:
            self.marker_lost_since = None
            if self._err.startswith('MARKER_HOLD'):
                self._err = 'OK'

        speed_scale = 1.0
        effective_yaw_limit = self.yaw_limit
        if self.marker_lost_since is not None:
            lost = now - self.marker_lost_since
            if lost > self.marker_stop:
                self.recoverable_hold(f'MARKER_HOLD {lost:.1f}s')
                return False
            if lost > self.marker_slowdown:
                speed_scale = 0.5
                effective_yaw_limit = self.yaw_limit * 2.0

        if abs(relative_yaw_error) > effective_yaw_limit:
            self.fatal_stop(
                f'YAW_ERROR {math.degrees(relative_yaw_error):.1f}deg')
            return False

        abs_dist_error = abs(dist_error)
        if abs_dist_error >= self.dist_stop_limit:
            self.fatal_stop(f'DIST_ERROR_FATAL {dist_error * 1000:.0f}mm')
            return False
        if abs_dist_error > self.dist_limit:
            if self.dist_error_since is None:
                self.dist_error_since = now
            elif now - self.dist_error_since > self.dist_error_timeout:
                self.fatal_stop(
                    f'DIST_ERROR_TIMEOUT {dist_error * 1000:.0f}mm')
                return False
            speed_scale = min(speed_scale, 0.30)
            self._err = f'DIST_ERROR {dist_error * 1000:.0f}mm'
        else:
            self.dist_error_since = None

        corr_x = self.dist_pid.compute(dist_error, 0.02)
        corr_w = self.yaw_pid.compute(relative_yaw_error, 0.02)

        front_cmd = (
            (front_vel[0] - 0.5 * corr_x) * speed_scale,
            front_vel[1] * speed_scale,
            (front_vel[2] - 0.5 * corr_w) * speed_scale)
        rear_cmd = (
            (rear_vel[0] + 0.5 * corr_x) * speed_scale,
            rear_vel[1] * speed_scale,
            (rear_vel[2] + 0.5 * corr_w) * speed_scale)

        # 두 로봇을 개별 제한하면 한쪽의 회전 궤도속도만 잘려 강체
        # 관계가 무너진다. Front/Rear 명령 전체를 같은 비율로 줄여
        # 차량 중심 고정 회전과 동기 보정의 기하 관계를 보존한다.
        front_cmd, rear_cmd = self.kinematics.limit_twist_pair(
            front_cmd, rear_cmd,
            linear_limit * speed_scale,
            angular_limit * speed_scale)
        self.publish_twist(self.pub_fc, front_cmd, 'front_base')
        self.publish_twist(self.pub_rc, rear_cmd, 'rear_base')

        self._info = {
            'mode': mode,
            'enc_dist_cm': round(enc_dist * 100.0, 2),
            'aruco_distance_used': self.use_aruco_distance,
            'aruco_raw_cm': (None if self.aruco_raw_dist is None else
                             round(self.aruco_raw_dist * 100.0, 2)),
            'fused_dist_cm': round(fused_dist * 100.0, 2),
            'dist_err_mm': round(dist_error * 1000.0, 1),
            'relative_yaw_err_deg': round(
                math.degrees(relative_yaw_error), 2),
            'correction': correction,
            'speed_scale': speed_scale,
        }
        if extra_info:
            self._info.update(extra_info)
        return True

    # ===== 상태/발행 헬퍼 =====
    def finish_path(self, message, vehicle_x, vehicle_y, vehicle_yaw):
        self.send_stop()
        self._err = 'ARRIVED'
        arrival = make_arrival_status(
            vehicle_x, vehicle_y, vehicle_yaw,
            self.path_mission_stamp_ns)
        self._info.update(arrival)
        self.has_path = False
        self.final_mode = False
        self.publish_status_now()
        self.get_logger().info(message)

    def fatal_stop(self, reason):
        self.send_stop()
        self._err = reason
        self.has_path = False
        self.final_mode = False
        self.publish_status_now()
        policy = classify_fault(f'SYNC,{reason}')
        if policy.estop_required:
            self.pub_estop.publish(Bool(data=True))
            self.estop = True
        self.get_logger().error(reason)

    def recoverable_hold(self, reason):
        """Stop both robots without discarding the path or latching E-stop."""
        self.send_stop()
        self._err = str(reason)
        self.get_logger().warn(
            f'{reason}; waiting for fresh visual evidence',
            throttle_duration_sec=2.0)

    def publish_status_now(self):
        info = dict(self._info)
        info['error'] = self._err
        info['marker_bool'] = self.marker_visible
        msg = String()
        msg.data = json.dumps(info)
        self.pub_err.publish(msg)

    @staticmethod
    def limit_twist(command, linear_limit, angular_limit):
        vx, vy, omega = command
        linear_limit = max(0.0, linear_limit)
        angular_limit = max(0.0, angular_limit)
        planar = math.hypot(vx, vy)
        if linear_limit == 0.0:
            vx = vy = 0.0
        elif planar > linear_limit:
            ratio = linear_limit / planar
            vx *= ratio
            vy *= ratio
        omega = RigidBodySyncNode.clamp(omega, angular_limit)
        return vx, vy, omega

    def publish_twist(self, publisher, command, frame_id):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        (msg.twist.linear.x,
         msg.twist.linear.y,
         msg.twist.angular.z) = command
        publisher.publish(msg)

    @staticmethod
    def angle_norm(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def clamp(value, limit):
        return max(-limit, min(limit, value))

    def send_stop(self):
        self.publish_twist(
            self.pub_fc, (0.0, 0.0, 0.0), 'front_base')
        self.publish_twist(
            self.pub_rc, (0.0, 0.0, 0.0), 'rear_base')
        self.dist_pid.reset()
        self.yaw_pid.reset()

    def log_status(self):
        self.publish_status_now()
        info = dict(self._info)
        if self.has_path and info.get('fused_dist_cm') is not None:
            self.get_logger().info(
                f"{info.get('mode')} | 거리 {info.get('fused_dist_cm')}cm | "
                f"상대yaw {info.get('relative_yaw_err_deg')}deg | "
                f"{info.get('correction')} | {self._err}")
        if self._err != 'ARRIVED' and not self.estop:
            self._err = 'OK'


def main(args=None):
    rclpy.init(args=args)
    node = RigidBodySyncNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
