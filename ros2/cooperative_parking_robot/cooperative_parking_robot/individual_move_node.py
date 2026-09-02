#!/usr/bin/env python3
"""Safe individual motion for underbody entry, axle scan, and exit.

Top-level state ownership is unchanged:

* ``APPROACH``: require a rear-of-vehicle start and move directly to the
  longitudinal standoff/observation queue; no side-entry staging is used.
* ``ALIGN``: hold longitudinal motion in ``PRE_ALIGN`` until lateral/yaw are
  stable, then scan inward while correcting the axis; retreat/retry on a
  bounded lateral deviation and center on the paired ultrasonic axle result.
* ``RETURN``: after release, explicitly leave the underbody longitudinally,
  move to the side lane, then return home.

The internal phase is published on ``/{role}/motion_phase`` for diagnostic
verification. A bounded phase or scan failure is published on
``/{role}/motion_fault`` and is converted to FAULT by the robot state machine.
"""

import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, QoSProfile, ReliabilityPolicy,
)
from std_msgs.msg import Bool, Float64, String

from cooperative_parking_robot.command_qos import CMD_VEL_QOS
from cooperative_parking_robot.latest_qos import (
    SENSOR_LATEST_QOS,
    STATE_LATEST_QOS,
)
from cooperative_parking_robot.vehicle_entry import (
    DEFAULT_WHEELBASE_M,
    MIN_INTER_ROBOT_GAP_M,
    ROBOT_LENGTH_M,
    ROBOT_WIDTH_M,
    angle_norm,
    approach_longitudinal,
    axle_longitudinal,
    exit_longitudinal_translation,
    initial_align_phase,
    initial_approach_phase,
    marker_loss_speed_scale,
    plan_around_vehicle,
    rear_scan_speed_from_relative,
    relative_alignment_is_consistent,
    scan_direction,
    segment_intersects_open_rect,
    standoff_longitudinal,
    validate_wheelbase_clearance,
    vehicle_to_world,
    world_to_vehicle,
)
from cooperative_parking_robot.freshness import StampGate, stamp_to_ns


# Rear owns the ID0 camera and keeps it active only while either robot can be
# below the vehicle or while final relative alignment is being held.
REAR_RELATIVE_VISION_PHASES = frozenset({
    "READY_TO_SCAN", "PRE_ALIGN", "WAIT_ULTRASONIC_READY", "PREALIGNED",
    "SCAN_IN", "RETREAT", "CENTER_AXLE", "ALIGNED",
    "WAIT_PEER_RETURN", "WAIT_EXIT_ODOM", "EXIT_UNDERBODY",
    "WAIT_PEER_EXIT_CLEAR",
})


class IndividualMoveNode(Node):
    def __init__(self, **kwargs):
        super().__init__("individual_move_node", **kwargs)

        self.declare_parameter("role", "front")
        self.declare_parameter("simultaneous_entry", False)
        self.declare_parameter("max_speed", 0.06)
        self.declare_parameter("scan_speed", 0.03)
        self.declare_parameter("centerline_speed", 0.035)
        self.declare_parameter("waiting_x", 0.3)
        self.declare_parameter("waiting_y", 0.3)
        self.declare_parameter("default_wheelbase", DEFAULT_WHEELBASE_M)
        self.declare_parameter("use_vehicle_spec_wheelbase", True)

        # Measured vehicle/robot envelope. The standoff and side lane must lie
        # outside this envelope; invalid launch values fail at startup.
        self.declare_parameter("vehicle_half_length_m", 0.45)
        self.declare_parameter("vehicle_half_width_m", 0.175)
        self.declare_parameter("robot_length_m", ROBOT_LENGTH_M)
        self.declare_parameter("robot_width_m", ROBOT_WIDTH_M)
        self.declare_parameter(
            "minimum_inter_robot_gap_m", MIN_INTER_ROBOT_GAP_M)
        self.declare_parameter("robot_clearance_m", 0.06)
        self.declare_parameter("entry_standoff_m", 0.85)
        self.declare_parameter("entry_side_offset_m", 0.40)
        self.declare_parameter("entry_side", -1)
        self.declare_parameter("exit_distance_m", 0.50)
        self.declare_parameter("same_direction_exit", False)
        self.declare_parameter("same_direction_exit_sign", 1)
        self.declare_parameter("exit_sync_gain", 0.15)
        self.declare_parameter("scan_overshoot_m", 0.10)

        # Front-rear ID0 relative observation. Longitudinal axle-center
        # authority remains ultrasonic; ArUco only keeps lateral/yaw alignment,
        # slows Rear's coarse scan, and validates the final separation.
        # Calibration is supplied by config/id0_calibration.yaml in every
        # production launch. Zero is an invalid fail-closed direct-run value.
        self.declare_parameter("aruco_distance_offset_m", 0.0)
        self.declare_parameter("aruco_timeout_s", 0.30)
        self.declare_parameter("cctv_marker_timeout_s", 0.50)
        self.declare_parameter("marker_slowdown_s", 0.75)
        self.declare_parameter("marker_stop_s", 1.50)
        self.declare_parameter("relative_lateral_gain", 0.60)
        self.declare_parameter("relative_yaw_gain", 0.70)
        self.declare_parameter("relative_distance_tolerance_m", 0.06)
        # Provisional demo value; calibrate from ID0 repeatability and gripper
        # lateral clearance before increasing the operating envelope.
        self.declare_parameter("relative_lateral_tolerance_m", 0.03)
        self.declare_parameter("relative_yaw_tolerance_deg", 4.0)
        self.declare_parameter("relative_mismatch_timeout_s", 0.50)
        self.declare_parameter("rear_min_scan_speed", 0.006)
        self.declare_parameter("rear_aruco_slowdown_window_m", 0.12)

        self.declare_parameter("position_tolerance_m", 0.025)
        self.declare_parameter("centerline_tolerance_m", 0.012)
        # Close lateral/yaw error before longitudinal insertion. The hold
        # count prevents a single noisy pose sample from opening the gate.
        self.declare_parameter("prealign_hold_n", 10)
        # Use paired ultrasonic returns as a vehicle-referenced lateral
        # correction while the overhead marker is occluded below the body.
        self.declare_parameter("use_ultrasonic_lateral", True)
        self.declare_parameter("ultrasonic_lateral_timeout_s", 0.30)
        self.declare_parameter("ultrasonic_lateral_yaw_gate_deg", 10.0)
        self.declare_parameter("ultrasonic_activation_timeout_s", 1.50)
        # Retreat and retry before a drifting insertion reaches the body.
        self.declare_parameter("lateral_deviation_limit_m", 0.030)
        self.declare_parameter("lateral_deviation_n", 5)
        self.declare_parameter("max_scan_retry", 2)
        self.declare_parameter("center_tolerance_m", 0.01)
        self.declare_parameter("center_gain", 1.5)
        self.declare_parameter("center_speed", 0.012)
        self.declare_parameter("lateral_gain", 1.2)
        self.declare_parameter("max_lateral_speed", 0.025)
        self.declare_parameter("yaw_gain", 1.5)
        self.declare_parameter("max_yaw_rate", 0.15)
        self.declare_parameter("yaw_tolerance_deg", 3.0)
        self.declare_parameter("substate_timeout_s", 60.0)
        self.declare_parameter("target_timeout_s", 2.0)
        self.declare_parameter("mission_data_timeout_s", 5.0)
        self.declare_parameter("future_tolerance_s", 0.10)
        self.declare_parameter("odom_timeout_s", 0.50)

        gp = self.get_parameter
        self.role = str(gp("role").value)
        if self.role not in ("front", "rear"):
            raise ValueError("role must be 'front' or 'rear'")
        self.is_front = self.role == "front"
        self.simultaneous_entry = bool(gp("simultaneous_entry").value)
        self.max_speed = float(gp("max_speed").value)
        self.scan_speed = float(gp("scan_speed").value)
        self.centerline_speed = float(gp("centerline_speed").value)
        self.wait_pos = (
            float(gp("waiting_x").value),
            float(gp("waiting_y").value),
        )
        self.wheelbase = float(gp("default_wheelbase").value)
        self.use_vehicle_spec_wheelbase = bool(
            gp("use_vehicle_spec_wheelbase").value)
        self.vehicle_half_length = float(gp("vehicle_half_length_m").value)
        self.vehicle_half_width = float(gp("vehicle_half_width_m").value)
        self.robot_length = float(gp("robot_length_m").value)
        self.robot_width = float(gp("robot_width_m").value)
        self.minimum_inter_robot_gap = float(
            gp("minimum_inter_robot_gap_m").value)
        self.robot_clearance = float(gp("robot_clearance_m").value)
        self.entry_standoff = float(gp("entry_standoff_m").value)
        self.entry_side_offset = float(gp("entry_side_offset_m").value)
        self.entry_side = int(gp("entry_side").value)
        self.exit_distance = float(gp("exit_distance_m").value)
        self.same_direction_exit = bool(
            gp("same_direction_exit").value)
        self.same_direction_exit_sign = int(
            gp("same_direction_exit_sign").value)
        self.exit_sync_gain = float(gp("exit_sync_gain").value)
        self.scan_overshoot = float(gp("scan_overshoot_m").value)
        self.aruco_distance_offset = float(
            gp("aruco_distance_offset_m").value)
        if self.aruco_distance_offset <= 0.0:
            raise ValueError(
                "aruco_distance_offset_m must come from ID0 calibration")
        self.aruco_timeout = float(gp("aruco_timeout_s").value)
        self.cctv_marker_timeout = float(
            gp("cctv_marker_timeout_s").value)
        self.marker_slowdown = float(gp("marker_slowdown_s").value)
        self.marker_stop = float(gp("marker_stop_s").value)
        self.relative_lateral_gain = float(
            gp("relative_lateral_gain").value)
        self.relative_yaw_gain = float(gp("relative_yaw_gain").value)
        self.relative_distance_tolerance = float(
            gp("relative_distance_tolerance_m").value)
        self.relative_lateral_tolerance = float(
            gp("relative_lateral_tolerance_m").value)
        self.relative_yaw_tolerance = math.radians(
            float(gp("relative_yaw_tolerance_deg").value))
        self.relative_mismatch_timeout = float(
            gp("relative_mismatch_timeout_s").value)
        self.rear_min_scan_speed = float(gp("rear_min_scan_speed").value)
        self.rear_aruco_slowdown_window = float(
            gp("rear_aruco_slowdown_window_m").value)
        self.position_tolerance = float(gp("position_tolerance_m").value)
        self.centerline_tolerance = float(
            gp("centerline_tolerance_m").value)
        self.prealign_hold_n = int(gp("prealign_hold_n").value)
        self.use_us_lateral = bool(gp("use_ultrasonic_lateral").value)
        self.us_lateral_timeout = float(
            gp("ultrasonic_lateral_timeout_s").value)
        self.us_lateral_yaw_gate = math.radians(
            float(gp("ultrasonic_lateral_yaw_gate_deg").value))
        self.ultrasonic_activation_timeout = float(
            gp("ultrasonic_activation_timeout_s").value)
        self.deviation_limit = float(gp("lateral_deviation_limit_m").value)
        self.deviation_n = int(gp("lateral_deviation_n").value)
        self.max_scan_retry = int(gp("max_scan_retry").value)
        self.center_tolerance = float(gp("center_tolerance_m").value)
        self.center_gain = float(gp("center_gain").value)
        self.center_speed = float(gp("center_speed").value)
        self.lateral_gain = float(gp("lateral_gain").value)
        self.max_lateral_speed = float(gp("max_lateral_speed").value)
        self.yaw_gain = float(gp("yaw_gain").value)
        self.max_yaw_rate = float(gp("max_yaw_rate").value)
        self.yaw_tolerance = math.radians(
            float(gp("yaw_tolerance_deg").value))
        self.substate_timeout = float(gp("substate_timeout_s").value)
        self.target_timeout = float(gp("target_timeout_s").value)
        self.mission_data_timeout = float(
            gp("mission_data_timeout_s").value)
        self.future_tolerance = float(gp("future_tolerance_s").value)
        self.odom_timeout = float(gp("odom_timeout_s").value)
        self._validate_parameters()
        self.target_gate = StampGate(
            self.target_timeout, self.future_tolerance)
        self.slot_gate = StampGate(
            self.mission_data_timeout, self.future_tolerance)
        self.spec_gate = StampGate(
            self.mission_data_timeout, self.future_tolerance)
        self.odom_gate = StampGate(
            self.odom_timeout, self.future_tolerance)
        self.relative_gate = StampGate(
            self.aruco_timeout, self.future_tolerance)

        self.robot_state = "IDLE"
        self.phase = "IDLE"
        self.phase_enter_time = time.monotonic()
        self.x = self.y = self.theta = 0.0
        self.odom_ready = False
        self.last_odom_time = 0.0
        self.latest_target = None
        self.latest_target_time = 0.0
        self.active_target = None
        self.slot_target = None
        self.route = []
        self.scan_origin_s = None
        self.exit_yaw = None
        self.exit_goal = None
        self.side_exit_goal = None
        self.wheel_detected = False
        self.wheel_center_s = None
        self.us_lateral = 0.0
        self.us_lateral_valid = False
        self.us_lateral_value_time = 0.0
        self.us_lateral_valid_time = 0.0
        self.ultrasonic_ready = False
        self.peer_ultrasonic_ready = False
        self.ultrasonic_requested = False
        self.lateral_source = None
        self.prealign_ok_n = 0
        self.deviation_cnt = 0
        self.scan_retry = 0
        self.retreat_goal_s = None
        self.alignment_sent = False
        self.front_staged = False
        self.rear_observation_ready = False
        self.front_align_done = False
        self.relative_x = None
        self.relative_y = None
        self.relative_yaw = None
        self.relative_receipt_time = None
        self.relative_marker_visible = False
        self.relative_vision_ready = False
        self.top_marker_visible = False
        self.top_visibility_received = False
        self.top_marker_receipt_time = None
        self.last_visual_observation_time = None
        self.relative_lost_since = None
        self.relative_mismatch_since = None
        self.motion_speed_scale = 1.0
        self.approach_sent = False
        self.return_sent = False
        self.fault_sent = False
        self.other_role = "rear" if self.is_front else "front"
        self.peer_robot_state = "UNKNOWN"
        self.peer_motion_phase = "UNKNOWN"
        self.mission_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(
            String, f"/{self.role}/robot_state", self.state_cb,
            STATE_LATEST_QOS)
        self.create_subscription(
            String, f"/{self.other_role}/robot_state",
            self.peer_state_cb, STATE_LATEST_QOS)
        self.create_subscription(
            String, f"/{self.other_role}/motion_phase",
            self.peer_phase_cb, 10)
        self.create_subscription(
            Odometry, f"/{self.role}/odom", self.odom_cb,
            SENSOR_LATEST_QOS)
        self.create_subscription(
            PoseStamped, "/parking/target_pose", self.target_cb, 10)
        self.create_subscription(
            PoseStamped, "/parking/slot_pose", self.slot_cb,
            self.mission_qos)
        self.create_subscription(
            String, "/parking/vehicle_spec", self.spec_cb,
            self.mission_qos)
        self.create_subscription(
            Bool, f"/{self.role}/wheel_detected", self.detected_cb, 10)
        self.create_subscription(
            Float64, f"/{self.role}/wheel_center_s", self.center_cb, 10)
        self.create_subscription(
            Float64, f"/{self.role}/wheel_lateral_offset",
            self.us_lateral_cb, SENSOR_LATEST_QOS)
        self.create_subscription(
            Bool, f"/{self.role}/wheel_lateral_valid",
            self.us_lateral_valid_cb, SENSOR_LATEST_QOS)
        self.create_subscription(
            Bool, f"/{self.role}/ultrasonic_ready",
            self.ultrasonic_ready_cb, STATE_LATEST_QOS)
        self.create_subscription(
            Bool, f"/{self.other_role}/ultrasonic_ready",
            self.peer_ultrasonic_ready_cb, STATE_LATEST_QOS)
        self.create_subscription(
            PoseStamped, "/sync/relative_pose", self.relative_pose_cb,
            SENSOR_LATEST_QOS)
        self.create_subscription(
            Bool, "/sync/marker_visible", self.relative_marker_cb,
            SENSOR_LATEST_QOS)
        if not self.is_front:
            self.create_subscription(
                Bool, "/rear/relative_vision_ready",
                self.relative_vision_ready_cb, STATE_LATEST_QOS)
        self.create_subscription(
            Bool, f"/{self.role}/cctv_marker_visible", self.top_marker_cb,
            SENSOR_LATEST_QOS)
        if self.is_front:
            self.create_subscription(
                Bool, "/rear/approach_done", self.rear_approach_cb, 10)
        else:
            self.create_subscription(
                Bool, "/front/approach_done", self.front_approach_cb, 10)
            self.create_subscription(
                Bool, "/align/front_done", self.front_done_cb, 10)

        self.pub_vel = self.create_publisher(
            TwistStamped, f"/{self.role}/cmd_vel", CMD_VEL_QOS)
        self.pub_scan_reset = self.create_publisher(
            Bool, f"/{self.role}/wheel_scan_reset", 10)
        self.pub_approach_done = self.create_publisher(
            Bool, f"/{self.role}/approach_done", 10)
        self.pub_return_done = self.create_publisher(
            Bool, f"/{self.role}/return_done", 10)
        self.pub_aligned = self.create_publisher(
            Bool, f"/{self.role}/wheel_aligned", 10)
        self.pub_phase = self.create_publisher(
            String, f"/{self.role}/motion_phase", 10)
        self.pub_fault = self.create_publisher(
            String, f"/{self.role}/motion_fault", 10)
        self.pub_active_target = self.create_publisher(
            PoseStamped, f"/{self.role}/active_target_pose", 10)
        self.pub_ultrasonic_enable = self.create_publisher(
            Bool, f"/{self.role}/ultrasonic_enable", 10)
        self.pub_relative_vision_enable = None
        if not self.is_front:
            self.pub_relative_vision_enable = self.create_publisher(
                Bool, "/rear/relative_vision_enable", STATE_LATEST_QOS)
            self.pub_relative_vision_enable.publish(Bool(data=False))

        self.create_timer(0.05, self.move_loop)
        self.create_timer(0.5, self.publish_phase)
        self.get_logger().info(
            f"individual_move underbody [{self.role}] | "
            f"entry={'simultaneous' if self.simultaneous_entry else 'front-first'} | "
            f"exit={'shared-front' if self.same_direction_exit else 'split'} | "
            f"wheelbase={self.wheelbase:.3f}m | "
            f"standoff={self.entry_standoff:.3f}m | "
            f"return_side={self.entry_side * self.entry_side_offset:+.3f}m")

    def _validate_parameters(self):
        positive = {
            "max_speed": self.max_speed,
            "scan_speed": self.scan_speed,
            "centerline_speed": self.centerline_speed,
            "default_wheelbase": self.wheelbase,
            "vehicle_half_length_m": self.vehicle_half_length,
            "vehicle_half_width_m": self.vehicle_half_width,
            "robot_length_m": self.robot_length,
            "robot_width_m": self.robot_width,
            "minimum_inter_robot_gap_m": self.minimum_inter_robot_gap,
            "robot_clearance_m": self.robot_clearance,
            "entry_standoff_m": self.entry_standoff,
            "entry_side_offset_m": self.entry_side_offset,
            "exit_distance_m": self.exit_distance,
            "exit_sync_gain": self.exit_sync_gain,
            "scan_overshoot_m": self.scan_overshoot,
            "aruco_timeout_s": self.aruco_timeout,
            "cctv_marker_timeout_s": self.cctv_marker_timeout,
            "relative_distance_tolerance_m":
                self.relative_distance_tolerance,
            "relative_lateral_tolerance_m":
                self.relative_lateral_tolerance,
            "relative_yaw_tolerance_deg": self.relative_yaw_tolerance,
            "relative_mismatch_timeout_s": self.relative_mismatch_timeout,
            "rear_min_scan_speed": self.rear_min_scan_speed,
            "rear_aruco_slowdown_window_m":
                self.rear_aruco_slowdown_window,
            "position_tolerance_m": self.position_tolerance,
            "centerline_tolerance_m": self.centerline_tolerance,
            "ultrasonic_lateral_timeout_s": self.us_lateral_timeout,
            "ultrasonic_lateral_yaw_gate_deg": self.us_lateral_yaw_gate,
            "ultrasonic_activation_timeout_s":
                self.ultrasonic_activation_timeout,
            "lateral_deviation_limit_m": self.deviation_limit,
            "center_tolerance_m": self.center_tolerance,
            "substate_timeout_s": self.substate_timeout,
            "target_timeout_s": self.target_timeout,
            "mission_data_timeout_s": self.mission_data_timeout,
            "odom_timeout_s": self.odom_timeout,
        }
        for name, value in positive.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if (not math.isfinite(self.future_tolerance) or
                self.future_tolerance < 0.0):
            raise ValueError("future_tolerance_s must be finite and non-negative")
        if self.entry_side not in (-1, 1):
            raise ValueError("entry_side must be -1 or 1")
        if self.same_direction_exit_sign not in (-1, 1):
            raise ValueError("same_direction_exit_sign must be -1 or 1")
        if self.prealign_hold_n <= 0:
            raise ValueError("prealign_hold_n must be positive")
        if self.deviation_n <= 0:
            raise ValueError("lateral_deviation_n must be positive")
        if self.max_scan_retry < 0:
            raise ValueError("max_scan_retry must be non-negative")
        if not math.isfinite(self.aruco_distance_offset):
            raise ValueError("aruco_distance_offset_m must be finite")
        if not 0.0 < self.marker_slowdown < self.marker_stop:
            raise ValueError(
                "need 0 < marker_slowdown_s < marker_stop_s")
        if not 0.0 < self.aruco_timeout < self.marker_stop:
            raise ValueError("aruco_timeout_s must be in (0, marker_stop_s)")
        if not 0.0 < self.rear_min_scan_speed <= self.scan_speed:
            raise ValueError(
                "rear_min_scan_speed must be in (0, scan_speed]")
        validate_wheelbase_clearance(
            self.wheelbase, self.robot_length,
            self.minimum_inter_robot_gap)
        protected_s = (
            self.vehicle_half_length + self.robot_length / 2.0 +
            self.robot_clearance)
        protected_d = (
            self.vehicle_half_width + self.robot_width / 2.0 +
            self.robot_clearance)
        if self.entry_standoff <= protected_s:
            raise ValueError(
                "entry_standoff_m must clear vehicle and robot half-length")
        if self.entry_side_offset <= protected_d:
            raise ValueError(
                "entry_side_offset_m must clear vehicle and robot half-width")
        if self.entry_standoff <= self.wheelbase / 2.0:
            raise ValueError("entry_standoff_m must lie beyond both axles")
        if self.wheelbase / 2.0 + self.exit_distance <= protected_s:
            raise ValueError(
                "exit_distance_m must carry an aligned robot beyond the "
                "protected vehicle envelope")

    @staticmethod
    def pose_from_msg(msg):
        p = msg.pose.position
        q = msg.pose.orientation
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if norm < 1e-6:
            return None
        yaw = math.atan2(
            2.0 * (q.w*q.z + q.x*q.y),
            1.0 - 2.0 * (q.y*q.y + q.z*q.z))
        values = (float(p.x), float(p.y), float(yaw))
        return values if all(math.isfinite(value) for value in values) else None

    def state_cb(self, msg):
        if msg.data == self.robot_state:
            return
        self.robot_state = msg.data
        if msg.data == "APPROACH":
            self.request_ultrasonic(False, force=True)
            self.approach_sent = False
            self.return_sent = False
            self.wheel_detected = False
            self.wheel_center_s = None
            self.us_lateral_valid = False
            self.us_lateral_value_time = 0.0
            self.us_lateral_valid_time = 0.0
            self.lateral_source = None
            self.prealign_ok_n = 0
            self.deviation_cnt = 0
            self.scan_retry = 0
            self.retreat_goal_s = None
            self.alignment_sent = False
            self.front_staged = False
            self.rear_observation_ready = False
            self.front_align_done = False
            self.relative_x = None
            self.relative_y = None
            self.relative_yaw = None
            self.relative_receipt_time = None
            self.relative_marker_visible = False
            self.top_marker_visible = False
            self.top_visibility_received = False
            self.top_marker_receipt_time = None
            self.last_visual_observation_time = None
            self.active_target = None
            self.route = []
            self.scan_origin_s = None
            self.relative_lost_since = None
            self.relative_mismatch_since = None
            self.motion_speed_scale = 1.0
            self.fault_sent = False
            self.set_phase(initial_approach_phase(
                self.role, self.simultaneous_entry))
        elif msg.data == "ALIGN":
            self.scan_origin_s = None
            self.prealign_ok_n = 0
            self.deviation_cnt = 0
            self.scan_retry = 0
            self.retreat_goal_s = None
            self.relative_mismatch_since = None
            self.set_phase(initial_align_phase(
                self.role, self.simultaneous_entry))
        elif msg.data == "RETURN":
            self.return_sent = False
            self.route = []
            if self.same_direction_exit:
                self.set_phase("WAIT_PEER_RETURN")
            else:
                self.start_exit()
        else:
            self.request_ultrasonic(False, force=True)
            self.set_phase(msg.data)

    def odom_cb(self, msg):
        accepted, reason = self.odom_gate.accept(
            stamp_to_ns(msg.header.stamp),
            self.get_clock().now().nanoseconds)
        if not accepted:
            self.get_logger().warn(
                f"[{self.role}] odom rejected: {reason}",
                throttle_duration_sec=2.0)
            return
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.x = float(p.x)
        self.y = float(p.y)
        self.theta = math.atan2(
            2.0 * (q.w*q.z + q.x*q.y),
            1.0 - 2.0 * (q.y*q.y + q.z*q.z))
        self.odom_ready = all(
            math.isfinite(value) for value in (self.x, self.y, self.theta))
        if self.odom_ready:
            self.last_odom_time = time.monotonic()

    def peer_state_cb(self, msg):
        self.peer_robot_state = str(msg.data)

    def peer_phase_cb(self, msg):
        self.peer_motion_phase = str(msg.data)

    def target_cb(self, msg):
        if msg.header.frame_id not in ("", "map"):
            return
        accepted, reason = self.target_gate.accept(
            stamp_to_ns(msg.header.stamp),
            self.get_clock().now().nanoseconds)
        if not accepted:
            self.get_logger().warn(
                f"[{self.role}] target rejected: {reason}",
                throttle_duration_sec=2.0)
            return
        target = self.pose_from_msg(msg)
        if target is None:
            self.get_logger().error(
                f"[{self.role}] invalid target pose ignored",
                throttle_duration_sec=2.0)
            return
        self.latest_target = target
        self.latest_target_time = time.monotonic()

    def slot_cb(self, msg):
        if msg.header.frame_id not in ("", "map"):
            return
        accepted, reason = self.slot_gate.accept(
            stamp_to_ns(msg.header.stamp),
            self.get_clock().now().nanoseconds)
        if not accepted:
            self.get_logger().warn(
                f"[{self.role}] slot rejected: {reason}",
                throttle_duration_sec=2.0)
            return
        target = self.pose_from_msg(msg)
        if target is not None:
            self.slot_target = target

    def spec_cb(self, msg):
        if not self.use_vehicle_spec_wheelbase:
            return
        try:
            payload = json.loads(msg.data)
            candidate = float(payload["wheelbase"])
            accepted, reason = self.spec_gate.accept(
                int(payload["stamp_ns"]),
                self.get_clock().now().nanoseconds)
            if not accepted:
                raise ValueError(f"vehicle_spec {reason}")
            if not math.isfinite(candidate) or candidate <= 0.0:
                raise ValueError("wheelbase must be finite and positive")
            validate_wheelbase_clearance(
                candidate, self.robot_length,
                self.minimum_inter_robot_gap)
            if self.entry_standoff <= candidate / 2.0:
                raise ValueError("wheelbase places axle beyond standoff")
            if (candidate / 2.0 + self.exit_distance <=
                    self.vehicle_half_length + self.robot_length / 2.0 +
                    self.robot_clearance):
                raise ValueError(
                    "wheelbase and exit_distance leave robot under vehicle")
            self.wheelbase = candidate
            self.get_logger().info(
                f"[{self.role}] vehicle_spec wheelbase={candidate:.3f}m")
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warn(
                f"[{self.role}] invalid vehicle_spec ignored: {exc}",
                throttle_duration_sec=2.0)

    def detected_cb(self, msg):
        if msg.data:
            self.wheel_detected = True

    def center_cb(self, msg):
        value = float(msg.data)
        if math.isfinite(value):
            self.wheel_center_s = value

    def ultrasonic_ready_cb(self, msg):
        self.ultrasonic_ready = bool(msg.data)

    def peer_ultrasonic_ready_cb(self, msg):
        self.peer_ultrasonic_ready = bool(msg.data)

    def request_ultrasonic(self, enabled, *, force=False):
        enabled = bool(enabled)
        if (not force and
                enabled == getattr(self, 'ultrasonic_requested', False)):
            return
        self.ultrasonic_requested = enabled
        if not enabled:
            self.ultrasonic_ready = False
        publisher = getattr(self, 'pub_ultrasonic_enable', None)
        if publisher is not None:
            publisher.publish(Bool(data=enabled))
            self.get_logger().info(
                f"[{self.role}] ultrasonic "
                f"{'enable' if enabled else 'disable'}")

    def front_done_cb(self, msg):
        if msg.data:
            self.front_align_done = True

    def front_approach_cb(self, msg):
        if msg.data:
            self.front_staged = True

    def rear_approach_cb(self, msg):
        if msg.data:
            self.rear_observation_ready = True

    def peer_staged(self):
        if self.is_front:
            return self.rear_observation_ready
        return self.front_staged

    def relative_pose_cb(self, msg):
        if msg.header.frame_id != "rear_base":
            self.get_logger().warn(
                "relative pose rejected: WRONG_FRAME",
                throttle_duration_sec=2.0)
            return
        raw_distance = float(msg.pose.position.x)
        lateral = float(msg.pose.position.y)
        q = msg.pose.orientation
        quaternion = (float(q.x), float(q.y), float(q.z), float(q.w))
        if not all(math.isfinite(value) for value in quaternion):
            self.get_logger().warn(
                "relative pose rejected: INVALID_QUATERNION",
                throttle_duration_sec=2.0)
            return
        norm = math.sqrt(sum(value * value for value in quaternion))
        if norm < 1e-6:
            self.get_logger().warn(
                "relative pose rejected: INVALID_QUATERNION",
                throttle_duration_sec=2.0)
            return
        qx, qy, qz, qw = (value / norm for value in quaternion)
        yaw = math.atan2(
            2.0 * (qw*qz + qx*qy),
            1.0 - 2.0 * (qy*qy + qz*qz))
        corrected_distance = raw_distance + self.aruco_distance_offset
        values = (corrected_distance, lateral, yaw)
        if not all(math.isfinite(value) for value in values):
            return
        if corrected_distance <= 0.0:
            return
        accepted, reason = self.relative_gate.accept(
            stamp_to_ns(msg.header.stamp),
            self.get_clock().now().nanoseconds)
        if not accepted:
            self.get_logger().warn(
                f"relative pose rejected: {reason}",
                throttle_duration_sec=2.0)
            return
        now = time.monotonic()
        self.relative_x = corrected_distance
        self.relative_y = lateral
        self.relative_yaw = angle_norm(yaw)
        self.relative_receipt_time = now
        self.last_visual_observation_time = now

    def relative_marker_cb(self, msg):
        self.relative_marker_visible = bool(msg.data)

    def relative_vision_ready_cb(self, msg):
        self.relative_vision_ready = bool(msg.data)

    def top_marker_cb(self, msg):
        now = time.monotonic()
        self.top_marker_visible = bool(msg.data)
        self.top_visibility_received = True
        self.top_marker_receipt_time = now
        if self.top_marker_visible:
            self.last_visual_observation_time = now

    def top_marker_is_fresh(self, now=None):
        now = time.monotonic() if now is None else float(now)
        return (
            self.top_visibility_received and
            self.top_marker_visible and
            self.top_marker_receipt_time is not None and
            0.0 <= now - self.top_marker_receipt_time <
            self.cctv_marker_timeout)

    def relative_is_fresh(self):
        if (not self.relative_marker_visible or
                self.relative_receipt_time is None or
                self.relative_x is None):
            return False
        age = time.monotonic() - self.relative_receipt_time
        return 0.0 <= age < self.aruco_timeout

    def underbody_visual_required(self):
        return self.phase in (
            "READY_TO_SCAN", "PRE_ALIGN", "WAIT_ULTRASONIC_READY",
            "PREALIGNED",
            "SCAN_IN", "RETREAT",
            "CENTER_AXLE", "ALIGNED")

    def relative_vision_required(self, phase=None):
        return (not self.is_front and
                (self.phase if phase is None else phase) in
                REAR_RELATIVE_VISION_PHASES)

    def publish_relative_vision_request(self):
        if self.pub_relative_vision_enable is None:
            return
        enabled = self.relative_vision_required()
        self.pub_relative_vision_enable.publish(Bool(data=enabled))
        if not enabled:
            # Never let a previous mission's latched readiness/pose reopen a
            # later motion gate while the perception pipeline is in standby.
            self.relative_vision_ready = False
            self.relative_marker_visible = False
            self.relative_receipt_time = None
            self.relative_x = None
            self.relative_y = None
            self.relative_yaw = None

    def relative_vision_observation_ready(self):
        return (self.is_front or
                (self.relative_vision_ready and self.relative_is_fresh()))

    def publish_approach_ready_if_observed(self):
        if self.approach_sent:
            return True
        if not self.relative_vision_observation_ready():
            self.stop()
            return False
        self.pub_approach_done.publish(Bool(data=True))
        self.approach_sent = True
        return True

    def update_visual_fallback(self):
        """Prefer top pose outside and ID0 inside, then bound encoder fallback."""
        if not self.underbody_visual_required():
            self.relative_lost_since = None
            self.motion_speed_scale = 1.0
            return True
        now = time.monotonic()
        top_usable = self.top_marker_is_fresh(now)
        if top_usable or self.relative_is_fresh():
            self.relative_lost_since = None
            self.motion_speed_scale = 1.0
            return True
        if self.relative_lost_since is None:
            self.relative_lost_since = (
                self.last_visual_observation_time
                if self.last_visual_observation_time is not None else now)
        lost_age = now - self.relative_lost_since
        self.motion_speed_scale = marker_loss_speed_scale(
            lost_age, self.marker_slowdown, self.marker_stop)
        if self.motion_speed_scale <= 0.0:
            # Missing visual evidence is recoverable. Hold this local motion
            # instead of publishing motion_fault, which the Robot FSM promotes
            # to the global emergency-stop channel.
            self.stop()
            return False
        return True

    def us_lateral_cb(self, msg):
        value = float(msg.data)
        if not math.isfinite(value):
            return
        self.us_lateral = value
        self.us_lateral_value_time = time.monotonic()

    def us_lateral_valid_cb(self, msg):
        self.us_lateral_valid = bool(msg.data)
        self.us_lateral_valid_time = time.monotonic()

    def ultrasonic_lateral(self, odom_lateral, yaw_error):
        """Return lateral error and the source used by both control and gate."""
        now = time.monotonic()
        if (self.use_us_lateral and self.us_lateral_valid
                and abs(yaw_error) <= self.us_lateral_yaw_gate
                and now - self.us_lateral_value_time <= self.us_lateral_timeout
                and now - self.us_lateral_valid_time <= self.us_lateral_timeout):
            return self.us_lateral, "ultrasonic"
        return odom_lateral, "odom"

    def relative_axis_errors(self):
        """Return corrections that move this robot toward the other robot."""
        if not self.relative_is_fresh():
            return 0.0, 0.0
        # Pose is Front in Rear frame. Rear follows +error; Front follows -error.
        direction = -1.0 if self.is_front else 1.0
        return direction * self.relative_y, direction * self.relative_yaw

    def final_relative_check(self):
        """Validate Rear after ultrasonic centering; never command distance."""
        now = time.monotonic()
        if not self.relative_is_fresh():
            if self.relative_mismatch_since is None:
                self.relative_mismatch_since = now
            elif now - self.relative_mismatch_since > self.relative_mismatch_timeout:
                self.get_logger().warn(
                    "FINAL_ID0_MISSING: holding alignment without E-stop",
                    throttle_duration_sec=2.0)
            return False
        consistent = relative_alignment_is_consistent(
            self.relative_x, self.relative_yaw, self.wheelbase,
            self.relative_distance_tolerance, self.relative_yaw_tolerance,
            relative_lateral=self.relative_y,
            lateral_tolerance=self.relative_lateral_tolerance)
        if consistent:
            self.relative_mismatch_since = None
            return True
        if self.relative_mismatch_since is None:
            self.relative_mismatch_since = now
        elif now - self.relative_mismatch_since > self.relative_mismatch_timeout:
            self.get_logger().warn(
                "FINAL_RELATIVE_CHECK_FAILED:"
                f"distance={self.relative_x:.3f},"
                f"lateral={self.relative_y:.3f},"
                f"yaw={math.degrees(self.relative_yaw):.2f}; "
                "holding alignment without E-stop",
                throttle_duration_sec=2.0)
        return False

    def set_phase(self, phase):
        if phase != self.phase:
            self.get_logger().info(
                f"[{self.role}] motion {self.phase} -> {phase}")
        self.phase = phase
        self.phase_enter_time = time.monotonic()
        self.publish_relative_vision_request()
        self.publish_phase()

    def publish_phase(self):
        self.pub_phase.publish(String(data=self.phase))
        self.publish_active_target()
        self.publish_relative_vision_request()
        # Coordination Bool topics are volatile. Repeating READY avoids a
        # startup/state-transition race where the peer resets after the first
        # one-shot message.
        if (self.approach_sent and
                self.robot_state in ("APPROACH", "ALIGN")):
            self.pub_approach_done.publish(Bool(data=True))

    def publish_active_target(self):
        """Publish the exact frame used by motion for the edge detector."""
        if self.active_target is None:
            return
        x, y, yaw = self.active_target
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self.pub_active_target.publish(msg)

    def fault(self, reason):
        self.stop()
        self.request_ultrasonic(False, force=True)
        if self.fault_sent:
            return
        self.fault_sent = True
        self.set_phase("FAULT")
        self.pub_fault.publish(String(data=reason))
        self.get_logger().error(f"[{self.role}] motion fault: {reason}")

    def phase_timed_out(self):
        if time.monotonic() - self.phase_enter_time <= self.substate_timeout:
            return False
        self.fault(f"{self.phase}_TIMEOUT")
        return True

    def latch_target_and_plan(self):
        if self.latest_target is None:
            if self.phase_timed_out():
                return False
            self.stop()
            return False
        if time.monotonic() - self.latest_target_time > self.target_timeout:
            self.fault("TARGET_POSE_STALE")
            return False
        self.active_target = self.latest_target
        tx, ty, yaw = self.active_target
        start = world_to_vehicle(self.x, self.y, tx, ty, yaw)
        goal = (
            approach_longitudinal(
                self.role, self.entry_standoff, self.wheelbase),
            0.0,
        )
        protected_s = (
            self.vehicle_half_length + self.robot_length / 2.0 +
            self.robot_clearance)
        protected_d = (
            self.vehicle_half_width + self.robot_width / 2.0 +
            self.robot_clearance)
        if start[0] > -protected_s:
            self.fault(
                "APPROACH_START_NOT_BEHIND_VEHICLE:"
                f"s={start[0]:.3f},limit={-protected_s:.3f}")
            return False
        if segment_intersects_open_rect(
                start, goal, protected_s, protected_d):
            self.fault("APPROACH_NOT_LONGITUDINAL_SAFE")
            return False
        self.route = [vehicle_to_world(goal[0], goal[1], tx, ty, yaw)]
        self.set_phase("TO_REAR_STAGING")
        return True

    def advance_route(self, speed, goal_yaw=None):
        if not self.route:
            return True
        gx, gy = self.route[0]
        arrived = self.move_pose_toward(
            gx, gy, goal_yaw, speed, self.position_tolerance)
        if arrived:
            self.route.pop(0)
            self.phase_enter_time = time.monotonic()
        return not self.route

    def move_pose_toward(self, gx, gy, goal_yaw, speed, position_tolerance):
        dx = gx - self.x
        dy = gy - self.y
        distance = math.hypot(dx, dy)
        msg = self.new_velocity_command()
        if distance > position_tolerance:
            scale = min(1.0, distance / 0.10)
            vx_world = (
                self.motion_speed_scale * speed * dx / distance * scale)
            vy_world = (
                self.motion_speed_scale * speed * dy / distance * scale)
            c = math.cos(self.theta)
            s = math.sin(self.theta)
            msg.twist.linear.x = c * vx_world + s * vy_world
            msg.twist.linear.y = -s * vx_world + c * vy_world
        yaw_done = True
        if goal_yaw is not None:
            yaw_error = angle_norm(goal_yaw - self.theta)
            msg.twist.angular.z = self.motion_speed_scale * max(
                -self.max_yaw_rate,
                min(self.max_yaw_rate, self.yaw_gain * yaw_error))
            yaw_done = abs(yaw_error) <= self.yaw_tolerance
        self.pub_vel.publish(msg)
        return distance <= position_tolerance and yaw_done

    def command_vehicle_axis(self, longitudinal_speed):
        tx, ty, vehicle_yaw = self.active_target
        _, lateral = world_to_vehicle(
            self.x, self.y, tx, ty, vehicle_yaw)
        yaw_error = angle_norm(vehicle_yaw - self.theta)
        lateral, source = self.ultrasonic_lateral(lateral, yaw_error)
        if source != self.lateral_source:
            self.lateral_source = source
            self.get_logger().info(
                f"[{self.role}] lateral source -> {source}")
        relative_lateral, relative_yaw = self.relative_axis_errors()
        lateral_speed = max(
            -self.max_lateral_speed,
            min(self.max_lateral_speed,
                -self.lateral_gain * lateral +
                self.relative_lateral_gain * relative_lateral))
        longitudinal_speed *= self.motion_speed_scale
        lateral_speed *= self.motion_speed_scale
        c_vehicle = math.cos(vehicle_yaw)
        s_vehicle = math.sin(vehicle_yaw)
        vx_world = (
            c_vehicle * longitudinal_speed - s_vehicle * lateral_speed)
        vy_world = (
            s_vehicle * longitudinal_speed + c_vehicle * lateral_speed)
        c_robot = math.cos(self.theta)
        s_robot = math.sin(self.theta)
        msg = self.new_velocity_command()
        msg.twist.linear.x = c_robot * vx_world + s_robot * vy_world
        msg.twist.linear.y = -s_robot * vx_world + c_robot * vy_world
        msg.twist.angular.z = self.motion_speed_scale * max(
            -self.max_yaw_rate,
            min(self.max_yaw_rate,
                self.yaw_gain * yaw_error +
                self.relative_yaw_gain * relative_yaw))
        self.pub_vel.publish(msg)

    def current_vehicle_pose(self):
        """Return vehicle-frame pose using the same lateral source as control."""
        tx, ty, yaw = self.active_target
        longitudinal, lateral = world_to_vehicle(
            self.x, self.y, tx, ty, yaw)
        yaw_error = angle_norm(self.theta - yaw)
        lateral, _ = self.ultrasonic_lateral(lateral, yaw_error)
        return longitudinal, lateral, yaw_error

    def start_exit(self):
        if not self.odom_ready:
            self.set_phase("WAIT_EXIT_ODOM")
            return
        self.exit_yaw = self.theta
        translation = exit_longitudinal_translation(
            self.role,
            self.exit_distance,
            self.wheelbase,
            self.same_direction_exit,
            self.same_direction_exit_sign,
        )
        self.exit_goal = (
            self.x + translation * math.cos(self.exit_yaw),
            self.y + translation * math.sin(self.exit_yaw),
        )
        side = self.entry_side * self.entry_side_offset
        self.side_exit_goal = (
            self.exit_goal[0] - math.sin(self.exit_yaw) * side,
            self.exit_goal[1] + math.cos(self.exit_yaw) * side,
        )
        self.set_phase("EXIT_UNDERBODY")

    def synchronized_exit_speed(self):
        speed = self.centerline_speed
        if (not self.same_direction_exit or self.is_front or
                not self.relative_is_fresh()):
            return speed
        correction = self.exit_sync_gain * (
            self.relative_x - self.wheelbase)
        return max(0.5 * speed, min(1.5 * speed, speed + correction))

    def peer_reached_phase(self, *phases):
        return self.peer_motion_phase in phases

    def plan_return_home(self):
        if self.slot_target is None:
            self.fault("RETURN_SLOT_POSE_MISSING")
            return
        tx, ty, yaw = self.slot_target
        start = world_to_vehicle(self.x, self.y, tx, ty, yaw)
        goal = world_to_vehicle(
            self.wait_pos[0], self.wait_pos[1], tx, ty, yaw)
        try:
            route_sd = plan_around_vehicle(
                start,
                goal,
                self.vehicle_half_length + self.robot_length / 2.0 +
                self.robot_clearance,
                self.vehicle_half_width + self.robot_width / 2.0 +
                self.robot_clearance,
            )
        except ValueError as exc:
            self.fault(f"RETURN_ROUTE_INVALID:{exc}")
            return
        self.route = [
            vehicle_to_world(s, d, tx, ty, yaw) for s, d in route_sd]
        self.set_phase("RETURN_HOME")

    def stop(self):
        self.pub_vel.publish(self.new_velocity_command())

    def new_velocity_command(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f"{self.role}_base"
        return msg

    def move_loop(self):
        if not self.odom_ready or self.phase == "FAULT":
            return
        if (self.robot_state in ("APPROACH", "ALIGN", "RETURN") and
                time.monotonic() - self.last_odom_time > self.odom_timeout):
            self.fault("ODOM_TIMEOUT")
            return
        if (self.robot_state in ("APPROACH", "ALIGN") and
                not self.update_visual_fallback()):
            return

        if self.robot_state == "APPROACH":
            self.run_approach()
        elif self.robot_state == "ALIGN":
            self.run_align()
        elif self.robot_state == "RETURN":
            self.run_return()

    def run_approach(self):
        if self.phase == "WAIT_FRONT_STAGED":
            self.stop()
            if self.front_staged:
                self.set_phase("WAIT_TARGET")
            elif self.phase_timed_out():
                return
            return
        if self.phase == "WAIT_TARGET":
            self.latch_target_and_plan()
            return
        if self.phase_timed_out():
            return
        if self.phase == "TO_REAR_STAGING":
            yaw = self.active_target[2]
            if self.advance_route(self.centerline_speed, goal_yaw=yaw):
                self.stop()
                self.set_phase("READY_TO_SCAN")
                self.publish_approach_ready_if_observed()
            return
        if self.phase == "READY_TO_SCAN":
            self.stop()
            self.publish_approach_ready_if_observed()

    def run_align(self):
        if self.active_target is None:
            self.fault("ALIGN_WITHOUT_TARGET")
            return
        if (self.simultaneous_entry
                and self.peer_motion_phase == "RETREAT"):
            if self.phase == "ALIGNED":
                self.fault("PEER_RETREAT_AFTER_LOCAL_ALIGNMENT")
                return
            if self.phase in ("SCAN_IN", "CENTER_AXLE"):
                self.begin_scan_retreat("peer retreat")
                return
        if self.phase == "ALIGNED":
            self.command_vehicle_axis(0.0)
            return
        if self.phase_timed_out():
            return
        if self.phase == "WAIT_REAR_OBSERVATION":
            self.command_vehicle_axis(0.0)
            if self.rear_observation_ready and self.relative_is_fresh():
                self.enter_prealign()
            return
        if self.phase == "WAIT_FRONT_ALIGNED":
            self.command_vehicle_axis(0.0)
            if self.front_align_done and self.relative_is_fresh():
                self.enter_prealign()
            return
        if self.phase == "WAIT_PEER_STAGED":
            self.command_vehicle_axis(0.0)
            if self.peer_staged() and self.relative_is_fresh():
                self.enter_prealign()
            return

        longitudinal, lateral, yaw_error = self.current_vehicle_pose()
        if self.phase == "PRE_ALIGN":
            self.command_vehicle_axis(0.0)
            if (abs(lateral) <= self.centerline_tolerance
                    and abs(yaw_error) <= self.yaw_tolerance):
                self.prealign_ok_n += 1
            else:
                self.prealign_ok_n = 0
            if self.prealign_ok_n >= self.prealign_hold_n:
                self.get_logger().info(
                    f"[{self.role}] pre-align done: "
                    f"d={lateral * 1000:.1f}mm, "
                    f"yaw={math.degrees(yaw_error):.2f}deg")
                self.prealign_ok_n = 0
                self.deviation_cnt = 0
                self.request_ultrasonic(True)
                self.set_phase("WAIT_ULTRASONIC_READY")
            return

        if self.phase == "WAIT_ULTRASONIC_READY":
            self.command_vehicle_axis(0.0)
            if self.ultrasonic_ready:
                self.set_phase(
                    "PREALIGNED" if self.simultaneous_entry
                    else "SCAN_IN")
            elif (time.monotonic() - self.phase_enter_time >=
                  self.ultrasonic_activation_timeout):
                self.fault("ULTRASONIC_ACTIVATION_TIMEOUT")
            return

        if self.phase == "PREALIGNED":
            self.command_vehicle_axis(0.0)
            if (self.peer_robot_state == "ALIGN" and
                    self.peer_motion_phase in ("PREALIGNED", "SCAN_IN") and
                    self.ultrasonic_ready and
                    self.peer_ultrasonic_ready):
                self.set_phase("SCAN_IN")
            return

        if self.phase == "RETREAT":
            if (self.retreat_goal_s is None
                    or abs(longitudinal - self.retreat_goal_s)
                    <= self.center_tolerance):
                self.stop()
                self.scan_origin_s = None
                self.retreat_goal_s = None
                self.wheel_detected = False
                self.wheel_center_s = None
                self.pub_scan_reset.publish(Bool(data=True))
                self.enter_prealign()
                return
            self.command_vehicle_axis(
                -scan_direction(self.role) * self.scan_speed)
            return

        if self.phase == "SCAN_IN":
            if not self.ultrasonic_ready:
                self.stop()
                self.fault("ULTRASONIC_LOST_DURING_SCAN")
                return
            if self.scan_origin_s is None:
                self.scan_origin_s = longitudinal
            if abs(lateral) > self.deviation_limit:
                self.deviation_cnt += 1
            else:
                self.deviation_cnt = 0
            if self.deviation_cnt >= self.deviation_n:
                self.deviation_cnt = 0
                if (self.simultaneous_entry
                        and self.peer_motion_phase == "ALIGNED"):
                    self.fault("LATERAL_DEVIATION_AFTER_PEER_ALIGNMENT")
                    return
                if self.scan_retry >= self.max_scan_retry:
                    self.fault("LATERAL_DEVIATION")
                    return
                self.scan_retry += 1
                self.begin_scan_retreat(
                    "lateral deviation "
                    f"{lateral * 1000:.1f}mm > "
                    f"{self.deviation_limit * 1000:.0f}mm "
                    f"({self.scan_retry}/{self.max_scan_retry})")
                return
            if self.wheel_detected:
                if self.wheel_center_s is None:
                    self.command_vehicle_axis(0.0)
                    return
                self.set_phase("CENTER_AXLE")
                return
            max_travel = (
                abs(standoff_longitudinal(
                    self.role, self.entry_standoff) -
                    axle_longitudinal(self.role, self.wheelbase)) +
                self.scan_overshoot)
            if not self.is_front:
                max_travel = (
                    abs(approach_longitudinal(
                        self.role, self.entry_standoff, self.wheelbase) -
                        axle_longitudinal(self.role, self.wheelbase)) +
                    self.scan_overshoot)
            if abs(longitudinal - self.scan_origin_s) > max_travel:
                self.fault("WHEEL_PAIR_NOT_DETECTED")
                return
            active_scan_speed = self.scan_speed
            if not self.is_front and self.relative_is_fresh():
                active_scan_speed = rear_scan_speed_from_relative(
                    self.relative_x, self.wheelbase, self.scan_speed,
                    self.rear_min_scan_speed,
                    self.rear_aruco_slowdown_window,
                    self.scan_overshoot)
                if active_scan_speed is None:
                    self.fault("ARUCO_DISTANCE_GUARD")
                    return
            self.command_vehicle_axis(
                scan_direction(self.role) * active_scan_speed)
            return

        if self.phase == "CENTER_AXLE":
            if not self.ultrasonic_ready:
                self.stop()
                self.fault("ULTRASONIC_LOST_DURING_CENTERING")
                return
            error = self.wheel_center_s - longitudinal
            # Both final axle states require a live ID0 yaw observation.
            # Front does not check distance yet because Rear is still queued;
            # Rear additionally performs final_relative_check below.
            relative_yaw_ok = (
                self.relative_is_fresh() and
                abs(self.relative_yaw) <= self.relative_yaw_tolerance)
            aligned = (
                abs(error) <= self.center_tolerance and
                abs(lateral) <= self.centerline_tolerance and
                abs(yaw_error) <= self.yaw_tolerance and
                relative_yaw_ok)
            if aligned:
                if not self.is_front and not self.final_relative_check():
                    self.command_vehicle_axis(0.0)
                    return
                self.stop()
                self.request_ultrasonic(False)
                self.set_phase("ALIGNED")
                if not self.alignment_sent:
                    self.pub_aligned.publish(Bool(data=True))
                    self.alignment_sent = True
                    self.get_logger().info(
                        f"[{self.role}] axle aligned: "
                        f"ds={error * 1000:.1f}mm, "
                        f"d={lateral * 1000:.1f}mm, "
                        f"yaw={math.degrees(yaw_error):.2f}deg")
                return
            if abs(error) <= self.center_tolerance:
                # Hold longitudinal position while lateral/yaw servos finish.
                speed = 0.0
            else:
                speed = max(
                    -self.center_speed,
                    min(self.center_speed, self.center_gain * error))
                if abs(speed) < 0.004:
                    speed = math.copysign(0.004, error)
            self.command_vehicle_axis(speed)
            return

    def enter_prealign(self):
        self.prealign_ok_n = 0
        self.deviation_cnt = 0
        self.set_phase("PRE_ALIGN")

    def begin_scan_retreat(self, reason):
        if self.scan_origin_s is None:
            self.fault("RETREAT_WITHOUT_SCAN_ORIGIN")
            return
        self.stop()
        self.wheel_detected = False
        self.wheel_center_s = None
        self.retreat_goal_s = self.scan_origin_s
        self.get_logger().error(
            f"[{self.role}] scan retreat: {reason}")
        self.set_phase("RETREAT")

    def run_return(self):
        if self.phase == "WAIT_PEER_RETURN":
            self.stop()
            if (self.peer_robot_state == "RETURN" or
                    self.peer_reached_phase(
                        "WAIT_PEER_RETURN", "EXIT_UNDERBODY",
                        "WAIT_PEER_EXIT_CLEAR", "EXIT_TO_SIDE",
                        "WAIT_PEER_SIDE_CLEAR", "RETURN_HOME")):
                self.start_exit()
            return
        if self.phase == "WAIT_EXIT_ODOM":
            self.start_exit()
            return
        if self.phase_timed_out():
            return
        if self.phase == "EXIT_UNDERBODY":
            if self.move_pose_toward(
                    self.exit_goal[0], self.exit_goal[1], self.exit_yaw,
                    self.synchronized_exit_speed(),
                    self.position_tolerance):
                if self.same_direction_exit:
                    self.set_phase("WAIT_PEER_EXIT_CLEAR")
                else:
                    self.set_phase("EXIT_TO_SIDE")
            return
        if self.phase == "WAIT_PEER_EXIT_CLEAR":
            self.stop()
            if self.peer_reached_phase(
                    "WAIT_PEER_EXIT_CLEAR", "EXIT_TO_SIDE",
                    "WAIT_PEER_SIDE_CLEAR", "RETURN_HOME", "RETURNED"):
                self.set_phase("EXIT_TO_SIDE")
            return
        if self.phase == "EXIT_TO_SIDE":
            if self.move_pose_toward(
                    self.side_exit_goal[0], self.side_exit_goal[1],
                    self.exit_yaw, self.centerline_speed,
                    self.position_tolerance):
                if self.same_direction_exit:
                    self.set_phase("WAIT_PEER_SIDE_CLEAR")
                else:
                    self.plan_return_home()
            return
        if self.phase == "WAIT_PEER_SIDE_CLEAR":
            self.stop()
            if self.peer_reached_phase(
                    "WAIT_PEER_SIDE_CLEAR", "RETURN_HOME", "RETURNED"):
                self.plan_return_home()
            return
        if self.phase == "RETURN_HOME":
            if self.advance_route(self.max_speed):
                self.stop()
                self.set_phase("RETURNED")
                if not self.return_sent:
                    self.pub_return_done.publish(Bool(data=True))
                    self.return_sent = True
            return
        if self.phase == "RETURNED":
            self.stop()


def main(args=None):
    rclpy.init(args=args)
    node = IndividualMoveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
