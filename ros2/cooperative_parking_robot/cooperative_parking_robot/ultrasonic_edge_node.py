#!/usr/bin/env python3
"""Pair left/right ultrasonic wheel edges in the target-vehicle frame.

STM32 owns HC-SR04 timing and publishes Range messages through the bridge.
This node projects robot odometry onto the motion node's latched vehicle axis,
applies each sensor-to-gripper mounting offset, and publishes an axle target
``s`` coordinate. It only accepts samples while ALIGN is active and while the
robot yaw is close enough to the vehicle yaw for the projection to be valid.
"""

import json
import math
import time
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, Float64, Int32, String

from cooperative_parking_robot.vehicle_entry import (
    DEFAULT_WHEELBASE_M,
    angle_norm,
    projected_robot_x_offset,
    scan_direction,
    target_axle_index,
    vehicle_to_world,
    world_to_vehicle,
)
from cooperative_parking_robot.freshness import StampGate, stamp_to_ns
from cooperative_parking_robot.latest_qos import (
    SENSOR_LATEST_QOS,
    STATE_LATEST_QOS,
)
from cooperative_parking_robot.wheel_edge_detector import AxleSequenceDetector


def paired_lateral_offset(left, right, threshold, sign=1.0):
    """Return vehicle-frame lateral error when both wheel echoes are valid."""
    values = (float(left), float(right), float(threshold), float(sign))
    if not all(math.isfinite(value) for value in values):
        return None
    left, right, threshold, sign = values
    if threshold <= 0.0 or sign not in (-1.0, 1.0):
        raise ValueError("invalid lateral ultrasonic calibration")
    if left < 0.0 or right < 0.0:
        return None
    if left < threshold and right < threshold:
        return sign * (right - left) / 2.0
    return None


class UltrasonicEdgeNode(Node):
    def __init__(self, **kwargs):
        super().__init__("ultrasonic_edge_node", **kwargs)
        self.declare_parameter("role", "front")
        self.declare_parameter("threshold_m", 0.10)
        self.declare_parameter("lateral_median_n", 3)
        self.declare_parameter("lateral_pair_timeout_s", 0.20)
        self.declare_parameter("lateral_sign", 1.0)
        self.declare_parameter("exit_hysteresis_m", 0.02)
        self.declare_parameter("window_size", 3)
        self.declare_parameter("pair_timeout_s", 1.0)
        self.declare_parameter("sensor_timeout_s", 0.50)
        self.declare_parameter("max_sensor_yaw_error_deg", 10.0)
        self.declare_parameter("left_sensor_to_gripper_x_m", 0.0)
        self.declare_parameter("right_sensor_to_gripper_x_m", 0.0)
        self.declare_parameter("target_axle_index", 0)
        self.declare_parameter(
            "expected_axle_spacing_m", DEFAULT_WHEELBASE_M)
        self.declare_parameter("axle_spacing_tolerance_m", 0.15)
        # Provisional demo window. Final value must include CCTV target-pose and
        # ultrasonic mounting repeatability measured on the physical vehicle.
        self.declare_parameter("axle_position_tolerance_m", 0.15)
        self.declare_parameter("use_vehicle_spec_wheelbase", True)
        self.declare_parameter("odom_timeout_s", 0.50)
        self.declare_parameter("future_tolerance_s", 0.10)

        gp = self.get_parameter
        self.role = str(gp("role").value)
        if self.role not in ("front", "rear"):
            raise ValueError("role must be 'front' or 'rear'")
        self.sensor_timeout = float(gp("sensor_timeout_s").value)
        self.max_sensor_yaw_error = math.radians(
            float(gp("max_sensor_yaw_error_deg").value))
        if self.sensor_timeout <= 0.0:
            raise ValueError("sensor_timeout_s must be positive")
        if self.max_sensor_yaw_error <= 0.0:
            raise ValueError("max_sensor_yaw_error_deg must be positive")
        self.sensor_to_gripper_x = {
            "left": float(gp("left_sensor_to_gripper_x_m").value),
            "right": float(gp("right_sensor_to_gripper_x_m").value),
        }
        configured_target = int(gp("target_axle_index").value)
        self.target_axle = (
            configured_target if configured_target > 0 else
            target_axle_index(self.role))
        self.expected_axle_spacing = float(
            gp("expected_axle_spacing_m").value)
        self.axle_spacing_tolerance = float(
            gp("axle_spacing_tolerance_m").value)
        self.axle_position_tolerance = float(
            gp("axle_position_tolerance_m").value)
        self.use_vehicle_spec_wheelbase = bool(
            gp("use_vehicle_spec_wheelbase").value)
        if self.expected_axle_spacing <= 0.0:
            raise ValueError("expected_axle_spacing_m must be positive")
        if (self.axle_spacing_tolerance <= 0.0 or
                self.axle_position_tolerance <= 0.0):
            raise ValueError("axle tolerances must be positive")
        self.odom_timeout = float(gp("odom_timeout_s").value)
        self.future_tolerance = float(gp("future_tolerance_s").value)
        if self.odom_timeout <= 0.0 or self.future_tolerance < 0.0:
            raise ValueError("invalid odom/future timeout")
        self.odom_gate = StampGate(
            self.odom_timeout, self.future_tolerance)
        self.range_gates = {
            side: StampGate(self.sensor_timeout, self.future_tolerance)
            for side in ("left", "right")
        }
        self.target_gate = StampGate(1.0, self.future_tolerance)

        self.robot_pose = None
        self.last_odom_time = 0.0
        self.active_target = None
        self.robot_state = "IDLE"
        self.ultrasonic_phase_ready = False
        self.published = False
        self.last_range_time = {"left": 0.0, "right": 0.0}
        self.threshold_m = float(gp("threshold_m").value)
        self.lateral_pair_timeout = float(
            gp("lateral_pair_timeout_s").value)
        self.lateral_sign = float(gp("lateral_sign").value)
        lateral_median_n = int(gp("lateral_median_n").value)
        if self.threshold_m <= 0.0:
            raise ValueError("threshold_m must be positive")
        if self.lateral_pair_timeout <= 0.0:
            raise ValueError("lateral_pair_timeout_s must be positive")
        if self.lateral_sign not in (-1.0, 1.0):
            raise ValueError("lateral_sign must be +1.0 or -1.0")
        if lateral_median_n <= 0:
            raise ValueError("lateral_median_n must be positive")
        self.lateral_hist = {
            side: deque(maxlen=lateral_median_n)
            for side in ("left", "right")
        }
        self.lateral_valid_prev = None
        self.stale_reported = set()
        self.yaw_reject_reported = False
        self.target_missing_reported = False
        self.fault_sent = False
        self.detector = AxleSequenceDetector(
            target_index=self.target_axle,
            expected_spacing_m=self.expected_axle_spacing,
            spacing_tolerance_m=self.axle_spacing_tolerance,
            direction=scan_direction(self.role),
            expected_first_position_m=-self.expected_axle_spacing / 2.0,
            position_tolerance_m=self.axle_position_tolerance,
            threshold_m=self.threshold_m,
            exit_hysteresis_m=float(gp("exit_hysteresis_m").value),
            window_size=int(gp("window_size").value),
            pair_timeout_s=float(gp("pair_timeout_s").value),
        )

        self.create_subscription(
            Odometry, f"/{self.role}/odom", self.odom_cb,
            SENSOR_LATEST_QOS)
        self.create_subscription(
            PoseStamped, f"/{self.role}/active_target_pose",
            self.active_target_cb, 10)
        self.create_subscription(
            String, f"/{self.role}/robot_state", self.state_cb,
            STATE_LATEST_QOS)
        self.create_subscription(
            Bool, f"/{self.role}/wheel_scan_reset", self.scan_reset_cb, 10)
        self.create_subscription(
            Bool, f"/{self.role}/ultrasonic_ready",
            self.ultrasonic_ready_cb, STATE_LATEST_QOS)
        self.create_subscription(
            String, "/parking/vehicle_spec", self.vehicle_spec_cb, 10)
        for side in ("left", "right"):
            self.create_subscription(
                Range,
                f"/{self.role}/ultrasonic_{side}",
                lambda msg, s=side: self.range_cb(s, msg),
                SENSOR_LATEST_QOS,
            )

        self.pub_detected = self.create_publisher(
            Bool, f"/{self.role}/wheel_detected", 10)
        self.pub_center_s = self.create_publisher(
            Float64, f"/{self.role}/wheel_center_s", 10)
        self.pub_axle_count = self.create_publisher(
            Int32, f"/{self.role}/axle_count", 10)
        # Retained for monitoring/backward compatibility. It is the world-x
        # coordinate of the centerline target, not the control coordinate.
        self.pub_center_x = self.create_publisher(
            Float64, f"/{self.role}/wheel_center_x", 10)
        self.pub_fault = self.create_publisher(
            String, f"/{self.role}/motion_fault", 10)
        self.pub_lateral = self.create_publisher(
            Float64, f"/{self.role}/wheel_lateral_offset", 10)
        self.pub_lateral_valid = self.create_publisher(
            Bool, f"/{self.role}/wheel_lateral_valid", 10)
        self.create_timer(0.25, self.check_sensor_freshness)

        self.get_logger().info(
            f"[{self.role}] vehicle-frame ultrasonic edge mode "
            f"(target_axle={self.target_axle}, "
            f"spacing={self.expected_axle_spacing:.3f}m, "
            f"position_tol={self.axle_position_tolerance:.3f}m, "
            f"left_offset={self.sensor_to_gripper_x['left']:+.3f}m, "
            f"right_offset={self.sensor_to_gripper_x['right']:+.3f}m)")

    @staticmethod
    def pose_from_stamped(msg):
        p = msg.pose.position
        q = msg.pose.orientation
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if norm < 1e-6:
            return None
        yaw = math.atan2(
            2.0 * (q.w*q.z + q.x*q.y),
            1.0 - 2.0 * (q.y*q.y + q.z*q.z))
        result = (float(p.x), float(p.y), float(yaw))
        return result if all(math.isfinite(value) for value in result) else None

    def odom_cb(self, msg):
        accepted, reason = self.odom_gate.accept(
            stamp_to_ns(msg.header.stamp),
            self.get_clock().now().nanoseconds)
        if not accepted:
            self.get_logger().warn(
                f"[{self.role}] ultrasonic odom rejected: {reason}",
                throttle_duration_sec=2.0)
            return
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w*q.z + q.x*q.y),
            1.0 - 2.0 * (q.y*q.y + q.z*q.z))
        result = (float(p.x), float(p.y), float(yaw))
        if all(math.isfinite(value) for value in result):
            self.robot_pose = result
            self.last_odom_time = time.monotonic()

    def active_target_cb(self, msg):
        if msg.header.frame_id not in ("", "map"):
            return
        accepted, reason = self.target_gate.accept(
            stamp_to_ns(msg.header.stamp),
            self.get_clock().now().nanoseconds)
        if not accepted:
            self.get_logger().warn(
                f"[{self.role}] active target rejected: {reason}",
                throttle_duration_sec=2.0)
            return
        target = self.pose_from_stamped(msg)
        if target is not None:
            self.active_target = target
            self.target_missing_reported = False

    def vehicle_spec_cb(self, msg):
        if not self.use_vehicle_spec_wheelbase:
            return
        try:
            candidate = float(json.loads(msg.data)["wheelbase"])
            if not math.isfinite(candidate) or candidate <= 0.0:
                raise ValueError("wheelbase must be finite and positive")
            if self.robot_state == "ALIGN" and self.detector.centers:
                self.get_logger().warn(
                    f"[{self.role}] wheelbase update ignored during axle scan",
                    throttle_duration_sec=2.0)
                return
            self.expected_axle_spacing = candidate
            self.detector.set_expected_geometry(candidate, -candidate / 2.0)
        except (KeyError, TypeError, ValueError, RuntimeError,
                json.JSONDecodeError) as exc:
            self.get_logger().warn(
                f"[{self.role}] invalid ultrasonic wheelbase ignored: {exc}",
                throttle_duration_sec=2.0)

    def state_cb(self, msg):
        previous = self.robot_state
        self.robot_state = msg.data
        if msg.data == "APPROACH" and previous != "APPROACH":
            self.active_target = None
            self.reset_cycle()
        elif msg.data == "ALIGN" and previous != "ALIGN":
            # The individual motion node republishes one immutable active frame.
            # Do not sample the live CCTV target again at ALIGN entry.
            self.reset_cycle()

    def scan_reset_cb(self, msg):
        if msg.data:
            self.get_logger().info(
                f"[{self.role}] wheel scan reset requested")
            self.reset_cycle()

    def ultrasonic_ready_cb(self, msg):
        ready = bool(msg.data)
        previous = self.ultrasonic_phase_ready
        self.ultrasonic_phase_ready = ready
        if ready and not previous:
            # Start the detector with only samples from this acknowledged,
            # validated activation generation.
            self.reset_cycle()
        elif not ready and previous:
            self.publish_lateral(time.monotonic())

    def median_distance(self, side):
        history = self.lateral_hist[side]
        if not history:
            return None
        ordered = sorted(history)
        return ordered[len(ordered) // 2]

    def publish_lateral(self, now):
        """Publish lateral offset only for a fresh simultaneous echo pair."""
        offset = None
        fresh = all(
            now - self.last_range_time[side] <= self.lateral_pair_timeout
            for side in ("left", "right"))
        if self.robot_state == "ALIGN" and fresh:
            left = self.median_distance("left")
            right = self.median_distance("right")
            if left is not None and right is not None:
                offset = paired_lateral_offset(
                    left, right, self.threshold_m, self.lateral_sign)
        valid = offset is not None
        if valid != self.lateral_valid_prev:
            self.lateral_valid_prev = valid
            self.get_logger().info(
                f"[{self.role}] lateral offset "
                f"{'available' if valid else 'unavailable'}")
        self.pub_lateral.publish(Float64(data=offset or 0.0))
        self.pub_lateral_valid.publish(Bool(data=valid))

    def reset_cycle(self):
        self.detector.reset()
        self.published = False
        now = time.monotonic()
        self.last_range_time = {"left": now, "right": now}
        for history in self.lateral_hist.values():
            history.clear()
        self.lateral_valid_prev = False
        self.pub_lateral.publish(Float64(data=0.0))
        self.pub_lateral_valid.publish(Bool(data=False))
        self.stale_reported.clear()
        self.yaw_reject_reported = False
        self.target_missing_reported = False
        self.fault_sent = False
        self.cycle_start_time = now
        self.get_logger().info(f"[{self.role}] ultrasonic edge reset")

    def report_fault(self, reason):
        if self.fault_sent:
            return
        self.fault_sent = True
        self.pub_fault.publish(String(data=reason))
        self.get_logger().error(f"[{self.role}] ultrasonic fault: {reason}")

    def range_cb(self, side, msg):
        if not self.ultrasonic_phase_ready:
            return
        accepted, reason = self.range_gates[side].accept(
            stamp_to_ns(msg.header.stamp),
            self.get_clock().now().nanoseconds)
        if not accepted:
            self.get_logger().warn(
                f"[{self.role}] ultrasonic {side} rejected: {reason}",
                throttle_duration_sec=2.0)
            return
        now = time.monotonic()
        self.last_range_time[side] = now
        if side in self.stale_reported:
            self.stale_reported.remove(side)
            self.get_logger().info(
                f"[{self.role}] ultrasonic {side} stream recovered")
        distance = float(msg.range)
        self.lateral_hist[side].append(distance)
        self.process_distance(side, distance, now)
        self.publish_lateral(now)

    def check_sensor_freshness(self):
        if (self.robot_state != "ALIGN" or
                not self.ultrasonic_phase_ready):
            return
        now = time.monotonic()
        if (self.active_target is None and
                now - self.cycle_start_time > self.sensor_timeout):
            self.report_fault("ACTIVE_TARGET_FRAME_MISSING")
            return
        if now - self.last_odom_time > self.odom_timeout:
            self.report_fault("ODOM_TIMEOUT")
            return
        for side in ("left", "right"):
            if now - self.last_range_time[side] > self.sensor_timeout:
                if side not in self.stale_reported:
                    self.stale_reported.add(side)
                    self.get_logger().error(
                        f"[{self.role}] ultrasonic {side} Range timeout "
                        f"(>{self.sensor_timeout:.2f}s)")
                    self.report_fault(
                        f"ULTRASONIC_{side.upper()}_STREAM_TIMEOUT")

    def process_distance(self, side, distance, timestamp):
        if (self.robot_state != "ALIGN" or
                not self.ultrasonic_phase_ready or self.published):
            return
        if self.robot_pose is None:
            return
        if self.active_target is None:
            if not self.target_missing_reported:
                self.target_missing_reported = True
                self.get_logger().error(
                    f"[{self.role}] ALIGN has no latched target frame")
            return

        x, y, robot_yaw = self.robot_pose
        tx, ty, vehicle_yaw = self.active_target
        yaw_error = angle_norm(robot_yaw - vehicle_yaw)
        if abs(yaw_error) > self.max_sensor_yaw_error:
            if not self.yaw_reject_reported:
                self.yaw_reject_reported = True
                self.get_logger().error(
                    f"[{self.role}] ultrasonic sample rejected: "
                    f"yaw error={math.degrees(yaw_error):.1f}deg")
            return
        self.yaw_reject_reported = False

        position_s, _ = world_to_vehicle(
            x, y, tx, ty, vehicle_yaw)
        offset_s = projected_robot_x_offset(
            self.sensor_to_gripper_x[side], robot_yaw, vehicle_yaw)
        corrected_s = position_s - offset_s
        event = self.detector.update(
            side, distance, corrected_s, timestamp)
        if event is None:
            return

        self.pub_axle_count.publish(Int32(data=event.index))
        if not event.final:
            self.get_logger().info(
                f"[{self.role}] axle {event.index}/{self.target_axle} "
                f"passed at s={event.center_x:.3f}m")
            return

        self.published = True
        center_s = event.center_x
        center_x, _ = vehicle_to_world(
            center_s, 0.0, tx, ty, vehicle_yaw)
        # Publish the target before detected; the consumer still tolerates
        # cross-topic reordering by stopping until both values are present.
        self.pub_center_s.publish(Float64(data=center_s))
        self.pub_center_x.publish(Float64(data=center_x))
        self.pub_detected.publish(Bool(data=True))
        self.get_logger().info(
            f"[{self.role}] target axle {event.index} paired wheel edge: "
            f"gripper_target_s={center_s:.3f}m")


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicEdgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
