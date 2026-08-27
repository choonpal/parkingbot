#!/usr/bin/env python3
"""Production pose fusion with source-aware CCTV and delayed replay."""

from __future__ import annotations

import json
import math

import rclpy
from std_msgs.msg import String

from cooperative_parking_robot.cctv_observation import CctvObservation
from cooperative_parking_robot.cctv_source_policy import SourceSwitchGuard
from cooperative_parking_robot.freshness import stamp_to_ns
from cooperative_parking_robot.pose_fusion_node import (
    PoseFusionNode as BaselinePoseFusionNode,
)
from cooperative_parking_robot.pose_replay import EkfReplayBuffer


class PoseFusionNode(BaselinePoseFusionNode):
    """Apply a camera frame at capture time, then replay later wheel deltas."""

    def __init__(self):
        super().__init__()
        self.declare_parameter('cctv_replay_history_s', 1.0)
        self.declare_parameter('cctv_source_confirm_frames', 3)
        self.declare_parameter('cctv_source_consistency_position_m', 0.03)
        self.declare_parameter('cctv_source_consistency_yaw_deg', 3.0)
        self.declare_parameter('cctv_source_max_jump_m', 0.12)
        self.declare_parameter('cctv_source_max_jump_yaw_deg', 12.0)
        gp = self.get_parameter
        self.replay = EkfReplayBuffer(
            self.ekf, history_s=float(gp('cctv_replay_history_s').value))
        self.source_guard = SourceSwitchGuard(
            confirmations=int(gp('cctv_source_confirm_frames').value),
            consistency_position_m=float(
                gp('cctv_source_consistency_position_m').value),
            consistency_yaw_rad=math.radians(float(
                gp('cctv_source_consistency_yaw_deg').value)),
            max_position_jump_m=float(gp('cctv_source_max_jump_m').value),
            max_yaw_jump_rad=math.radians(float(
                gp('cctv_source_max_jump_yaw_deg').value)))
        self.pub_fusion_status = self.create_publisher(
            String, f'/{self.role}/cctv_fusion_status', 10)
        self.create_subscription(
            String, f'/{self.role}/cctv_observation',
            self.cctv_observation_cb, 10)
        self._last_cctv_sequence = 0
        self._last_replay = None
        self.get_logger().info(
            f'[{self.role}] source-aware delayed CCTV fusion active')

    def wheel_odom_cb(self, msg):
        """Mirror baseline wheel prediction while retaining replay history."""
        now = self.get_clock().now()
        stamp_ns = stamp_to_ns(msg.header.stamp)
        if stamp_ns <= 0:
            stamp_ns = now.nanoseconds
        if self._last_wheel_stamp is None:
            dt = self.default_dt
        else:
            previous_ns = int(self._last_wheel_stamp.nanoseconds)
            raw_dt = (stamp_ns - previous_ns) * 1.0e-9
            if raw_dt <= 0.0:
                self.get_logger().warn(
                    '역행/중복 wheel_odom timestamp — 측정 폐기')
                return
            dt = min(raw_dt, self.max_dt)
        dx_body = float(msg.twist.twist.linear.x)
        dy_body = float(msg.twist.twist.linear.y)
        dtheta = float(msg.twist.twist.angular.z)
        if not all(math.isfinite(value) for value in (
                dx_body, dy_body, dtheta, dt)):
            self.get_logger().warn('비정상 wheel_odom delta 폐기')
            return
        self.replay.record_predict(
            stamp_ns, dx_body, dy_body, dtheta, dt)
        from rclpy.time import Time
        self._last_wheel_stamp = Time(nanoseconds=stamp_ns)
        self.publish_odom(msg.header.stamp)

    def cctv_pose_cb(self, msg):
        """Legacy PoseStamped remains diagnostic only; envelope owns correction."""
        self._last_cctv_age = (
            self.get_clock().now().nanoseconds - stamp_to_ns(msg.header.stamp)
        ) * 1.0e-9

    def cctv_observation_cb(self, msg):
        try:
            observation = CctvObservation.from_json(msg.data)
        except ValueError as exc:
            self._last_source = 'ENCODER_BAD_CCTV_ENVELOPE'
            self.get_logger().warn(
                f'CCTV observation rejected: {exc}',
                throttle_duration_sec=2.0)
            return
        if observation.role != self.role:
            return
        if observation.sequence <= self._last_cctv_sequence:
            return
        now_ns = self.get_clock().now().nanoseconds
        age = (now_ns - observation.stamp_ns) * 1.0e-9
        self._last_cctv_age = age
        if age < -self.max_future_skew or age > self.cctv_timeout:
            self._last_source = (
                'ENCODER_FUTURE_CCTV' if age < 0.0 else
                'ENCODER_STALE_CCTV')
            return
        predicted = self.ekf.pose()
        decision = self.source_guard.evaluate(
            observation.camera_id, observation.pose, predicted,
            handover_validated=observation.handover_validated)
        if not decision.accepted:
            self._last_source = f'CCTV_SOURCE_REJECT:{decision.reason}'
            return

        def apply_correction():
            was_initialized = self.ekf.initialized
            accepted = self.ekf.correct(
                observation.pose[0], observation.pose[1],
                observation.pose[2], R=self.R)
            if accepted and not was_initialized:
                self.get_logger().info(
                    f'[{self.role}] EKF 절대 초기화: '
                    f'{observation.pose[0]:.3f},'
                    f'{observation.pose[1]:.3f},'
                    f'{math.degrees(observation.pose[2]):.1f}deg')
            return accepted

        result = self.replay.correct_at(
            observation.stamp_ns, apply_correction)
        self._last_replay = result
        self._last_cctv_sequence = observation.sequence
        self._last_cctv_stamp = None
        self._cctv_visible = True
        self._last_source = (
            f'CCTV_{observation.camera_id}_{result.status}'
            if result.accepted else 'CCTV_REJECTED_GATE')
        self._publish_fusion_status(observation, decision.reason, result)

    def _publish_fusion_status(self, observation, source_reason, replay):
        payload = {
            'role': self.role,
            'camera_id': observation.camera_id,
            'sequence': observation.sequence,
            'switch_sequence': observation.switch_sequence,
            'source_changed': observation.source_changed,
            'handover_validated': observation.handover_validated,
            'source_reason': source_reason,
            'accepted': replay.accepted,
            'replay_status': replay.status,
            'rewind_s': replay.rewind_s,
            'replayed_steps': replay.replayed_steps,
            'quantization_s': replay.quantization_s,
            'history_steps': len(self.replay.steps),
        }
        self.pub_fusion_status.publish(String(
            data=json.dumps(payload, ensure_ascii=False)))


def main(args=None):
    rclpy.init(args=args)
    node = PoseFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
