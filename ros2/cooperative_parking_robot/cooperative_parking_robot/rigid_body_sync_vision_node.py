#!/usr/bin/env python3
"""Vision-source-aware final production wrapper for rigid-body sync."""

from __future__ import annotations

import rclpy
from std_msgs.msg import String

from cooperative_parking_robot.cctv_observation import CctvObservation
from cooperative_parking_robot.rigid_body_sync_production_node import (
    RigidBodySyncNode as P0RigidBodySyncNode,
)


class RigidBodySyncNode(P0RigidBodySyncNode):
    """Prevent mixed-camera overhead fallback from entering relative control."""

    def __init__(self):
        super().__init__()
        self._cctv_observation = {'front': None, 'rear': None}
        self._cctv_observation_receipt = {'front': 0.0, 'rear': 0.0}
        self._cctv_observation_stamp_ns = {'front': 0, 'rear': 0}
        self.create_subscription(
            String, '/front/cctv_observation',
            lambda msg: self._cctv_observation_cb('front', msg), 10)
        self.create_subscription(
            String, '/rear/cctv_observation',
            lambda msg: self._cctv_observation_cb('rear', msg), 10)
        self.get_logger().info(
            'source-aware rigid fallback active | same-camera overhead pairs only')

    def _cctv_observation_cb(self, role, msg):
        import time
        try:
            observation = CctvObservation.from_json(msg.data)
        except ValueError:
            return
        if observation.role != role:
            return
        if observation.stamp_ns <= self._cctv_observation_stamp_ns[role]:
            return
        self._cctv_observation[role] = observation
        self._cctv_observation_stamp_ns[role] = observation.stamp_ns
        self._cctv_observation_receipt[role] = time.monotonic()

    def _new_cctv_pair(self, now):
        observations = self._cctv_observation
        if any(observations[role] is None for role in ('front', 'rear')):
            return None
        if any(
                now - self._cctv_observation_receipt[role] >
                self.cctv_pair_timeout
                for role in ('front', 'rear')):
            return None
        front = observations['front']
        rear = observations['rear']
        # Relative correction must not subtract two independently biased camera
        # frames. The PoseFusion path may use them individually; this fallback
        # intentionally requires one common camera source.
        if front.camera_id != rear.camera_id:
            self._last_visual_reason = (
                f'mixed_camera_pair:{front.camera_id}/{rear.camera_id}')
            return None
        if abs(front.stamp_ns - rear.stamp_ns) * 1.0e-9 > (
                self.cctv_pair_sync_slop):
            self._last_visual_reason = 'source_pair_stamp_skew'
            return None
        if not self._cctv_pair_stamp_gate.accept(
                front.stamp_ns, rear.stamp_ns):
            return None
        self._last_cctv_pair_used = {
            'front': front.stamp_ns, 'rear': rear.stamp_ns}
        front_pose = dict(
            x=front.pose[0], y=front.pose[1], theta=front.pose[2])
        rear_pose = dict(
            x=rear.pose[0], y=rear.pose[1], theta=rear.pose[2])
        longitudinal, lateral, yaw = (
            self.kinematics.relative_pose_in_rear_frame(
                front_pose, rear_pose))
        self._last_visual_seen_time = now
        self._last_visual_reason = f'same_camera:{front.camera_id}'
        return longitudinal, lateral, yaw


def main(args=None):
    rclpy.init(args=args)
    node = RigidBodySyncNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
