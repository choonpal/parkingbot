#!/usr/bin/env python3
"""Source-aware production wrapper for ceiling-marker localization."""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, String

from cooperative_parking_robot.cctv_observation import (
    CctvObservation,
    normalize_angle,
)
from cooperative_parking_robot.cctv_robot_marker_node import (
    CctvRobotMarkerNode as BaselineCctvRobotMarkerNode,
)
from cooperative_parking_robot.freshness import stamp_to_ns
from cooperative_parking_robot.site_geometry import (
    CAMERA_GEOMETRY,
    FRONT_MARKER_OFFSET_X_M,
    REAR_MARKER_OFFSET_X_M,
    ROBOT_MARKER_HEIGHT_M,
)


class CctvRobotMarkerNode(BaselineCctvRobotMarkerNode):
    """Publish pose and source atomically while smoothing camera handover."""

    def __init__(self):
        super().__init__()

        self.declare_parameter('source_switch_min_improvement_m', 0.05)
        self.declare_parameter('source_switch_confirm_frames', 3)
        self.declare_parameter('source_alignment_alpha', 0.25)
        self.declare_parameter('source_alignment_sync_slop_s', 0.05)
        self.declare_parameter('source_alignment_max_position_m', 0.15)
        self.declare_parameter('source_alignment_max_yaw_deg', 10.0)
        self.declare_parameter('source_handover_valid_s', 0.50)
        self.declare_parameter('source_handover_max_position_m', 0.03)
        self.declare_parameter('source_handover_max_yaw_deg', 3.0)

        gp = self.get_parameter
        self.source_switch_margin = float(
            gp('source_switch_min_improvement_m').value)
        self.source_switch_confirm_frames = int(
            gp('source_switch_confirm_frames').value)
        self.source_alignment_alpha = float(
            gp('source_alignment_alpha').value)
        self.source_alignment_sync_slop = float(
            gp('source_alignment_sync_slop_s').value)
        self.source_alignment_max_position = float(
            gp('source_alignment_max_position_m').value)
        self.source_alignment_max_yaw = math.radians(float(
            gp('source_alignment_max_yaw_deg').value))
        self.source_handover_valid_s = float(
            gp('source_handover_valid_s').value)
        self.source_handover_max_position = float(
            gp('source_handover_max_position_m').value)
        self.source_handover_max_yaw = math.radians(float(
            gp('source_handover_max_yaw_deg').value))
        if (self.source_switch_margin < 0.0 or
                self.source_switch_confirm_frames < 1 or
                not 0.0 < self.source_alignment_alpha <= 1.0 or
                self.source_alignment_sync_slop <= 0.0 or
                self.source_alignment_max_position <= 0.0 or
                self.source_alignment_max_yaw <= 0.0 or
                self.source_handover_valid_s <= 0.0 or
                self.source_handover_max_position <= 0.0 or
                self.source_handover_max_yaw <= 0.0):
            raise ValueError('invalid CCTV source handover parameters')

        self._apply_measured_site_geometry()
        self.pub_observation = {
            role: self.create_publisher(
                String, f'/{role}/cctv_observation', 10)
            for role in self.marker_ids
        }
        self._source_sequence = {role: 0 for role in self.marker_ids}
        self._switch_sequence = {role: 0 for role in self.marker_ids}
        self._switch_candidate = {role: (None, 0) for role in self.marker_ids}
        self._source_bias = {
            role: {
                camera['camera_id']: (0.0, 0.0, 0.0)
                for camera in self.cameras}
            for role in self.marker_ids
        }
        self._alignment_wall = {
            role: {camera['camera_id']: 0.0 for camera in self.cameras}
            for role in self.marker_ids
        }
        canonical = self.cameras[0]['camera_id'] if self.cameras else None
        self._canonical_camera = canonical
        self.get_logger().info(
            'source-aware CCTV marker handover active | '
            f'canonical={canonical} | switch_confirm='
            f'{self.source_switch_confirm_frames}')

    def _set_parameter(self, name, value):
        result = self.set_parameters([Parameter(name, value=value)])[0]
        if not result.successful:
            raise RuntimeError(
                f'failed to apply measured parameter {name}: {result.reason}')

    def _apply_measured_site_geometry(self):
        """Prefer launch-provided site geometry and fill only missing values."""
        if len(self.cameras) == 2:
            ids = [camera['camera_id'] for camera in self.cameras]
            configured = all(
                camera['height_m'] > 0.0 and
                any(abs(value) > 1.0e-9 for value in camera['axis_ground'])
                for camera in self.cameras)
            if not configured and set(ids) == set(CAMERA_GEOMETRY):
                ground = []
                heights = []
                for camera_id in ids:
                    geometry = CAMERA_GEOMETRY[camera_id]
                    ground.extend(geometry.optical_axis_ground_m)
                    heights.append(geometry.optical_center_height_m)
                self._set_parameter('camera_ground_points', ground)
                self._set_parameter('camera_heights_m', heights)
                for camera, camera_id in zip(self.cameras, ids):
                    geometry = CAMERA_GEOMETRY[camera_id]
                    camera['axis_ground'] = geometry.optical_axis_ground_m
                    camera['height_m'] = geometry.optical_center_height_m
                self.get_logger().warn(
                    'site camera geometry was not configured; using '
                    'repository fallback')
        for role, fallback_height, fallback_offset in (
                ('front', ROBOT_MARKER_HEIGHT_M, FRONT_MARKER_OFFSET_X_M),
                ('rear', ROBOT_MARKER_HEIGHT_M, REAR_MARKER_OFFSET_X_M)):
            if self.marker_height[role] <= 0.0:
                self._set_parameter(
                    f'{role}_marker_height_m', fallback_height)
                self.marker_height[role] = fallback_height
            # Zero is a valid measured centre offset. Preserve every explicit
            # launch value instead of silently replacing it after a camera or
            # marker remount.
            if not math.isfinite(self.marker_offset_x[role]):
                self._set_parameter(
                    f'{role}_marker_offset_x_m', fallback_offset)
                self.marker_offset_x[role] = fallback_offset

    @staticmethod
    def _stamp_s(stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9

    @staticmethod
    def _pose_delta(lhs, rhs):
        return (
            float(lhs[0]) - float(rhs[0]),
            float(lhs[1]) - float(rhs[1]),
            normalize_angle(float(lhs[2]) - float(rhs[2])),
        )

    @staticmethod
    def _apply_bias(pose, bias):
        return (
            float(pose[0]) + float(bias[0]),
            float(pose[1]) + float(bias[1]),
            normalize_angle(float(pose[2]) + float(bias[2])),
        )

    def _update_source_alignment(self, role, fresh, now):
        canonical_id = self._canonical_camera
        if canonical_id not in fresh:
            return
        canonical = fresh[canonical_id]
        canonical_stamp_s = self._stamp_s(canonical['stamp'])
        for camera_id, observation in fresh.items():
            if camera_id == canonical_id:
                continue
            if abs(self._stamp_s(observation['stamp']) - canonical_stamp_s) > (
                    self.source_alignment_sync_slop):
                continue
            delta = self._pose_delta(canonical['pose'], observation['pose'])
            if (math.hypot(delta[0], delta[1]) >
                    self.source_alignment_max_position or
                    abs(delta[2]) > self.source_alignment_max_yaw):
                continue
            previous = self._source_bias[role][camera_id]
            alpha = self.source_alignment_alpha
            bias = (
                (1.0 - alpha) * previous[0] + alpha * delta[0],
                (1.0 - alpha) * previous[1] + alpha * delta[1],
                normalize_angle(previous[2] + alpha * normalize_angle(
                    delta[2] - previous[2])),
            )
            self._source_bias[role][camera_id] = bias
            self._alignment_wall[role][camera_id] = now

    def _handover_validated(self, role, current_id, candidate_id,
                            fresh, now):
        if current_id not in fresh or candidate_id not in fresh:
            return False
        current = self._apply_bias(
            fresh[current_id]['pose'], self._source_bias[role][current_id])
        candidate = self._apply_bias(
            fresh[candidate_id]['pose'], self._source_bias[role][candidate_id])
        delta = self._pose_delta(candidate, current)
        candidate_alignment = self._alignment_wall[role][candidate_id]
        return (
            math.hypot(delta[0], delta[1]) <=
            self.source_handover_max_position and
            abs(delta[2]) <= self.source_handover_max_yaw and
            (candidate_id == self._canonical_camera or
             now - candidate_alignment <= self.source_handover_valid_s)
        )

    def _choose_source(self, role, fresh, now):
        current_id, selected_at = self._selected[role]
        if not fresh:
            self._selected[role] = (None, now)
            self._switch_candidate[role] = (None, 0)
            return None, False
        best_id = min(fresh, key=lambda camera_id: fresh[camera_id]['cost'])
        if current_id not in fresh:
            self._selected[role] = (best_id, now)
            self._switch_candidate[role] = (None, 0)
            return best_id, False
        if best_id == current_id:
            self._switch_candidate[role] = (None, 0)
            return current_id, False
        if now - selected_at < self.selection_hold_s:
            return current_id, False
        current_cost = float(fresh[current_id]['cost'])
        best_cost = float(fresh[best_id]['cost'])
        if current_cost - best_cost < self.source_switch_margin:
            self._switch_candidate[role] = (None, 0)
            return current_id, False
        candidate_id, count = self._switch_candidate[role]
        count = count + 1 if candidate_id == best_id else 1
        self._switch_candidate[role] = (best_id, count)
        if count < self.source_switch_confirm_frames:
            return current_id, False
        validated = self._handover_validated(
            role, current_id, best_id, fresh, now)
        self._selected[role] = (best_id, now)
        self._switch_candidate[role] = (None, 0)
        self._switch_sequence[role] += 1
        self.get_logger().info(
            f'[{role}] CCTV source {current_id} -> {best_id} | '
            f'validated={validated}')
        return best_id, validated

    def _publish_selected(self, now):
        for role in self.marker_ids:
            fresh = {
                camera_id: observation
                for camera_id, observation in self._observations[role].items()
                if now - observation['wall'] <= self.observation_timeout_s
            }
            self._observations[role] = fresh
            self._update_source_alignment(role, fresh, now)
            previous_id = self._selected[role][0]
            chosen_id, handover_validated = self._choose_source(
                role, fresh, now)
            source_changed = (
                previous_id is not None and chosen_id is not None and
                previous_id != chosen_id)
            visible = chosen_id is not None
            self.pub_visible[role].publish(Bool(data=visible))
            if visible:
                observation = fresh[chosen_id]
                raw_pose = observation['pose']
                bias = self._source_bias[role][chosen_id]
                corrected = self._apply_bias(raw_pose, bias)
                stamp_ns = stamp_to_ns(observation['stamp'])
                if stamp_ns <= 0:
                    stamp_ns = self.get_clock().now().nanoseconds
                self._source_sequence[role] += 1
                envelope = CctvObservation(
                    role=role,
                    camera_id=chosen_id,
                    stamp_ns=stamp_ns,
                    sequence=self._source_sequence[role],
                    switch_sequence=self._switch_sequence[role],
                    source_changed=source_changed,
                    handover_validated=(
                        handover_validated or not source_changed),
                    pose=corrected,
                    raw_pose=raw_pose,
                    source_bias=bias,
                    selection_cost=float(observation['cost']),
                )
                out = PoseStamped()
                out.header.stamp = observation['stamp']
                out.header.frame_id = 'map'
                out.pose.position.x = corrected[0]
                out.pose.position.y = corrected[1]
                out.pose.orientation.z = math.sin(corrected[2] / 2.0)
                out.pose.orientation.w = math.cos(corrected[2] / 2.0)
                self.pub_pose[role].publish(out)
                self.pub_observation[role].publish(
                    String(data=envelope.to_json()))
            if visible != self._last_visible[role]:
                suffix = f' ({chosen_id})' if visible else ''
                self.get_logger().info(
                    f"[{role}] CCTV 상판 마커 "
                    f"{'인식' if visible else '놓침'}{suffix}")
            self._last_visible[role] = visible


def main(args=None):
    rclpy.init(args=args)
    node = CctvRobotMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
