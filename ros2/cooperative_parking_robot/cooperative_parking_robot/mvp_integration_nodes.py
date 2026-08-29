#!/usr/bin/env python3
"""Selective MVP integration on top of the hardened main-branch nodes.

This module keeps the latest fail-closed camera/coverage behavior from ``main``
while adding the negative-origin map geometry and HOME-pose behavior developed
on ``feature/retrieval-integration-mvp``. Console entry points are routed here
so future fixes in the original nodes remain inherited instead of being
replaced by an older full-file copy.
"""

from __future__ import annotations

import json
import math
from typing import Type

import rclpy
from std_msgs.msg import Bool

from cooperative_parking_robot import bev_layout_calibrator_node as calibrator_module
from cooperative_parking_robot import cctv_merge_node as cctv_module
from cooperative_parking_robot import individual_move_node as move_module
from cooperative_parking_robot import yolo_bev_map_node as yolo_module
from cooperative_parking_robot.bev_fusion_core import (
    CameraDetection, MergedDetection,
)
from cooperative_parking_robot.parking_geometry import grid_cell_count
from cooperative_parking_robot.vehicle_entry import angle_norm


def _finite(name: str, value: object) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


class _OriginPublisher:
    """Set OccupancyGrid metadata while delegating to the ROS publisher."""

    def __init__(self, delegate, owner):
        self._delegate = delegate
        self._owner = owner

    def publish(self, message):
        message.info.origin.position.x = self._owner.map_origin_x_m
        message.info.origin.position.y = self._owner.map_origin_y_m
        self._delegate.publish(message)

    def __getattr__(self, name):
        return getattr(self._delegate, name)


def _shift_point(point, origin_x, origin_y):
    return float(point[0]) - origin_x, float(point[1]) - origin_y


def _shift_polygon(polygon, origin_x, origin_y):
    if polygon is None:
        return None
    return [_shift_point(point, origin_x, origin_y) for point in polygon]


def _shift_pose(pose, origin_x, origin_y):
    if pose is None:
        return None
    return (_shift_point(pose, origin_x, origin_y) + tuple(pose[2:]))


def _shift_detection(detection, origin_x, origin_y):
    if isinstance(detection, MergedDetection):
        shifted = MergedDetection(
            _shift_detection(detection.primary, origin_x, origin_y))
        shifted.center = _shift_point(
            detection.center, origin_x, origin_y)
        shifted.polygon = _shift_polygon(
            detection.polygon, origin_x, origin_y)
        shifted.yaw = detection.yaw
        shifted.length_m = detection.length_m
        shifted.width_m = detection.width_m
        shifted.in_waiting = detection.in_waiting
        shifted.sources = list(detection.sources)
        return shifted
    return CameraDetection(
        camera_id=detection.camera_id,
        center=_shift_point(detection.center, origin_x, origin_y),
        polygon=_shift_polygon(detection.polygon, origin_x, origin_y),
        yaw=detection.yaw,
        length_m=detection.length_m,
        width_m=detection.width_m,
        in_waiting=detection.in_waiting,
        confidence=detection.confidence,
        axis_dist_m=detection.axis_dist_m,
        vehicle_class=detection.vehicle_class,
        classified_wheelbase_m=detection.classified_wheelbase_m,
    )


def _origin_aware_calibrator_html():
    """Extend the latest main UI without replacing its full embedded page."""
    html = calibrator_module._HTML
    html = html.replace(
        '      <label>맵 폭(m) <input id="mapW" type="number" '
        'value="4.40" step="0.01"></label>\n'
        '      <label>맵 높이(m) <input id="mapH" type="number" '
        'value="3.83" step="0.01"></label>',
        '      <label>맵 원점 X(m) <input id="mapOriginX" type="number" '
        'value="-0.40" step="0.01"></label>\n'
        '      <label>맵 원점 Y(m) <input id="mapOriginY" type="number" '
        'value="-0.80" step="0.01"></label>\n'
        '      <label>맵 폭(m) <input id="mapW" type="number" '
        'value="4.80" step="0.01"></label>\n'
        '      <label>맵 높이(m) <input id="mapH" type="number" '
        'value="4.63" step="0.01"></label>')
    html = html.replace(
        'let references = [], slotClicks = [], waitingClicks = [];\n',
        'let references = [], slotClicks = [], waitingClicks = [];\n'
        'let mapFieldsInitialized = false;\n')
    html = html.replace(
        "      '등록 슬롯 없음';\n  } catch (error)",
        "      '등록 슬롯 없음';\n"
        "    if (!mapFieldsInitialized) {\n"
        "      document.getElementById('mapOriginX').value = "
        "state.map_origin_x_m;\n"
        "      document.getElementById('mapOriginY').value = "
        "state.map_origin_y_m;\n"
        "      document.getElementById('mapW').value = state.map_width_m;\n"
        "      document.getElementById('mapH').value = state.map_height_m;\n"
        "      mapFieldsInitialized = true;\n"
        "    }\n  } catch (error)")
    html = html.replace(
        "    const output = await post('/api/save', {\n"
        "      map_width_m:",
        "    const output = await post('/api/save', {\n"
        "      map_origin_x_m: Number("
        "document.getElementById('mapOriginX').value),\n"
        "      map_origin_y_m: Number("
        "document.getElementById('mapOriginY').value),\n"
        "      map_width_m:")
    return html


class OriginAwareBevLayoutCalibratorNode(
        calibrator_module.BevLayoutCalibratorNode):
    """Add editable map origin to the latest hardened registration node."""

    def __init__(self):
        # Used by the wrapped Flask routes while the base constructor starts
        # the web server. Launch overrides are applied immediately afterwards.
        self.map_origin_x_m = -0.40
        self.map_origin_y_m = -0.80
        super().__init__()
        if not self.has_parameter('default_map_origin_x_m'):
            self.declare_parameter('default_map_origin_x_m', -0.40)
        if not self.has_parameter('default_map_origin_y_m'):
            self.declare_parameter('default_map_origin_y_m', -0.80)
        self.map_origin_x_m = _finite(
            'default_map_origin_x_m',
            self.get_parameter('default_map_origin_x_m').value)
        self.map_origin_y_m = _finite(
            'default_map_origin_y_m',
            self.get_parameter('default_map_origin_y_m').value)
        if self.append_existing_layout:
            existing = calibrator_module.load_layout_yaml(str(self.layout_path))
            if existing is not None:
                self.map_origin_x_m = float(existing.get(
                    'map_origin_x_m', self.map_origin_x_m))
                self.map_origin_y_m = float(existing.get(
                    'map_origin_y_m', self.map_origin_y_m))
                self.map_width_m = float(existing.get(
                    'map_width_m', self.map_width_m))
                self.map_height_m = float(existing.get(
                    'map_height_m', self.map_height_m))

    def _make_flask_app(self):
        app = super()._make_flask_app()
        original_state = app.view_functions['state']
        original_save = app.view_functions['save']

        def index_with_origin():
            return calibrator_module.Response(
                _origin_aware_calibrator_html(),
                mimetype='text/html; charset=utf-8')

        def state_with_origin():
            response = original_state()
            payload = dict(response.get_json())
            payload.update({
                'map_origin_x_m': self.map_origin_x_m,
                'map_origin_y_m': self.map_origin_y_m,
                'map_width_m': self.map_width_m,
                'map_height_m': self.map_height_m,
            })
            return calibrator_module.jsonify(payload)

        def save_with_origin():
            request_payload = calibrator_module.request.get_json(
                silent=True) or {}
            try:
                origin_x = _finite(
                    'map_origin_x_m', request_payload.get(
                        'map_origin_x_m', self.map_origin_x_m))
                origin_y = _finite(
                    'map_origin_y_m', request_payload.get(
                        'map_origin_y_m', self.map_origin_y_m))
                map_width = _finite(
                    'map_width_m', request_payload.get(
                        'map_width_m', self.map_width_m))
                map_height = _finite(
                    'map_height_m', request_payload.get(
                        'map_height_m', self.map_height_m))
                if map_width <= 0.0 or map_height <= 0.0:
                    raise ValueError('map width/height must be positive')
            except (TypeError, ValueError) as exc:
                return self._json_error(exc)

            original_renderer = calibrator_module.render_parking_layout_yaml

            def renderer(*args, **kwargs):
                kwargs['map_origin_x_m'] = origin_x
                kwargs['map_origin_y_m'] = origin_y
                return original_renderer(*args, **kwargs)

            calibrator_module.render_parking_layout_yaml = renderer
            try:
                response = original_save()
            finally:
                calibrator_module.render_parking_layout_yaml = original_renderer

            status_code = getattr(response, 'status_code', None)
            response_object = response
            if isinstance(response, tuple):
                response_object = response[0]
                status_code = response[1]
            if status_code is not None and int(status_code) >= 400:
                return response

            self.map_origin_x_m = origin_x
            self.map_origin_y_m = origin_y
            self.map_width_m = map_width
            self.map_height_m = map_height
            metadata_path = self.homography_path.with_suffix('.json')
            try:
                metadata = json.loads(metadata_path.read_text(encoding='utf-8'))
                metadata.update({
                    'map_origin_x_m': origin_x,
                    'map_origin_y_m': origin_y,
                    'map_width_m': map_width,
                    'map_height_m': map_height,
                })
                calibrator_module.write_text_atomic(
                    str(metadata_path),
                    json.dumps(
                        metadata, ensure_ascii=False, indent=2) + '\n')
            except (OSError, TypeError, ValueError, json.JSONDecodeError):
                self.get_logger().warn(
                    'Homography metadata map geometry update failed',
                    throttle_duration_sec=5.0)

            body = dict(response_object.get_json())
            body.update({
                'map_origin_x_m': origin_x,
                'map_origin_y_m': origin_y,
                'map_width_m': map_width,
                'map_height_m': map_height,
            })
            return calibrator_module.jsonify(body)

        app.view_functions['index'] = index_with_origin
        app.view_functions['state'] = state_with_origin
        app.view_functions['save'] = save_with_origin
        return app

    def _render_preview(self):
        frame = self._require_snapshot()
        matrix = self._require_homography()
        cv2 = calibrator_module.cv2
        np = calibrator_module.np
        width = max(1, int(round(self.map_width_m * self.preview_ppm)))
        height = max(1, int(round(self.map_height_m * self.preview_ppm)))
        metre_to_preview = np.array([
            [self.preview_ppm, 0.0,
             -self.map_origin_x_m * self.preview_ppm],
            [0.0, -self.preview_ppm,
             height - 1.0 + self.map_origin_y_m * self.preview_ppm],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)
        preview = cv2.warpPerspective(
            frame, metre_to_preview @ matrix, (width, height))

        def to_px(point):
            return (
                int(round(
                    (point[0] - self.map_origin_x_m) * self.preview_ppm)),
                int(round(
                    height - 1.0 -
                    (point[1] - self.map_origin_y_m) * self.preview_ppm)),
            )

        half_step = max(1, int(round(0.5 * self.preview_ppm)))
        for x_value in range(0, width, half_step):
            major = (x_value % max(1, self.preview_ppm)) == 0
            cv2.line(
                preview, (x_value, 0), (x_value, height - 1),
                (95, 95, 95) if major else (45, 45, 45), 1)
        for y_value in range(0, height, half_step):
            major = (y_value % max(1, self.preview_ppm)) == 0
            cv2.line(
                preview, (0, y_value), (width - 1, y_value),
                (95, 95, 95) if major else (45, 45, 45), 1)

        with self._lock:
            slot_metadata = dict(self._slot_metadata)
            slots = dict(self._slots)
            waiting = None if self._waiting_world is None else list(
                self._waiting_world)
        for slot_id, registered in slots.items():
            corners = slot_metadata[slot_id]['world_corners_m']
            polygon = np.asarray(
                [to_px(point) for point in corners], np.int32)
            hull = cv2.convexHull(polygon)
            cv2.polylines(preview, [hull], True, (80, 230, 100), 2)
            center = to_px(registered.center)
            arrow_end = to_px((
                registered.center_x_m +
                0.35 * math.cos(registered.entry_yaw_rad),
                registered.center_y_m +
                0.35 * math.sin(registered.entry_yaw_rad)))
            cv2.arrowedLine(preview, center, arrow_end, (80, 230, 100), 2)
            cv2.putText(
                preview, slot_id, center,
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (80, 230, 100), 1)
        if waiting is not None:
            polygon = np.asarray(
                [to_px(point) for point in waiting], np.int32)
            cv2.polylines(
                preview, [cv2.convexHull(polygon)], True,
                (40, 170, 255), 2)
        origin_label = (
            f'origin ({self.map_origin_x_m:.2f},'
            f'{self.map_origin_y_m:.2f})')
        cv2.putText(
            preview, origin_label, (6, height - 8),
            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        return preview


class OriginAwareCctvMergeNode(cctv_module.CctvMergeNode):
    """Add map origin without losing main's fail-closed camera coverage map."""

    def _read_parameters(self):
        if not self.has_parameter('map_origin_x_m'):
            self.declare_parameter('map_origin_x_m', 0.0)
        if not self.has_parameter('map_origin_y_m'):
            self.declare_parameter('map_origin_y_m', 0.0)
        super()._read_parameters()
        self.map_origin_x_m = _finite(
            'map_origin_x_m', self.get_parameter('map_origin_x_m').value)
        self.map_origin_y_m = _finite(
            'map_origin_y_m', self.get_parameter('map_origin_y_m').value)
        self.grid_w = grid_cell_count(self.map_w_m, self.resolution)
        self.grid_h = grid_cell_count(self.map_h_m, self.resolution)

    def _publish_map(self, merged, latched, coverage_polygons):
        origin_x = self.map_origin_x_m
        origin_y = self.map_origin_y_m
        shifted_merged = [
            _shift_detection(item, origin_x, origin_y) for item in merged]
        shifted_latched = (
            None if latched is None else
            _shift_point(latched, origin_x, origin_y))
        shifted_coverage = {
            camera_id: _shift_polygon(polygon, origin_x, origin_y)
            for camera_id, polygon in coverage_polygons.items()
        }
        original_pose = self.robot_pose
        original_publisher = self.pub_map
        self.robot_pose = {
            role: _shift_pose(pose, origin_x, origin_y)
            for role, pose in original_pose.items()
        }
        self.pub_map = _OriginPublisher(original_publisher, self)
        try:
            super()._publish_map(
                shifted_merged, shifted_latched, shifted_coverage)
        finally:
            self.robot_pose = original_pose
            self.pub_map = original_publisher


class OriginAwareYoloBevMapNode(yolo_module.YoloBevMapNode):
    """Support a non-zero OccupancyGrid origin in single-camera operation."""

    def __init__(self):
        super().__init__()
        if not self.has_parameter('map_origin_x_m'):
            self.declare_parameter('map_origin_x_m', 0.0)
        if not self.has_parameter('map_origin_y_m'):
            self.declare_parameter('map_origin_y_m', 0.0)
        self.map_origin_x_m = _finite(
            'map_origin_x_m', self.get_parameter('map_origin_x_m').value)
        self.map_origin_y_m = _finite(
            'map_origin_y_m', self.get_parameter('map_origin_y_m').value)
        self.grid_w = grid_cell_count(self.map_w_m, self.resolution)
        self.grid_h = grid_cell_count(self.map_h_m, self.resolution)

    def publish_map_periodic(self):
        origin_x = self.map_origin_x_m
        origin_y = self.map_origin_y_m
        original_obstacles = self.latest_obstacles
        original_pose = self.robot_pose
        original_publisher = self.pub_map
        shifted_obstacles = []
        for obstacle in original_obstacles:
            shifted = dict(obstacle)
            shifted['center'] = _shift_point(
                obstacle['center'], origin_x, origin_y)
            shifted['polygon'] = _shift_polygon(
                obstacle.get('polygon'), origin_x, origin_y)
            shifted_obstacles.append(shifted)
        self.latest_obstacles = shifted_obstacles
        self.robot_pose = {
            role: _shift_pose(pose, origin_x, origin_y)
            for role, pose in original_pose.items()
        }
        self.pub_map = _OriginPublisher(original_publisher, self)
        try:
            super().publish_map_periodic()
        finally:
            self.latest_obstacles = original_obstacles
            self.robot_pose = original_pose
            self.pub_map = original_publisher


class HomeAwareIndividualMoveNode(move_module.IndividualMoveNode):
    """Restore HOME heading and avoid rotating beside the parked peer."""

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        if not self.has_parameter('home_yaw_deg'):
            self.declare_parameter('home_yaw_deg', 0.0)
        self.home_yaw = angle_norm(math.radians(_finite(
            'home_yaw_deg', self.get_parameter('home_yaw_deg').value)))

    def run_approach(self):
        if self.phase == 'WAIT_FRONT_STAGED':
            self.stop()
            if self.front_staged:
                self.set_phase('WAIT_TARGET')
            elif self.phase_timed_out():
                return
            return
        if self.phase == 'WAIT_TARGET':
            self.latch_target_and_plan()
            return
        if self.phase_timed_out():
            return
        if self.phase == 'TO_REAR_STAGING':
            target_yaw = self.active_target[2]
            # In Front-first mode, first translate away from the peer at HOME.
            # PRE_ALIGN owns target-yaw convergence after clearance is available.
            staging_yaw = target_yaw if self.simultaneous_entry else None
            if self.advance_route(
                    self.centerline_speed, goal_yaw=staging_yaw):
                self.stop()
                self.set_phase('READY_TO_SCAN')
                self.publish_approach_ready_if_observed()
            return
        if self.phase == 'READY_TO_SCAN':
            self.stop()
            self.publish_approach_ready_if_observed()

    def run_return(self):
        if self.phase == 'WAIT_PEER_RETURN':
            self.stop()
            if (self.peer_robot_state == 'RETURN' or
                    self.peer_reached_phase(
                        'WAIT_PEER_RETURN', 'EXIT_UNDERBODY',
                        'WAIT_PEER_EXIT_CLEAR', 'EXIT_TO_SIDE',
                        'WAIT_PEER_SIDE_CLEAR', 'RETURN_HOME')):
                self.start_exit()
            return
        if self.phase == 'WAIT_EXIT_ODOM':
            self.start_exit()
            return
        if self.phase_timed_out():
            return
        if self.phase == 'EXIT_UNDERBODY':
            if self.move_pose_toward(
                    self.exit_goal[0], self.exit_goal[1], self.exit_yaw,
                    self.synchronized_exit_speed(),
                    self.position_tolerance):
                if self.same_direction_exit:
                    self.set_phase('WAIT_PEER_EXIT_CLEAR')
                else:
                    self.set_phase('EXIT_TO_SIDE')
            return
        if self.phase == 'WAIT_PEER_EXIT_CLEAR':
            self.stop()
            if self.peer_reached_phase(
                    'WAIT_PEER_EXIT_CLEAR', 'EXIT_TO_SIDE',
                    'WAIT_PEER_SIDE_CLEAR', 'RETURN_HOME', 'RETURNED'):
                self.set_phase('EXIT_TO_SIDE')
            return
        if self.phase == 'EXIT_TO_SIDE':
            if self.move_pose_toward(
                    self.side_exit_goal[0], self.side_exit_goal[1],
                    self.exit_yaw, self.centerline_speed,
                    self.position_tolerance):
                if self.same_direction_exit:
                    self.set_phase('WAIT_PEER_SIDE_CLEAR')
                else:
                    self.plan_return_home()
            return
        if self.phase == 'WAIT_PEER_SIDE_CLEAR':
            self.stop()
            if self.peer_reached_phase(
                    'WAIT_PEER_SIDE_CLEAR', 'RETURN_HOME', 'RETURNED'):
                self.plan_return_home()
            return
        if self.phase == 'RETURN_HOME':
            goal_yaw = self.home_yaw if len(self.route) <= 1 else None
            if self.advance_route(self.max_speed, goal_yaw=goal_yaw):
                self.stop()
                self.set_phase('RETURNED')
                if not self.return_sent:
                    self.pub_return_done.publish(Bool(data=True))
                    self.return_sent = True
            return
        if self.phase == 'RETURNED':
            self.stop()


def _spin(node_type: Type, args=None):
    rclpy.init(args=args)
    node = node_type()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def bev_layout_calibrator_main(args=None):
    _spin(OriginAwareBevLayoutCalibratorNode, args)


def cctv_merge_main(args=None):
    _spin(OriginAwareCctvMergeNode, args)


def yolo_bev_map_main(args=None):
    _spin(OriginAwareYoloBevMapNode, args)


def individual_move_main(args=None):
    _spin(HomeAwareIndividualMoveNode, args)
