#!/usr/bin/env python3
"""Jetson MJPEG monitor plus the touchscreen operator UI.

This node stays a *view*. It renders status and forwards operator mission
intents on ``/ui/mission_request``; it never decides
whether a mission may start. ``fleet_manager_node`` owns that gate, so
killing this process must never change robot behaviour.

Mission outputs remain the responsibility of ``yolo_bev_map_node`` and
``cctv_robot_marker_node``. It subscribes to the rectified ROS image rather
than reopening the camera for every Flask client.

Endpoints:
  ``/``        기존 진단 페이지
  ``/kiosk``   7" 터치스크린용 운용 화면 (1024x600)
  ``/api/status``  상태 JSON (500 ms 폴링)
  ``/api/park``    입차 요청 (서버측 조건 재검사 후 발행)
  ``/api/retrieve`` 출차 요청
"""

from __future__ import annotations

import json
import math
import queue
import threading
import time
import uuid
from importlib.resources import files as resource_files

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, QoSProfile, ReliabilityPolicy,
)
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

from cooperative_parking_robot.aruco_utils import ArucoDetectorCompat
from cooperative_parking_robot.latest_qos import (
    SENSOR_LATEST_QOS,
    STATE_LATEST_QOS,
)
from cooperative_parking_robot.camera_calibration import (
    load_camera_calibration,
    scale_camera_matrix,
)
from cooperative_parking_robot.parking_registry import (
    normalize_vehicle_number,
    validate_parking_password,
)
from cooperative_parking_robot.parking_geometry import (
    parse_registered_slots,
    slot_polygon,
)
from cooperative_parking_robot.vision_utils import (
    parse_class_ids,
    pnp_distance_m,
    select_marker_by_id,
)

try:
    import cv2
    import numpy as np
    from cv_bridge import CvBridge
    from flask import Flask, Response, jsonify, request
    from werkzeug.serving import make_server
    WEB_DEPS_OK = True
except ImportError:
    WEB_DEPS_OK = False

# 1024x600 Waveshare 7" Display-C layout. Assets are local because the
# exhibition network may not provide internet access.
_WEB_ASSET_DIR = resource_files('cooperative_parking_robot').joinpath('web')
KIOSK_PAGE = _WEB_ASSET_DIR.joinpath('kiosk.html').read_text(encoding='utf-8')
KIOSK_CSS = _WEB_ASSET_DIR.joinpath('kiosk.css').read_text(encoding='utf-8')
KIOSK_JS = _WEB_ASSET_DIR.joinpath('kiosk.js').read_text(encoding='utf-8')


MAP_PAGE = """<!DOCTYPE html>
<html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Parkingbot BEV Map</title><style>
*{box-sizing:border-box}html,body{width:100%;height:100%;margin:0}
body{background:#0b1118;color:#e5edf5;font-family:"Noto Sans KR","Malgun Gothic",
 sans-serif;display:flex;flex-direction:column;overflow:hidden}
header{height:58px;display:flex;align-items:center;gap:18px;padding:0 20px;
 background:#141e29;border-bottom:1px solid #334155}
h1{font-size:21px;margin:0}#state{font-weight:800;color:#f59e0b}
.legend{margin-left:auto;display:flex;gap:14px;font-size:14px;color:#cbd5e1}
.key:before{content:"";display:inline-block;width:13px;height:13px;margin-right:5px;
 vertical-align:-2px;border-radius:2px}.waiting:before{background:#16a34a}
.slot:before{background:#0ea5e9}.start:before{background:#f59e0b}
.occupied:before{background:#ef4444}
main{flex:1;min-height:0;display:flex;align-items:center;justify-content:center;
 padding:12px}img{max-width:100%;max-height:100%;object-fit:contain;
 border:1px solid #334155;border-radius:8px;background:#111827}
@media(max-width:760px){header{height:72px;flex-wrap:wrap;gap:5px 12px;padding:7px 12px}
h1{font-size:17px}.legend{width:100%;margin:0;font-size:11px;gap:9px}}
</style></head><body><header><h1>Parkingbot 실시간 BEV / Occupancy Map</h1>
<span id="state">맵 대기 중</span><div class="legend">
<span class="key waiting">WAITING</span><span class="key slot">P1–P4</span>
<span class="key start">ROBOT START</span><span class="key occupied">OCCUPIED</span>
</div></header><main><img src="/map_feed" alt="실시간 주차장 맵"></main>
<script>
function poll(){fetch('/api/map_status').then(function(r){return r.json();})
.then(function(s){var e=document.getElementById('state');
 if(!s.received){e.textContent='맵 대기 중';e.style.color='#f59e0b';}
 else if(s.fresh){e.textContent='LIVE · '+s.sequence;e.style.color='#22c55e';}
 else{e.textContent='STALE · '+s.age_s.toFixed(1)+'초';e.style.color='#ef4444';}})
.catch(function(){var e=document.getElementById('state');e.textContent='연결 끊김';
e.style.color='#ef4444';});}poll();setInterval(poll,500);
</script></body></html>"""


def _polygon_from_flat(values, name, minimum_points=3):
    values = [float(value) for value in values]
    if len(values) < 2 * minimum_points or len(values) % 2:
        raise ValueError(
            f'{name} must contain x,y pairs for at least '
            f'{minimum_points} points')
    if not all(math.isfinite(value) for value in values):
        raise ValueError(f'{name} must contain only finite values')
    return [tuple(values[index:index + 2])
            for index in range(0, len(values), 2)]


def render_occupancy_map(
        data, width, height, resolution, origin_x, origin_y,
        waiting_polygon, slots, robot_start_polygon, robot_starts,
        pixels_per_m=120, received=True):
    """OccupancyGrid와 현장 layout을 한 장의 BGR 이미지로 렌더링한다."""
    width, height = int(width), int(height)
    resolution = float(resolution)
    pixels_per_m = int(pixels_per_m)
    if width <= 0 or height <= 0 or resolution <= 0.0:
        raise ValueError('invalid OccupancyGrid metadata')
    if pixels_per_m < 20:
        raise ValueError('pixels_per_m must be at least 20')
    grid = np.asarray(data, dtype=np.int16)
    if grid.size != width * height:
        raise ValueError(
            f'OccupancyGrid size mismatch: {grid.size} != {width * height}')
    grid = grid.reshape((height, width))

    # ROS OccupancyGrid의 첫 행은 y가 가장 작은 행이다. 화면에서는 y가 위로
    # 증가하도록 뒤집어 map frame의 방향을 그대로 보존한다.
    colors = np.zeros((height, width, 3), dtype=np.uint8)
    colors[:] = (43, 52, 64)
    colors[grid < 0] = (76, 82, 91)
    colors[(grid >= 0) & (grid < 50)] = (30, 41, 53)
    colors[grid >= 50] = (52, 68, 220)

    plot_w = max(1, int(round(width * resolution * pixels_per_m)))
    plot_h = max(1, int(round(height * resolution * pixels_per_m)))
    left, right, top, bottom = 58, 20, 42, 48
    canvas = np.full(
        (top + plot_h + bottom, left + plot_w + right, 3),
        (11, 17, 24), dtype=np.uint8)
    resized = cv2.resize(
        np.flipud(colors), (plot_w, plot_h), interpolation=cv2.INTER_NEAREST)
    canvas[top:top + plot_h, left:left + plot_w] = resized

    def world_to_pixel(point):
        x_m, y_m = point
        return (
            int(round(left + (x_m - origin_x) * pixels_per_m)),
            int(round(top + plot_h - (y_m - origin_y) * pixels_per_m)),
        )

    # 40 cm 바닥 타일 격자를 얇게 표시해 homography 스케일도 눈으로 확인한다.
    x_min, x_max = origin_x, origin_x + width * resolution
    y_min, y_max = origin_y, origin_y + height * resolution
    tile = 0.4
    for index in range(int(math.ceil((x_max - x_min) / tile)) + 1):
        x_m = x_min + index * tile
        p1, p2 = world_to_pixel((x_m, y_min)), world_to_pixel((x_m, y_max))
        cv2.line(canvas, p1, p2, (55, 68, 82), 1, cv2.LINE_AA)
    for index in range(int(math.ceil((y_max - y_min) / tile)) + 1):
        y_m = y_min + index * tile
        p1, p2 = world_to_pixel((x_min, y_m)), world_to_pixel((x_max, y_m))
        cv2.line(canvas, p1, p2, (55, 68, 82), 1, cv2.LINE_AA)

    def draw_zone(polygon, color, label, alpha=0.20):
        points = np.asarray(
            [world_to_pixel(point) for point in polygon], dtype=np.int32)
        overlay = canvas.copy()
        cv2.fillPoly(overlay, [points], color)
        cv2.addWeighted(overlay, alpha, canvas, 1.0 - alpha, 0.0, canvas)
        cv2.polylines(canvas, [points], True, color, 2, cv2.LINE_AA)
        center = tuple(np.mean(points, axis=0).astype(int))
        size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)[0]
        cv2.putText(
            canvas, label, (center[0] - size[0] // 2, center[1] + size[1] // 2),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (238, 245, 252), 2,
            cv2.LINE_AA)

    draw_zone(waiting_polygon, (55, 176, 76), 'WAITING')
    for slot_id, polygon in slots:
        draw_zone(polygon, (235, 165, 14), str(slot_id), alpha=0.12)
    draw_zone(robot_start_polygon, (11, 158, 245), 'ROBOT START', alpha=0.15)

    for role, (x_m, y_m, yaw_deg) in robot_starts:
        center = world_to_pixel((x_m, y_m))
        yaw = math.radians(float(yaw_deg))
        tip = world_to_pixel((
            x_m + 0.28 * math.cos(yaw),
            y_m + 0.28 * math.sin(yaw)))
        cv2.circle(canvas, center, 8, (11, 158, 245), -1, cv2.LINE_AA)
        cv2.arrowedLine(
            canvas, center, tip, (255, 255, 255), 2, cv2.LINE_AA,
            tipLength=0.35)
        cv2.putText(
            canvas, str(role).upper(), (center[0] + 10, center[1] - 9),
            cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 224, 160), 1,
            cv2.LINE_AA)

    cv2.rectangle(
        canvas, (left, top), (left + plot_w, top + plot_h),
        (148, 163, 184), 1)
    title = (
        f"{'LIVE' if received else 'WAITING'} /parking/map  "
        f'{width * resolution:.2f} x {height * resolution:.2f} m  '
        f'{resolution:.3f} m/cell')
    cv2.putText(
        canvas, title, (left, 27), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
        (209, 250, 229) if received else (120, 190, 255), 1, cv2.LINE_AA)
    cv2.putText(
        canvas, 'X [m]', (left + plot_w - 48, top + plot_h + 34),
        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 193, 207), 1, cv2.LINE_AA)
    cv2.putText(
        canvas, 'Y', (18, top + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
        (180, 193, 207), 1, cv2.LINE_AA)
    return canvas


class JetsonVisionWebNode(Node):
    def __init__(self):
        super().__init__('jetson_vision_web_node')

        self.declare_parameter('image_topic', '/cctv/image_rect')
        self.declare_parameter('annotated_topic', '/cctv/debug/annotated')
        self.declare_parameter('enable_aruco', True)
        self.declare_parameter('aruco_dict', 'DICT_4X4_50')
        self.declare_parameter('front_marker_id', 2)
        self.declare_parameter('rear_marker_id', 1)
        self.declare_parameter('marker_size_m', 0.24)
        self.declare_parameter('min_marker_area_px', 100.0)
        self.declare_parameter('min_marker_area_ratio', 0.0003)
        self.declare_parameter(
            'camera_calib', 'cctv_camera_calibration.npz')
        self.declare_parameter('calibration_width_px', 0)
        self.declare_parameter('calibration_height_px', 0)
        self.declare_parameter('jpeg_quality', 70)
        self.declare_parameter('web_host', '0.0.0.0')
        self.declare_parameter('web_port', 5000)
        # ===== 터치 UI =====
        self.declare_parameter('enable_operator_ui', True)
        self.declare_parameter('enable_debug_overlay', False)
        # WiFi 너머 RPi 토픽이므로 오래된 값을 현재 상태로 표시하면 안 된다.
        self.declare_parameter('status_stale_s', 3.0)
        self.declare_parameter('ui_button_cooldown_s', 2.0)
        self.declare_parameter('localization_warning_streak', 5)
        # cam2 화면은 ROS map +x와 좌우가 반대다. 고객 도식은 실제 CCTV
        # 화면을 기준으로 보여 주므로 전체 배치를 좌우 반전한다.
        self.declare_parameter('site_plan_mirror_x', True)
        # ===== 실제 BEV/Occupancy 맵 화면 =====
        self.declare_parameter('map_topic', '/parking/map')
        self.declare_parameter('map_pixels_per_m', 120)
        self.declare_parameter('map_stale_s', 3.0)
        self.declare_parameter('map_resolution', 0.05)
        self.declare_parameter('map_width_m', 4.40)
        self.declare_parameter('map_height_m', 3.83)
        self.declare_parameter(
            'waiting_polygon',
            [0.0, 0.0, 1.2, 0.0, 1.2, 0.8, 0.0, 0.8])
        self.declare_parameter('slot_ids', ['P1', 'P2', 'P3', 'P4'])
        self.declare_parameter(
            'slot_coords', [1.2, 2.2, 2.0, 2.2, 2.8, 2.2, 3.6, 2.2])
        self.declare_parameter(
            'slot_sizes', [1.2, 0.8, 1.2, 0.8, 1.2, 0.8, 1.2, 0.8])
        self.declare_parameter('slot_yaws_deg', [90.0, 90.0, 90.0, 90.0])
        self.declare_parameter('slot_polygons', [0.0])
        self.declare_parameter(
            'robot_start_polygon',
            [3.2, 0.0, 4.0, 0.0, 4.0, 0.8, 3.2, 0.8])
        self.declare_parameter('front_start_pose', [3.6, 0.6, 180.0])
        self.declare_parameter('rear_start_pose', [3.6, 0.2, 180.0])

        if not WEB_DEPS_OK:
            raise RuntimeError(
                'Jetson web monitor dependencies missing: cv2, numpy, '
                'cv_bridge, flask, werkzeug')

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.annotated_topic = str(
            self.get_parameter('annotated_topic').value)
        self.enable_debug_overlay = bool(
            self.get_parameter('enable_debug_overlay').value)
        requested_enable_aruco = bool(
            self.get_parameter('enable_aruco').value)
        self.enable_aruco = (
            self.enable_debug_overlay and requested_enable_aruco)
        self.marker_size = float(self.get_parameter('marker_size_m').value)
        self.min_marker_area_px = float(
            self.get_parameter('min_marker_area_px').value)
        self.min_marker_area_ratio = float(
            self.get_parameter('min_marker_area_ratio').value)
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self.web_host = str(self.get_parameter('web_host').value)
        self.web_port = int(self.get_parameter('web_port').value)
        self.calibration_width = int(
            self.get_parameter('calibration_width_px').value)
        self.calibration_height = int(
            self.get_parameter('calibration_height_px').value)
        self.enable_operator_ui = bool(
            self.get_parameter('enable_operator_ui').value)
        self.status_stale_s = float(
            self.get_parameter('status_stale_s').value)
        self.ui_button_cooldown = float(
            self.get_parameter('ui_button_cooldown_s').value)
        self.localization_warning_streak = int(
            self.get_parameter('localization_warning_streak').value)
        self.site_plan_mirror_x = bool(
            self.get_parameter('site_plan_mirror_x').value)
        self.map_topic = str(self.get_parameter('map_topic').value)
        self.map_pixels_per_m = int(
            self.get_parameter('map_pixels_per_m').value)
        self.map_stale_s = float(self.get_parameter('map_stale_s').value)
        self.map_resolution = float(
            self.get_parameter('map_resolution').value)
        self.map_width_m = float(self.get_parameter('map_width_m').value)
        self.map_height_m = float(self.get_parameter('map_height_m').value)
        if self.status_stale_s <= 0.0 or self.ui_button_cooldown < 0.0:
            raise ValueError('invalid UI status/cooldown parameters')
        if self.localization_warning_streak <= 0:
            raise ValueError('localization_warning_streak must be positive')
        if (not self.map_topic or self.map_pixels_per_m < 20 or
                self.map_stale_s <= 0.0 or self.map_resolution <= 0.0 or
                self.map_width_m <= 0.0 or self.map_height_m <= 0.0):
            raise ValueError('invalid map viewer parameters')

        self.waiting_polygon = _polygon_from_flat(
            self.get_parameter('waiting_polygon').value,
            'waiting_polygon')
        slots = parse_registered_slots(
            self.get_parameter('slot_ids').value,
            self.get_parameter('slot_coords').value,
            self.get_parameter('slot_sizes').value,
            self.get_parameter('slot_yaws_deg').value)
        polygon_flat = list(self.get_parameter('slot_polygons').value)
        if len(polygon_flat) == 1 and float(polygon_flat[0]) == 0.0:
            polygons = [slot_polygon(slot) for slot in slots]
        elif len(polygon_flat) == 8 * len(slots):
            polygons = [
                _polygon_from_flat(
                    polygon_flat[index * 8:(index + 1) * 8],
                    f'slot_polygons[{index}]', minimum_points=4)
                for index in range(len(slots))]
        else:
            raise ValueError(
                'slot_polygons must contain eight values per slot')
        self.map_slots = [
            (slot.slot_id, polygon)
            for slot, polygon in zip(slots, polygons)]
        self.robot_start_polygon = _polygon_from_flat(
            self.get_parameter('robot_start_polygon').value,
            'robot_start_polygon')
        self.robot_starts = []
        for role, parameter in (
                ('front', 'front_start_pose'), ('rear', 'rear_start_pose')):
            pose = [float(value) for value in
                    self.get_parameter(parameter).value]
            if len(pose) != 3 or not all(math.isfinite(value) for value in pose):
                raise ValueError(f'{parameter} must contain finite x,y,yaw_deg')
            self.robot_starts.append((role, tuple(pose)))

        if not self.image_topic or not self.annotated_topic:
            raise ValueError('image and annotated topics must not be empty')
        if self.marker_size <= 0.0:
            raise ValueError('marker_size_m must be positive')
        if self.min_marker_area_px < 0.0 or self.min_marker_area_ratio < 0.0:
            raise ValueError('marker area thresholds must be non-negative')
        if not (1 <= self.jpeg_quality <= 100):
            raise ValueError('jpeg_quality must be in [1,100]')
        if not (1 <= self.web_port <= 65535):
            raise ValueError('web_port must be in [1,65535]')
        if bool(self.calibration_width) != bool(self.calibration_height):
            raise ValueError(
                'calibration_width_px and calibration_height_px must both '
                'be zero or both be positive')

        self.marker_ids = parse_class_ids([
            self.get_parameter('front_marker_id').value,
            self.get_parameter('rear_marker_id').value,
        ])

        self.bridge = CvBridge()

        self.detector = None
        if self.enable_aruco:
            self.detector = ArucoDetectorCompat(
                cv2, str(self.get_parameter('aruco_dict').value))

        self.base_camera_matrix = None
        self.effective_camera_matrix = None
        self.effective_size = None
        if self.enable_aruco:
            calibration_path = str(
                self.get_parameter('camera_calib').value)
            try:
                (self.base_camera_matrix,
                 _dist_coeffs,
                 source_keys) = load_camera_calibration(calibration_path)
                self.get_logger().info(
                    'debug ArUco calibration loaded | '
                    f'keys={source_keys[0]}/{source_keys[1]}')
            except Exception as exc:
                self.get_logger().warn(
                    'ArUco distance/axes disabled because calibration could '
                    f'not be loaded: {exc}')

        half = self.marker_size / 2.0
        self.object_points = np.array([
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0],
        ], dtype=np.float32)

        self.annotated_publisher = None
        if self.enable_debug_overlay:
            self.annotated_publisher = self.create_publisher(
                Image, self.annotated_topic, SENSOR_LATEST_QOS)

        # Subscriptions can invoke callbacks as soon as they are created.
        # Allocate every callback-owned buffer first so the first camera frame
        # cannot race node construction.
        self._frame_condition = threading.Condition()
        self._latest_frame = None
        self._latest_header = None
        self._input_sequence = 0
        self._processed_sequence = 0
        self._jpeg_condition = threading.Condition()
        self._latest_jpeg = None
        self._jpeg_sequence = 0
        self._map_condition = threading.Condition()
        self._latest_map_jpeg = None
        self._map_sequence = 0
        self._map_received = False
        self._last_map_time = None
        self._stop_event = threading.Event()
        self._last_process_time = None

        self.create_subscription(
            Image, self.image_topic, self.image_cb, SENSOR_LATEST_QOS)
        self.create_subscription(
            OccupancyGrid, self.map_topic, self.map_cb, 10)

        placeholder_width = max(
            1, int(round(self.map_width_m / self.map_resolution)))
        placeholder_height = max(
            1, int(round(self.map_height_m / self.map_resolution)))
        placeholder = render_occupancy_map(
            [-1] * (placeholder_width * placeholder_height),
            placeholder_width, placeholder_height, self.map_resolution,
            0.0, 0.0, self.waiting_polygon, self.map_slots,
            self.robot_start_polygon, self.robot_starts,
            self.map_pixels_per_m, received=False)
        ok, buffer = cv2.imencode(
            '.jpg', placeholder,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if ok:
            self._latest_map_jpeg = buffer.tobytes()
            self._map_sequence = 1

        # ===== 터치 UI 상태/발행 =====
        self._status_lock = threading.Lock()
        self._status = {}
        self._localization_reject_streak = {'front': 0, 'rear': 0}
        self._ui_queue = queue.Queue(maxsize=16)
        self._ui_sequence = 0
        self._ui_client_id = f'web-{uuid.uuid4()}'
        self._last_park_publish = 0.0
        self._last_retrieve_publish = 0.0
        if self.enable_operator_ui:
            self._setup_operator_ui()

        self._worker_thread = threading.Thread(
            target=self._worker_loop,
            name='jetson-vision-worker',
            daemon=True,
        )
        self._worker_thread.start()

        self._flask_app = self._make_flask_app()
        self._web_server = make_server(
            self.web_host,
            self.web_port,
            self._flask_app,
            threaded=True,
        )
        self._web_thread = threading.Thread(
            target=self._web_server.serve_forever,
            name='jetson-vision-web',
            daemon=True,
        )
        self._web_thread.start()

        mode = ('operator UI + debug overlay' if self.enable_debug_overlay
                and self.enable_operator_ui else
                'debug overlay' if self.enable_debug_overlay else
                'operator UI')
        self.get_logger().warn(
            f'{mode} web listening on http://{self.web_host}:'
            f'{self.web_port}/ without authentication; trusted LAN only')
        self.get_logger().info(
            f'Jetson vision web monitor: camera={self.image_topic}, '
            f'map={self.map_topic} -> /map')

    # ==================================================
    # 터치 UI — 상태 수집
    # ==================================================
    def _setup_operator_ui(self) -> None:
        """상태 구독과 미션 요청 발행자를 만든다.

        모든 콜백은 값과 수신 시각만 저장한다. 표시 판단은 /api/status에서
        하는데, 그래야 staleness 기준을 한 곳에서 일관되게 적용할 수 있다.
        """
        def store(key):
            def callback(msg, key=key):
                with self._status_lock:
                    self._status[key] = (msg.data, time.monotonic())
            return callback

        self.create_subscription(
            String, '/fleet/state', store('fleet'), STATE_LATEST_QOS)
        self.create_subscription(Bool, '/parking/target_ready',
                                 store('target_ready'), 10)
        self.create_subscription(String, '/parking/target_status',
                                 store('target_status'), 10)
        self.create_subscription(String, '/sync/error_state',
                                 store('sync_error'), 10)
        for role in ('front', 'rear'):
            self.create_subscription(
                String, f'/{role}/robot_state', store(f'{role}_state'),
                STATE_LATEST_QOS)
            self.create_subscription(
                String, f'/{role}/motion_phase', store(f'{role}_phase'), 10)
            self.create_subscription(
                String, f'/{role}/motion_fault', store(f'{role}_fault'), 10)
            self.create_subscription(
                String, f'/{role}/localization_status',
                lambda msg, r=role: self._localization_cb(r, msg), 10)

        request_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pub_ui_request = self.create_publisher(
            String, '/ui/mission_request', request_qos)
        # Flask 워커 스레드에서 직접 publish하지 않는다. 큐를 통해 rclpy
        # 실행 컨텍스트로 넘긴다.
        self.create_timer(0.05, self._drain_ui_queue)

    def _localization_cb(self, role, msg) -> None:
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        source = str(payload.get('source', ''))
        if source == 'CCTV_REJECTED_GATE':
            self._localization_reject_streak[role] += 1
        elif source.startswith('CCTV'):
            self._localization_reject_streak[role] = 0
        with self._status_lock:
            self._status[f'{role}_localization'] = (
                payload, time.monotonic())

    def _drain_ui_queue(self) -> None:
        while True:
            try:
                kind, payload = self._ui_queue.get_nowait()
            except queue.Empty:
                return
            if kind == 'mission':
                self.pub_ui_request.publish(String(data=payload))
                try:
                    envelope = json.loads(payload)
                    request_type = str(envelope.get('type', ''))
                    request_id = str(envelope.get('request_id', ''))
                except (TypeError, ValueError):
                    request_type, request_id = '', ''
                self.get_logger().info(
                    'UI mission request 발행: '
                    f'type={request_type}, request_id={request_id}')

    def _fresh(self, key):
        """(값, 신선함) 반환. 값이 없으면 (None, False)."""
        with self._status_lock:
            entry = self._status.get(key)
        if entry is None:
            return None, False
        value, stamp = entry
        return value, (time.monotonic() - stamp) <= self.status_stale_s

    def _json_field(self, key, field, default=None):
        raw, fresh = self._fresh(key)
        if raw is None:
            return default, fresh
        try:
            return json.loads(raw).get(field, default), fresh
        except (TypeError, ValueError):
            return default, False

    _PHASE_TEXT = {
        'WAIT_TARGET': '차량 대기 중',
        'WAIT_LIFT': '로봇 진입 중',
        'PLAN_PATH': '경로 계산 중',
        'NAVIGATING': '차량 운반 중',
    }

    def _ui_polygon(self, polygon) -> list[list[float]]:
        """Convert map metres to the camera-oriented 0..100 UI canvas."""
        width = float(getattr(self, 'map_width_m', 0.0))
        height = float(getattr(self, 'map_height_m', 0.0))
        if width <= 0.0 or height <= 0.0:
            return []
        converted = []
        for point in polygon:
            x = max(0.0, min(100.0, float(point[0]) / width * 100.0))
            if getattr(self, 'site_plan_mirror_x', True):
                x = 100.0 - x
            # ROS map +y points up; the SVG/camera image +y points down.
            y = max(
                0.0,
                min(100.0, (height - float(point[1])) / height * 100.0),
            )
            converted.append([round(x, 2), round(y, 2)])
        return converted

    def _site_layout_payload(self, fleet, parking_slots) -> tuple[list, dict]:
        """Build a customer-safe view of the configured physical layout."""
        lifecycle_by_id = {
            slot['slot_id']: slot['lifecycle'] for slot in parking_slots}
        available_values = fleet.get('available_slot_ids')
        available_ids = (
            {str(value) for value in available_values}
            if isinstance(available_values, list) else None)
        assigned_id = str(fleet.get('active_destination_slot_id', ''))
        map_slots = list(getattr(self, 'map_slots', []))
        converted_by_id = {
            str(value[0]): self._ui_polygon(value[1])
            for value in map_slots
        }
        # Customer space 1 is the rightmost polygon in the CCTV-oriented UI.
        customer_order = sorted(
            map_slots,
            key=lambda value: sum(
                point[0] for point in converted_by_id[str(value[0])]
            ) / len(converted_by_id[str(value[0])]),
            reverse=True,
        )
        display_number_by_id = {
            str(value[0]): number
            for number, value in enumerate(customer_order, start=1)
        }
        parking_spaces = []
        for value in map_slots:
            slot_id, polygon = value
            lifecycle = lifecycle_by_id.get(str(slot_id), 'UNKNOWN')
            parking_spaces.append({
                # slot_id is retained for diagnostics/API compatibility only.
                # The kiosk renders display_number instead.
                'slot_id': str(slot_id),
                'display_number': display_number_by_id[str(slot_id)],
                'polygon': converted_by_id[str(slot_id)],
                'available': bool(
                    lifecycle == 'EMPTY' and
                    (available_ids is None or str(slot_id) in available_ids)),
                'assigned': bool(
                    assigned_id and str(slot_id) == assigned_id),
            })
        site_layout = {
            'waiting_polygon': self._ui_polygon(
                getattr(self, 'waiting_polygon', [])),
            'robot_start_polygon': self._ui_polygon(
                getattr(self, 'robot_start_polygon', [])),
        }
        return parking_spaces, site_layout

    def build_status(self) -> dict:
        """UI 표시와 버튼 활성 판정을 한 곳에서 계산한다."""
        fleet_raw, fleet_fresh = self._fresh('fleet')
        fleet = {}
        if fleet_raw is not None:
            try:
                fleet = json.loads(fleet_raw)
            except (TypeError, ValueError):
                fleet, fleet_fresh = {}, False
        fleet_state = str(fleet.get('state', 'UNKNOWN'))
        empty_count = int(fleet.get('empty_count', 0))

        target_ready, target_fresh = self._fresh('target_ready')
        target_ready = bool(target_ready) and target_fresh
        target_status_raw, target_status_fresh = self._fresh('target_status')
        target_status = {}
        if target_status_raw is not None and target_status_fresh:
            try:
                target_status = json.loads(target_status_raw)
            except (TypeError, ValueError):
                target_status, target_status_fresh = {}, False
        status_stream_unavailable = (
            target_status_raw is not None and not target_status_fresh)
        target_state = str(target_status.get('state', '')).upper()
        perception_unavailable = (
            status_stream_unavailable or
            target_state == 'PERCEPTION_UNAVAILABLE')
        if perception_unavailable:
            target_state = 'PERCEPTION_UNAVAILABLE'
            target_ready = False
        elif target_state not in (
                'ABSENT', 'DETECTING', 'READY',
                'PERCEPTION_UNAVAILABLE'):
            target_state = 'READY' if target_ready else 'ABSENT'
        # /parking/target_ready remains the authoritative safety gate. A
        # slightly newer status message must never make the UI claim READY
        # while the Bool gate is false.
        if target_ready and not perception_unavailable:
            target_state = 'READY'
        elif target_state == 'READY':
            target_state = (
                'DETECTING'
                if bool(target_status.get('observed_recently', False))
                else 'ABSENT')

        robots = {}
        for role in ('front', 'rear'):
            state, state_fresh = self._fresh(f'{role}_state')
            phase, _ = self._fresh(f'{role}_phase')
            robots[role] = {
                'state': str(state) if state is not None else 'UNKNOWN',
                'phase': str(phase) if phase is not None else '-',
                'fresh': bool(state_fresh),
            }

        fault = None
        for role in ('front', 'rear'):
            reason, fresh = self._fresh(f'{role}_fault')
            if reason and fresh:
                fault = {'source': role, 'reason': str(reason)}
            if robots[role]['state'] == 'FAULT':
                fault = fault or {'source': role, 'reason': 'ROBOT_FAULT'}
        sync_error, sync_fresh = self._json_field('sync_error', 'error')
        if sync_fresh and sync_error not in (None, 'OK', 'ARRIVED'):
            fault = fault or {'source': 'sync', 'reason': str(sync_error)}

        localization_warning = any(
            count >= self.localization_warning_streak
            for count in self._localization_reject_streak.values())
        planning_blocker = fleet.get('planning_blocker')
        if not isinstance(planning_blocker, dict):
            planning_blocker = None
        validation_warnings = [
            {
                'code': str(value.get('code', 'UNKNOWN')),
                'mission_phase': str(value.get('mission_phase', '')),
            }
            for value in fleet.get('validation_warnings', [])
            if isinstance(value, dict)
        ]
        planning_warning = bool(validation_warnings)

        idle = all(robots[role]['state'] == 'IDLE'
                   for role in ('front', 'rear'))
        common_fresh = (
            fleet_fresh and
            all(robots[role]['fresh'] for role in ('front', 'rear')))
        all_fresh = common_fresh and target_fresh

        # Fleet owns the production freshness policy and republishes its
        # evaluated result in /fleet/state, avoiding a second UI-only timeout.
        dimension_ready = bool(
            fleet_fresh and fleet.get('vehicle_spec_ready', False))

        park_enabled = bool(
            target_ready and fleet_state == 'WAIT_TARGET' and
            empty_count >= 1 and idle and fault is None and all_fresh and
            dimension_ready)
        parking_slots = []
        retrieve_gate = bool(
            fleet_state == 'WAIT_TARGET' and idle and fault is None and
            common_fresh and not fleet.get('mission_id'))
        for value in fleet.get('parking_slots', []):
            if not isinstance(value, dict):
                continue
            slot = {
                'slot_id': str(value.get('slot_id', '')),
                'lifecycle': str(value.get('lifecycle', 'UNKNOWN')),
                'retrievable': bool(value.get('retrievable', False)),
            }
            slot['retrieve_enabled'] = bool(
                retrieve_gate and slot['retrievable'])
            parking_slots.append(slot)
        retrieve_enabled = any(
            slot['retrieve_enabled'] for slot in parking_slots)
        parking_spaces, site_layout = self._site_layout_payload(
            fleet, parking_slots)
        site_layout['vehicle_state'] = target_state
        site_layout['vehicle_present'] = target_state in ('DETECTING', 'READY')

        if fault is not None:
            banner = f"오류: {fault['source']} — {fault['reason']}"
        elif not common_fresh:
            banner = '일부 노드와 통신이 끊겼습니다'
        elif planning_blocker is not None:
            banner = (
                '경로 생성 불가: '
                f"{planning_blocker.get('code', 'UNKNOWN')}")
        elif planning_warning:
            banner = (
                '경고 운행 중: ' +
                ', '.join(value['code'] for value in validation_warnings))
        elif fleet_state != 'WAIT_TARGET':
            banner = self._PHASE_TEXT.get(fleet_state, fleet_state)
        elif park_enabled and retrieve_enabled:
            banner = '입차 또는 출차 가능'
        elif park_enabled:
            banner = '차량 인식 완료 — 입차 가능'
        elif retrieve_enabled:
            banner = '출차 가능 — 차량번호와 비밀번호를 입력하세요'
        elif not target_fresh:
            banner = '일부 노드와 통신이 끊겼습니다'
        elif not target_ready:
            banner = (
                '카메라 인식 일시 중단 — 인식 확인 중'
                if target_state == 'PERCEPTION_UNAVAILABLE' else
                '차량 감지 중 — 정차 확인까지 잠시 기다려 주세요'
                if target_state == 'DETECTING'
                else '대기공간에 차량을 x축 방향으로 세워 주세요')
        elif not dimension_ready:
            banner = '차량 크기 측정 중 — 잠시 기다려 주세요'
        elif empty_count < 1:
            banner = '빈 주차면이 없습니다'
        else:
            banner = '로봇 준비 중'

        return {
            'fleet': {
                'state': fleet_state,
                'empty_count': empty_count,
                'ui_approved': bool(fleet.get('ui_approved', False)),
                'require_ui_confirmation': bool(
                    fleet.get('require_ui_confirmation', True)),
                'fresh': bool(fleet_fresh),
            },
            'front': robots['front'],
            'rear': robots['rear'],
            'target_ready': target_ready,
            'target_state': target_state,
            'target_status': target_status,
            'vehicle_dimension_ready': dimension_ready,
            'park_block_reason': (
                '' if park_enabled else
                ('WAITING_VEHICLE_DIMENSION' if not dimension_ready else '')),
            'park_enabled': park_enabled,
            'retrieve_enabled': retrieve_enabled,
            'parking_slots': parking_slots,
            'parking_spaces': parking_spaces,
            'site_layout': site_layout,
            'request_status': fleet.get('request_status'),
            'last_completed': fleet.get('last_completed'),
            'fault': fault,
            'localization_warning': localization_warning,
            'planning_validation_mode': fleet.get(
                'planning_validation_mode', 'enforce'),
            'validation_warnings': validation_warnings,
            'planning_warning': planning_warning,
            'planning_blocker': planning_blocker,
            'banner': banner,
        }

    def request_park(
            self, vehicle_number, password, destination_slot_id=''):
        """서버측에서 조건을 다시 확인한 뒤에만 발행한다."""
        status = self.build_status()
        if not status['park_enabled']:
            return False, status['banner'], status, ''
        try:
            vehicle_number = normalize_vehicle_number(vehicle_number)
            password = validate_parking_password(password)
        except ValueError:
            return False, '차량번호 또는 비밀번호 형식을 확인하세요', status, ''
        destination_slot_id = str(destination_slot_id).strip()
        empty_slots = [
            slot for slot in status['parking_slots']
            if slot['lifecycle'] == 'EMPTY']
        if destination_slot_id:
            selected = next((
                space for space in status.get('parking_spaces', [])
                if space['slot_id'] == destination_slot_id and
                space['available']), None)
            if selected is None:
                return (
                    False, '선택한 주차면을 사용할 수 없습니다', status, '')
        elif not empty_slots:
            return False, '현재 사용할 수 있는 주차공간이 없습니다', status, ''
        now = time.monotonic()
        if now - self._last_park_publish < self.ui_button_cooldown:
            return False, '요청 처리 중입니다', status, ''
        self._ui_sequence += 1
        payload_fields = {
            'type': 'park',
            'vehicle_number': vehicle_number,
            'password': password,
            'request_id': f'ui-{uuid.uuid4()}',
            'client_id': self._ui_client_id,
            'sequence': self._ui_sequence,
            'stamp_ns': self.get_clock().now().nanoseconds,
        }
        # Customer kiosk requests omit the internal slot ID. Fleet Manager
        # chooses from registry-empty AND perception-empty spaces atomically.
        if destination_slot_id:
            payload_fields['destination_slot_id'] = destination_slot_id
        payload = json.dumps(payload_fields)
        try:
            self._ui_queue.put_nowait(('mission', payload))
        except queue.Full:
            return False, '요청 큐가 가득 찼습니다', status, ''
        self._last_park_publish = now
        request_id = json.loads(payload)['request_id']
        return True, '입차 요청을 제출했습니다', status, request_id

    def request_retrieve(self, vehicle_number, password):
        status = self.build_status()
        if not status['retrieve_enabled']:
            return False, '현재 출차할 수 있는 차량이 없습니다', status, ''
        try:
            vehicle_number = normalize_vehicle_number(vehicle_number)
            password = validate_parking_password(password)
        except ValueError:
            return False, '차량번호 또는 비밀번호 형식을 확인하세요', status, ''
        now = time.monotonic()
        if now - self._last_retrieve_publish < self.ui_button_cooldown:
            return False, '요청 처리 중입니다', status, ''
        self._ui_sequence += 1
        request_id = f'ui-{uuid.uuid4()}'
        payload = json.dumps({
            'type': 'retrieve',
            'vehicle_number': vehicle_number,
            'password': password,
            'request_id': request_id,
            'client_id': self._ui_client_id,
            'sequence': self._ui_sequence,
            'stamp_ns': self.get_clock().now().nanoseconds,
        })
        try:
            self._ui_queue.put_nowait(('mission', payload))
        except queue.Full:
            return False, '요청 큐가 가득 찼습니다', status, ''
        self._last_retrieve_publish = now
        return True, '출차 요청을 제출했습니다', status, request_id

    def _make_flask_app(self):
        app = Flask('jetson_vision_web')

        @app.route('/')
        def index():
            if not self.enable_debug_overlay:
                if self.enable_operator_ui:
                    return Response(KIOSK_PAGE, mimetype='text/html')
                return ('debug overlay disabled', 404)
            return (
                '<html><head><title>Jetson Vision</title></head>'
                '<body style="background:#2c3e50;text-align:center;color:white">'
                '<h2>ROS2 CCTV + ArUco</h2>'
                '<img src="/video_feed" style="border-radius:10px;max-width:100%">'
                '</body></html>')

        @app.route('/video_feed')
        def video_feed():
            return Response(
                self._mjpeg_stream(),
                mimetype='multipart/x-mixed-replace; boundary=frame')

        @app.route('/map')
        def map_page():
            return Response(MAP_PAGE, mimetype='text/html')

        @app.route('/map_feed')
        def map_feed():
            return Response(
                self._map_mjpeg_stream(),
                mimetype='multipart/x-mixed-replace; boundary=frame')

        @app.route('/map_snapshot.jpg')
        def map_snapshot():
            with self._map_condition:
                jpeg = self._latest_map_jpeg
            if jpeg is None:
                return ('map not ready', 503)
            return Response(
                jpeg, mimetype='image/jpeg',
                headers={'Cache-Control': 'no-store, max-age=0'})

        @app.route('/api/map_status')
        def api_map_status():
            now = time.monotonic()
            with self._map_condition:
                received = self._map_received
                sequence = self._map_sequence
                last_map_time = self._last_map_time
            age_s = (
                None if last_map_time is None
                else max(0.0, now - last_map_time))
            return jsonify({
                'received': received,
                'fresh': bool(
                    received and age_s is not None and
                    age_s <= self.map_stale_s),
                'age_s': age_s,
                'sequence': sequence,
                'map_topic': self.map_topic,
            })

        @app.route('/kiosk')
        def kiosk():
            if not self.enable_operator_ui:
                return ('operator UI disabled '
                        '(enable_operator_ui:=true)', 404)
            return Response(KIOSK_PAGE, mimetype='text/html')

        @app.route('/assets/kiosk.css')
        def kiosk_css():
            return Response(
                KIOSK_CSS,
                mimetype='text/css',
                headers={'Cache-Control': 'no-store'},
            )

        @app.route('/assets/kiosk.js')
        def kiosk_js():
            return Response(
                KIOSK_JS,
                mimetype='application/javascript',
                headers={'Cache-Control': 'no-store'},
            )

        @app.route('/api/status')
        def api_status():
            if not self.enable_operator_ui:
                return jsonify({'error': 'operator UI disabled'}), 404
            return jsonify(self.build_status())

        @app.route('/api/park', methods=['POST'])
        def api_park():
            if not self.enable_operator_ui:
                return jsonify({'error': 'operator UI disabled'}), 404
            body = request.get_json(silent=True) or {}
            submitted, message, status, request_id = self.request_park(
                body.get('vehicle_number', ''),
                body.get('password', ''),
                body.get('destination_slot_id', ''))
            return jsonify({
                'submitted': submitted,
                'request_id': request_id,
                'message': message,
                'status': status,
            })

        @app.route('/api/retrieve', methods=['POST'])
        def api_retrieve():
            if not self.enable_operator_ui:
                return jsonify({'error': 'operator UI disabled'}), 404
            body = request.get_json(silent=True) or {}
            submitted, message, status, request_id = self.request_retrieve(
                body.get('vehicle_number', ''), body.get('password', ''))
            return jsonify({
                'submitted': submitted,
                'request_id': request_id,
                'message': message,
                'status': status,
            })

        @app.route('/health')
        def health():
            with self._jpeg_condition:
                ready = self._latest_jpeg is not None
                sequence = self._jpeg_sequence
            with self._map_condition:
                map_received = self._map_received
                map_sequence = self._map_sequence
            return jsonify({
                'ready': ready,
                'sequence': sequence,
                'image_topic': self.image_topic,
                'debug_overlay': self.enable_debug_overlay,
                'aruco': self.enable_aruco,
                'map_received': map_received,
                'map_sequence': map_sequence,
                'map_topic': self.map_topic,
            })

        return app

    def _mjpeg_stream(self):
        last_sequence = -1
        while not self._stop_event.is_set():
            with self._jpeg_condition:
                ready = self._jpeg_condition.wait_for(
                    lambda: self._stop_event.is_set()
                    or (self._latest_jpeg is not None
                        and self._jpeg_sequence != last_sequence),
                    timeout=1.0,
                )
                if self._stop_event.is_set():
                    break
                if not ready or self._latest_jpeg is None:
                    continue
                jpeg = self._latest_jpeg
                last_sequence = self._jpeg_sequence
            yield (
                b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n'
                + jpeg + b'\r\n')

    def _map_mjpeg_stream(self):
        last_sequence = -1
        while not self._stop_event.is_set():
            with self._map_condition:
                ready = self._map_condition.wait_for(
                    lambda: self._stop_event.is_set()
                    or (self._latest_map_jpeg is not None
                        and self._map_sequence != last_sequence),
                    timeout=1.0,
                )
                if self._stop_event.is_set():
                    break
                if not ready or self._latest_map_jpeg is None:
                    continue
                jpeg = self._latest_map_jpeg
                last_sequence = self._map_sequence
            yield (
                b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n'
                + jpeg + b'\r\n')

    def map_cb(self, message: OccupancyGrid) -> None:
        try:
            image = render_occupancy_map(
                message.data,
                message.info.width,
                message.info.height,
                message.info.resolution,
                message.info.origin.position.x,
                message.info.origin.position.y,
                self.waiting_polygon,
                self.map_slots,
                self.robot_start_polygon,
                self.robot_starts,
                self.map_pixels_per_m,
                received=True)
            ok, buffer = cv2.imencode(
                '.jpg', image,
                [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
            if not ok:
                raise ValueError('JPEG encoding failed')
        except (TypeError, ValueError) as exc:
            self.get_logger().warn(
                f'OccupancyGrid 렌더링 실패: {exc}',
                throttle_duration_sec=3.0)
            return
        with self._map_condition:
            self._latest_map_jpeg = buffer.tobytes()
            self._map_sequence += 1
            self._map_received = True
            self._last_map_time = time.monotonic()
            self._map_condition.notify_all()

    def image_cb(self, message: Image) -> None:
        frame = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        with self._frame_condition:
            self._latest_frame = frame.copy()
            self._latest_header = message.header
            self._input_sequence += 1
            self._frame_condition.notify()

    def _worker_loop(self) -> None:
        while not self._stop_event.is_set():
            with self._frame_condition:
                ready = self._frame_condition.wait_for(
                    lambda: self._stop_event.is_set()
                    or (self._latest_frame is not None
                        and self._input_sequence != self._processed_sequence),
                    timeout=1.0,
                )
                if self._stop_event.is_set():
                    return
                if not ready or self._latest_frame is None:
                    continue
                frame = self._latest_frame.copy()
                header = self._latest_header
                self._processed_sequence = self._input_sequence

            now = time.monotonic()
            fps = 0.0
            if self._last_process_time is not None:
                delta = now - self._last_process_time
                if delta > 1e-6:
                    fps = 1.0 / delta
            self._last_process_time = now

            display = frame.copy()
            if self.enable_debug_overlay:
                if self.enable_aruco:
                    self._draw_aruco(display)

                cv2.putText(
                    display, f'PROC FPS: {fps:.1f}', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)

            if self.annotated_publisher is not None:
                output = self.bridge.cv2_to_imgmsg(
                    display, encoding='bgr8')
                output.header = header
                self.annotated_publisher.publish(output)

            ok, buffer = cv2.imencode(
                '.jpg', display,
                [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
            if ok:
                with self._jpeg_condition:
                    self._latest_jpeg = buffer.tobytes()
                    self._jpeg_sequence += 1
                    self._jpeg_condition.notify_all()

    def _ensure_effective_intrinsics(self, width: int, height: int) -> None:
        if self.base_camera_matrix is None:
            return
        if self.effective_size == (width, height):
            return
        if self.calibration_width > 0:
            matrix = scale_camera_matrix(
                self.base_camera_matrix,
                self.calibration_width,
                self.calibration_height,
                width,
                height,
            )
        else:
            matrix = self.base_camera_matrix.copy()
            cx = float(matrix[0, 2])
            cy = float(matrix[1, 2])
            if not (0.0 <= cx < width and 0.0 <= cy < height):
                self.get_logger().error(
                    'debug ArUco intrinsics do not match rectified frame; '
                    'distance/axes disabled')
                self.base_camera_matrix = None
                return
        self.effective_camera_matrix = matrix
        self.effective_size = (width, height)

    def _draw_aruco(self, frame) -> None:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detect_markers(gray)
        height, width = frame.shape[:2]
        self._ensure_effective_intrinsics(width, height)

        for marker_id in self.marker_ids:
            selected, area = select_marker_by_id(
                corners, ids, marker_id,
                min_area_px=self.min_marker_area_px,
                min_area_ratio=self.min_marker_area_ratio,
                frame_width=width, frame_height=height)
            if selected is None:
                continue
            points = np.asarray(selected, dtype=np.float32)
            cv2.polylines(
                frame, [points.astype(np.int32)], True, (0, 255, 0), 2)
            text = f'ID:{marker_id} Area:{area:.0f}px'

            if self.effective_camera_matrix is not None:
                zero_dist = np.zeros((1, 5), dtype=np.float64)
                success, rvec, tvec = cv2.solvePnP(
                    self.object_points, points,
                    self.effective_camera_matrix, zero_dist,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if success and float(tvec.reshape(-1)[2]) > 0.0:
                    distance = pnp_distance_m(tvec)
                    text += f' Dist:{distance:.2f}m'
                    cv2.drawFrameAxes(
                        frame, self.effective_camera_matrix, zero_dist,
                        rvec, tvec, self.marker_size)

            text_pos = (
                int(points[0][0]), max(16, int(points[0][1]) - 10))
            cv2.putText(
                frame, text, text_pos,
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

    def destroy_node(self):
        self._stop_event.set()
        with self._frame_condition:
            self._frame_condition.notify_all()
        with self._jpeg_condition:
            self._jpeg_condition.notify_all()
        with self._map_condition:
            self._map_condition.notify_all()
        try:
            self._web_server.shutdown()
        except Exception:
            pass
        if self._worker_thread.is_alive():
            self._worker_thread.join(timeout=2.0)
        if self._web_thread.is_alive():
            self._web_thread.join(timeout=2.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JetsonVisionWebNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
