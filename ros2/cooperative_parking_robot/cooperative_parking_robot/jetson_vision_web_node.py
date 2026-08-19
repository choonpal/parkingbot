#!/usr/bin/env python3
"""Jetson MJPEG monitor plus the touchscreen operator UI.

This node stays a *view*. It renders status and forwards two operator
intents (``/ui/mission_request``, ``/emergency_stop``); it never decides
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
  ``/api/estop``   비상정지
"""

from __future__ import annotations

import json
import os
import queue
import threading
import time
import uuid
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, QoSProfile, ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

from cooperative_parking_robot.aruco_utils import ArucoDetectorCompat
from cooperative_parking_robot.camera_calibration import (
    load_camera_calibration,
    scale_camera_matrix,
)
from cooperative_parking_robot.parking_registry import (
    normalize_vehicle_number,
    validate_parking_password,
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

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None


# 1024x600 Waveshare 7" Display-C 고정 레이아웃.
# 시연장에 인터넷이 없을 수 있으므로 외부 CDN을 쓰지 않는다.
KIOSK_PAGE = """<!DOCTYPE html>
<html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,
 user-scalable=no,viewport-fit=cover">
<title>협동 주차 로봇</title><style>
*{box-sizing:border-box;margin:0;padding:0;-webkit-tap-highlight-color:transparent}
:root{--touch-target:44px}
html,body{width:100%;height:100%}
body{min-width:800px;min-height:480px;overflow:hidden;background:#141a21;
 color:#eef2f6;font-family:"Noto Sans KR","Malgun Gothic",sans-serif;display:flex}
#video{flex:0 0 56%;height:100%;background:#000;display:flex;align-items:center;
 justify-content:center}
#video img{max-width:100%;max-height:100%}
#panel{flex:1;min-width:0;height:100%;padding:8px;display:flex;
 flex-direction:column;gap:5px}
#banner{background:#1e2a36;border-radius:10px;padding:10px;font-size:18px;
 font-weight:700;text-align:center;min-height:52px;display:flex;
 align-items:center;justify-content:center}
#banner.alert{background:#7f1d1d}
#banner.warn{background:#78350f}
.robots{display:flex;gap:10px}
.robot{flex:1;background:#1e2a36;border-radius:10px;padding:7px}
.robot h3{font-size:14px;color:#94a3b8;margin-bottom:4px}
.robot .st{font-size:17px;font-weight:700}
.robot .ph{font-size:13px;color:#94a3b8;margin-top:2px}
.robot.fault{background:#7f1d1d}
.robot.stale{opacity:.45}
button{width:100%;border:0;border-radius:10px;color:#fff;font-size:19px;
 font-weight:800;font-family:inherit;min-height:var(--touch-target);
 touch-action:manipulation}
.mission{background:#1e2a36;border-radius:10px;padding:7px}
.mission h3{font-size:14px;color:#cbd5e1;margin-bottom:5px}
.fields{display:grid;gap:5px;margin-bottom:5px}
.fields.park{grid-template-columns:1.15fr .85fr .72fr}
.fields.retrieve{grid-template-columns:1.2fr .8fr}
input,select{width:100%;height:var(--touch-target);
 min-height:var(--touch-target);border:1px solid #475569;border-radius:7px;
 background:#0f172a;color:#f8fafc;padding:0 7px;font:14px inherit;min-width:0}
input::placeholder{color:#94a3b8}
#park,#retrieve{height:var(--touch-target)}
#park{background:#15803d}#retrieve{background:#0f766e}
#park:disabled,#retrieve:disabled{background:#334155;color:#64748b}
#slots{min-height:52px;max-height:82px;flex:1;overflow-y:auto;display:grid;
 grid-template-columns:1fr 1fr;gap:5px}
.slot{min-height:32px;background:#263646;font-size:13px;border-radius:7px;
 display:flex;align-items:center;justify-content:center;color:#cbd5e1}
.slot.empty{background:#14532d}.slot.occupied{background:#164e63}
#estop{height:54px;background:#b91c1c;font-size:22px;flex:0 0 auto}
#hint{font-size:12px;color:#94a3b8;text-align:center}
#toast{position:fixed;left:50%;top:24px;transform:translateX(-50%);
 background:#0f172a;border:2px solid #38bdf8;padding:12px 22px;border-radius:10px;
 font-size:18px;display:none}
#offline{position:fixed;inset:0;background:rgba(0,0,0,.86);display:none;
 align-items:center;justify-content:center;font-size:30px;font-weight:800}
@media (max-width:900px){
 #video{flex-basis:45%}
 #panel{padding:6px;gap:4px}
 .fields{gap:4px}
}
@media (max-height:520px){
 #banner{min-height:40px;padding:4px;font-size:16px}
 .robot{padding:4px}.robot h3{margin-bottom:2px}
 .robot .st{font-size:16px}.robot .ph{font-size:12px}
 .mission{padding:4px}.mission h3{margin-bottom:2px}
 .fields{margin-bottom:3px}
 input,select{height:var(--touch-target);min-height:var(--touch-target)}
 #park,#retrieve{height:var(--touch-target);min-height:var(--touch-target)}
 #slots{min-height:36px;max-height:44px}
 .slot{min-height:22px}
 #estop{height:44px;min-height:44px;font-size:20px}
 #hint{display:none}
}
</style></head><body>
<div id="video"><img src="/video_feed" alt="CCTV"></div>
<div id="panel">
  <div id="banner">연결 중…</div>
  <div class="robots">
    <div class="robot" id="rf"><h3>FRONT</h3>
      <div class="st">-</div><div class="ph">-</div></div>
    <div class="robot" id="rr"><h3>REAR</h3>
      <div class="st">-</div><div class="ph">-</div></div>
  </div>
  <div class="mission"><h3>입차 등록</h3>
    <div class="fields park">
      <input id="parkVehicle" maxlength="32" autocomplete="off" placeholder="차량번호">
      <input id="parkPassword" type="password" minlength="4" maxlength="64"
        autocomplete="new-password" placeholder="비밀번호">
      <select id="parkSlot" aria-label="주차 슬롯"></select>
    </div>
    <button id="park" disabled>입차 요청</button>
  </div>
  <div class="mission"><h3>출차 인증</h3>
    <div class="fields retrieve">
      <input id="retrieveVehicle" maxlength="32" autocomplete="off" placeholder="차량번호">
      <input id="retrievePassword" type="password" minlength="4" maxlength="64"
        autocomplete="current-password" placeholder="비밀번호">
    </div>
    <button id="retrieve" disabled>출차 요청</button>
  </div>
  <div id="slots"></div>
  <button id="estop">비 상 정 지</button>
  <div id="hint">물리 비상정지 스위치가 우선입니다</div>
</div>
<div id="toast"></div><div id="offline">UI 서버 연결 끊김</div>
<script>
var park=document.getElementById('park'),retrieve=document.getElementById('retrieve'),
    parkVehicle=document.getElementById('parkVehicle'),
    parkPassword=document.getElementById('parkPassword'),
    parkSlot=document.getElementById('parkSlot'),
    retrieveVehicle=document.getElementById('retrieveVehicle'),
    retrievePassword=document.getElementById('retrievePassword'),
    banner=document.getElementById('banner'),
    slotsEl=document.getElementById('slots'),toastEl=document.getElementById('toast'),
    offline=document.getElementById('offline'),lastCompletion=null,lastRequest=null,
    pendingRequest='',latestStatus=null;
var reasons={INVALID_REQUEST:'잘못된 요청',MISSION_ALREADY_ACTIVE:'미션 진행 중',
  INVALID_VEHICLE_NUMBER:'차량번호 형식 오류',INVALID_PASSWORD:'비밀번호 형식 오류',
  VEHICLE_ALREADY_PARKED:'이미 입차된 차량번호',
  VEHICLE_OR_PASSWORD_INVALID:'차량번호 또는 비밀번호 불일치',
  DESTINATION_SLOT_NOT_FOUND:'주차면 없음',DESTINATION_SLOT_NOT_EMPTY:'주차면 사용 중',
  DESTINATION_SLOT_UNAVAILABLE:'주차면을 현재 사용할 수 없음',
  SOURCE_SLOT_NOT_FOUND:'슬롯 없음',SOURCE_SLOT_NOT_OCCUPIED:'차량 없는 슬롯',
  UNSUPPORTED_PARKING_DIRECTION:'지원하지 않는 주차 방향',
  MISSING_VEHICLE_RECORD:'차량 기록 없음',APPROACH_CORRIDOR_BLOCKED:'접근 경로 막힘',
  ROBOT_NOT_IDLE:'로봇 복귀 대기',STALE_REQUEST:'오래된 요청',
  DUPLICATE_REQUEST_ID:'중복 요청',DUPLICATE_SEQUENCE:'중복 순번'};
function toast(m){toastEl.textContent=m;toastEl.style.display='block';
  setTimeout(function(){toastEl.style.display='none'},2500);}
function robot(id,d){var e=document.getElementById(id);
  e.querySelector('.st').textContent=d.state;
  e.querySelector('.ph').textContent=d.phase;
  e.className='robot'+(d.state==='FAULT'?' fault':'')+(d.fresh?'':' stale');}
function validVehicle(v){return v.replace(/\\s/g,'').length>0;}
function updateButtons(){var s=latestStatus||{};
  park.disabled=!(s.park_enabled&&validVehicle(parkVehicle.value)&&
    parkPassword.value.length>=4&&parkSlot.value);
  retrieve.disabled=!(s.retrieve_enabled&&validVehicle(retrieveVehicle.value)&&
    retrievePassword.value.length>=4);}
function renderSlots(slots){var previous=parkSlot.value;
  slotsEl.textContent='';parkSlot.textContent='';
  slots.forEach(function(s){var d=document.createElement('div');
    d.className='slot '+(s.lifecycle==='EMPTY'?'empty':'occupied');
    d.textContent=s.slot_id+' · '+s.lifecycle;slotsEl.appendChild(d);
    if(s.lifecycle==='EMPTY'){var o=document.createElement('option');
      o.value=s.slot_id;o.textContent=s.slot_id;parkSlot.appendChild(o);}});
  if(previous&&Array.from(parkSlot.options).some(function(o){return o.value===previous;})){
    parkSlot.value=previous;}updateButtons();}
function render(s){
  latestStatus=s;
  banner.textContent=s.banner;
  banner.className=s.fault?'alert':(s.localization_warning?'warn':'');
  if(s.localization_warning&&!s.fault){
    banner.textContent=s.banner+' / 위치추정 경고';}
  robot('rf',s.front);robot('rr',s.rear);renderSlots(s.parking_slots||[]);
  updateButtons();
  var c=s.last_completed,seq=c?Number(c.completion_sequence):-1;
  if(lastCompletion===null){lastCompletion=seq;}else if(seq>lastCompletion){
    lastCompletion=seq;toast(c.mission_type==='retrieve'?'출차 완료':'입차 완료');}
  var r=s.request_status,key=r?(r.request_id+':'+r.status):'';
  if(lastRequest===null){lastRequest=key;}
  else if(key&&key!==lastRequest&&r.request_id===pendingRequest){
    lastRequest=key;if(r.status==='ACCEPTED'){toast('Fleet 승인');}
    else if(r.status==='REJECTED'){toast('요청 거부: '+(reasons[r.reason]||r.reason));}}}
function poll(){fetch('/api/status').then(function(r){return r.json();})
  .then(function(s){offline.style.display='none';render(s);})
  .catch(function(){offline.style.display='flex';});}
park.onclick=function(){park.disabled=true;
  fetch('/api/park',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({vehicle_number:parkVehicle.value,password:parkPassword.value,
      destination_slot_id:parkSlot.value})}).then(function(r){return r.json();})
  .then(function(r){if(r.submitted){pendingRequest=r.request_id;}
    parkPassword.value='';toast(r.message);render(r.status);})
  .catch(function(){parkPassword.value='';toast('요청 실패');});};
retrieve.onclick=function(){retrieve.disabled=true;
  fetch('/api/retrieve',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({vehicle_number:retrieveVehicle.value,
      password:retrievePassword.value})}).then(function(r){return r.json();})
  .then(function(r){if(r.submitted){pendingRequest=r.request_id;}
    retrievePassword.value='';toast(r.message);render(r.status);})
  .catch(function(){retrievePassword.value='';toast('요청 실패');});};
['input','change'].forEach(function(eventName){
  [parkVehicle,parkPassword,parkSlot,retrieveVehicle,retrievePassword].forEach(
    function(e){e.addEventListener(eventName,updateButtons);});});
// 비상정지에는 확인창을 두지 않는다. 누르는 즉시 나가야 한다.
document.getElementById('estop').onclick=function(){
  fetch('/api/estop',{method:'POST'});toast('비상정지 발행');};
poll();setInterval(poll,500);
</script></body></html>"""


class JetsonVisionWebNode(Node):
    def __init__(self):
        super().__init__('jetson_vision_web_node')

        self.declare_parameter('image_topic', '/cctv/image_rect')
        self.declare_parameter('annotated_topic', '/cctv/debug/annotated')
        self.declare_parameter('enable_yolo', True)
        self.declare_parameter('enable_aruco', True)
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence', 0.4)
        self.declare_parameter('imgsz', 320)
        self.declare_parameter('process_every_n', 3)
        self.declare_parameter('yolo_class_ids', [])
        self.declare_parameter('aruco_dict', 'DICT_4X4_50')
        self.declare_parameter('front_marker_id', 10)
        self.declare_parameter('rear_marker_id', 11)
        self.declare_parameter('marker_size_m', 0.18)
        self.declare_parameter('min_marker_area_px', 1000.0)
        self.declare_parameter('min_marker_area_ratio', 0.0)
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

        if not WEB_DEPS_OK:
            raise RuntimeError(
                'Jetson web monitor dependencies missing: cv2, numpy, '
                'cv_bridge, flask, werkzeug')

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.annotated_topic = str(
            self.get_parameter('annotated_topic').value)
        self.enable_debug_overlay = bool(
            self.get_parameter('enable_debug_overlay').value)
        requested_enable_yolo = bool(
            self.get_parameter('enable_yolo').value)
        requested_enable_aruco = bool(
            self.get_parameter('enable_aruco').value)
        self.enable_yolo = (
            self.enable_debug_overlay and requested_enable_yolo)
        self.enable_aruco = (
            self.enable_debug_overlay and requested_enable_aruco)
        if self.enable_yolo and YOLO is None:
            raise RuntimeError(
                'debug overlay requires ultralytics when YOLO is enabled')
        self.confidence = float(self.get_parameter('confidence').value)
        self.imgsz = int(self.get_parameter('imgsz').value)
        self.process_every_n = int(
            self.get_parameter('process_every_n').value)
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
        if self.status_stale_s <= 0.0 or self.ui_button_cooldown < 0.0:
            raise ValueError('invalid UI status/cooldown parameters')
        if self.localization_warning_streak <= 0:
            raise ValueError('localization_warning_streak must be positive')

        if not self.image_topic or not self.annotated_topic:
            raise ValueError('image and annotated topics must not be empty')
        if not (0.0 < self.confidence <= 1.0):
            raise ValueError('confidence must be in (0,1]')
        if self.imgsz <= 0 or self.process_every_n <= 0:
            raise ValueError('imgsz and process_every_n must be positive')
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
        yolo_ids_raw = list(self.get_parameter('yolo_class_ids').value)
        self.yolo_class_ids = (
            parse_class_ids(yolo_ids_raw) if yolo_ids_raw else tuple())

        self.bridge = CvBridge()
        self.model = None
        if self.enable_yolo:
            model_path = os.path.expanduser(
                str(self.get_parameter('model_path').value))
            self.model = YOLO(model_path)
            self.get_logger().info(f'debug YOLO loaded: {model_path}')

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
                Image, self.annotated_topic, qos_profile_sensor_data)
        self.create_subscription(
            Image, self.image_topic, self.image_cb, qos_profile_sensor_data)

        self._frame_condition = threading.Condition()
        self._latest_frame = None
        self._latest_header = None
        self._input_sequence = 0
        self._processed_sequence = 0
        self._jpeg_condition = threading.Condition()
        self._latest_jpeg = None
        self._jpeg_sequence = 0
        self._stop_event = threading.Event()
        self._last_boxes: List[Tuple[int, int, int, int, int, float]] = []
        self._last_process_time = None

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
            f'Jetson vision web monitor: {self.image_topic} -> '
            f'{self.annotated_topic}, every_n={self.process_every_n}')

    # ==================================================
    # 터치 UI — 상태 수집
    # ==================================================
    def _setup_operator_ui(self) -> None:
        """상태 구독과 두 개의 운용 발행자를 만든다.

        모든 콜백은 값과 수신 시각만 저장한다. 표시 판단은 /api/status에서
        하는데, 그래야 staleness 기준을 한 곳에서 일관되게 적용할 수 있다.
        """
        def store(key):
            def callback(msg, key=key):
                with self._status_lock:
                    self._status[key] = (msg.data, time.monotonic())
            return callback

        self.create_subscription(String, '/fleet/state', store('fleet'), 10)
        self.create_subscription(Bool, '/parking/target_ready',
                                 store('target_ready'), 10)
        self.create_subscription(String, '/sync/error_state',
                                 store('sync_error'), 10)
        for role in ('front', 'rear'):
            self.create_subscription(
                String, f'/{role}/robot_state', store(f'{role}_state'), 10)
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
        self.pub_estop = self.create_publisher(Bool, '/emergency_stop', 10)
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
            elif kind == 'estop':
                self.pub_estop.publish(Bool(data=True))
                self.get_logger().error('UI 비상정지 발행')

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

        idle = all(robots[role]['state'] == 'IDLE'
                   for role in ('front', 'rear'))
        common_fresh = (
            fleet_fresh and
            all(robots[role]['fresh'] for role in ('front', 'rear')))
        all_fresh = common_fresh and target_fresh

        park_enabled = bool(
            target_ready and fleet_state == 'WAIT_TARGET' and
            empty_count >= 1 and idle and fault is None and all_fresh)
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

        if fault is not None:
            banner = f"오류: {fault['source']} — {fault['reason']}"
        elif not common_fresh:
            banner = '일부 노드와 통신이 끊겼습니다'
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
            banner = '대기공간에 차량을 x축 방향으로 세워 주세요'
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
            'park_enabled': park_enabled,
            'retrieve_enabled': retrieve_enabled,
            'parking_slots': parking_slots,
            'request_status': fleet.get('request_status'),
            'last_completed': fleet.get('last_completed'),
            'fault': fault,
            'localization_warning': localization_warning,
            'banner': banner,
        }

    def request_park(
            self, vehicle_number, password, destination_slot_id):
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
        selected = next((
            slot for slot in status['parking_slots']
            if slot['slot_id'] == destination_slot_id and
            slot['lifecycle'] == 'EMPTY'), None)
        if selected is None:
            return False, '선택한 주차면을 사용할 수 없습니다', status, ''
        now = time.monotonic()
        if now - self._last_park_publish < self.ui_button_cooldown:
            return False, '요청 처리 중입니다', status, ''
        self._ui_sequence += 1
        payload = json.dumps({
            'type': 'park',
            'vehicle_number': vehicle_number,
            'password': password,
            'destination_slot_id': destination_slot_id,
            'request_id': f'ui-{uuid.uuid4()}',
            'client_id': self._ui_client_id,
            'sequence': self._ui_sequence,
            'stamp_ns': self.get_clock().now().nanoseconds,
        })
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

    def request_estop(self) -> bool:
        try:
            self._ui_queue.put_nowait(('estop', ''))
        except queue.Full:
            return False
        return True

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
                '<h2>ROS2 CCTV YOLO + ArUco</h2>'
                '<img src="/video_feed" style="border-radius:10px;max-width:100%">'
                '</body></html>')

        @app.route('/video_feed')
        def video_feed():
            return Response(
                self._mjpeg_stream(),
                mimetype='multipart/x-mixed-replace; boundary=frame')

        @app.route('/kiosk')
        def kiosk():
            if not self.enable_operator_ui:
                return ('operator UI disabled '
                        '(enable_operator_ui:=true)', 404)
            return Response(KIOSK_PAGE, mimetype='text/html')

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

        @app.route('/api/estop', methods=['POST'])
        def api_estop():
            if not self.enable_operator_ui:
                return jsonify({'error': 'operator UI disabled'}), 404
            return jsonify({'accepted': self.request_estop()})

        @app.route('/health')
        def health():
            with self._jpeg_condition:
                ready = self._latest_jpeg is not None
                sequence = self._jpeg_sequence
            return jsonify({
                'ready': ready,
                'sequence': sequence,
                'image_topic': self.image_topic,
                'debug_overlay': self.enable_debug_overlay,
                'yolo': self.enable_yolo,
                'aruco': self.enable_aruco,
            })

        return app

    def _mjpeg_stream(self):
        last_sequence = -1
        while not self._stop_event.is_set():
            with self._jpeg_condition:
                self._jpeg_condition.wait_for(
                    lambda: self._stop_event.is_set()
                    or (self._latest_jpeg is not None
                        and self._jpeg_sequence != last_sequence),
                    timeout=1.0,
                )
                if self._stop_event.is_set():
                    break
                jpeg = self._latest_jpeg
                last_sequence = self._jpeg_sequence
            yield (
                b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n'
                + jpeg + b'\r\n')

    def image_cb(self, message: Image) -> None:
        frame = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        with self._frame_condition:
            self._latest_frame = frame.copy()
            self._latest_header = message.header
            self._input_sequence += 1
            self._frame_condition.notify()

    def _worker_loop(self) -> None:
        frame_number = 0
        while not self._stop_event.is_set():
            with self._frame_condition:
                self._frame_condition.wait_for(
                    lambda: self._stop_event.is_set()
                    or (self._latest_frame is not None
                        and self._input_sequence != self._processed_sequence),
                    timeout=1.0,
                )
                if self._stop_event.is_set():
                    return
                frame = self._latest_frame.copy()
                header = self._latest_header
                self._processed_sequence = self._input_sequence

            frame_number += 1
            now = time.monotonic()
            fps = 0.0
            if self._last_process_time is not None:
                delta = now - self._last_process_time
                if delta > 1e-6:
                    fps = 1.0 / delta
            self._last_process_time = now

            display = frame.copy()
            if self.enable_debug_overlay:
                if (self.enable_yolo and
                        frame_number % self.process_every_n == 0):
                    self._last_boxes = self._run_yolo(frame)
                self._draw_yolo(display, self._last_boxes)
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

    def _run_yolo(self, frame):
        results = self.model(
            frame, imgsz=self.imgsz, conf=self.confidence, verbose=False)
        boxes = []
        for result in results:
            if result.boxes is None:
                continue
            for box in result.boxes:
                class_id = int(box.cls[0].item())
                if self.yolo_class_ids and class_id not in self.yolo_class_ids:
                    continue
                x1, y1, x2, y2 = [
                    int(value) for value in box.xyxy[0].tolist()]
                confidence = float(box.conf[0].item())
                boxes.append((x1, y1, x2, y2, class_id, confidence))
        return boxes

    def _draw_yolo(self, frame, boxes) -> None:
        if self.model is None:
            return
        names = getattr(self.model, 'names', {})
        for x1, y1, x2, y2, class_id, confidence in boxes:
            if isinstance(names, dict):
                label_name = names.get(class_id, str(class_id))
            elif 0 <= class_id < len(names):
                label_name = names[class_id]
            else:
                label_name = str(class_id)
            label = f'{label_name} {confidence:.2f}'
            color = (0, 255, 255)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(
                frame, label, (x1, max(16, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

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
    rclpy.shutdown()


if __name__ == '__main__':
    main()
