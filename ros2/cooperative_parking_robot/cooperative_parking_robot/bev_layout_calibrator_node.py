#!/usr/bin/env python3
"""브라우저에서 Homography와 주차면을 등록하는 ROS 2 노드.

이 노드는 Jetson Ubuntu Server처럼 ``cv2.imshow()``를 쓸 수 없는 환경을 위해
Flask 웹 화면을 제공한다. 카메라를 다시 열지 않고 ``/cctv/image_rect`` 한 장을
정지시킨 뒤 그 **같은 원본 해상도** 위에서 클릭 좌표를 받는다.

사용 순서
---------
1. 바닥 기준점 4개 이상을 클릭하고 각 점의 실측 map 좌표(X,Y)m를 입력한다.
2. Homography를 계산하고 BEV 미리보기/재투영 오차를 확인한다.
3. 주차면마다 네 모서리와 통로 쪽 한 점을 클릭한다.
4. 대기영역 네 모서리를 클릭하고 저장한다.

저장되는 Homography는 픽셀 -> metre 변환이므로 주행 시
``homography_scale_to_m:=1.0``을 사용해야 한다.

천장 카메라 2대 등록 절차 (v1.11)
---------------------------------
카메라마다 이 노드를 한 번씩 돌린다. **핵심은 두 번 모두 같은 바닥 점에
같은 실측 (X,Y)m를 입력하는 것**이다. 그러면 H0와 H2가 같은 map frame으로
투영되므로 카메라 간 변환 행렬이 필요 없다.

  1회차 (cam0): image_topic:=/cctv0/image_rect,
                homography_output_file:=~/.ros/adaptive_valet_bot/homography_cam0_rectified.npy
                → 기준점 + cam0에 보이는 주차면 + 대기영역 등록 후 저장
  2회차 (cam2): image_topic:=/cctv2/image_rect,
                homography_output_file:=~/.ros/adaptive_valet_bot/homography_cam2_rectified.npy,
                append_existing_layout:=true
                → 같은 바닥 점을 같은 실측값으로 다시 등록하고,
                  cam2에서만 보이는 주차면을 추가로 등록 후 저장

``append_existing_layout:=true``면 저장 시 기존 parking_layout.yaml을 읽어
슬롯을 합치고, 대기영역을 새로 등록하지 않았다면 기존 값을 유지한다.
겹침 영역의 기준점은 최소 2~3개를 공유하도록 찍어야 두 H의 정합이 확인된다.
"""

from __future__ import annotations

from dataclasses import asdict
import json
import math
from pathlib import Path
import threading
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from cooperative_parking_robot.bev_layout_core import (
    homography_reprojection_errors,
    load_layout_yaml,
    merge_layout_registrations,
    render_parking_layout_yaml,
    transform_points,
    validate_reference_pairs,
    write_text_atomic,
)
from cooperative_parking_robot.parking_geometry import (
    ParkingSlot,
    slot_from_corners,
)

try:
    import cv2
    import numpy as np
    from cv_bridge import CvBridge
    from flask import Flask, Response, jsonify, request
    from werkzeug.serving import make_server
    DEPS_OK = True
except ImportError:
    DEPS_OK = False


# UI를 Python 안에 넣어 ament_python 설치 후에도 템플릿 경로가 깨지지 않게 한다.
_HTML = r'''<!doctype html>
<html lang="ko">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Adaptive Valet Bot · BEV 등록</title>
  <style>
    :root { color-scheme: dark; --bg:#10151d; --panel:#18212d; --line:#344358;
      --blue:#48a7ff; --green:#49d17d; --orange:#ffb454; --red:#ff6b6b; }
    * { box-sizing:border-box; } body { margin:0; font-family:system-ui,sans-serif;
      background:var(--bg); color:#edf3fa; }
    header { padding:14px 20px; border-bottom:1px solid var(--line); }
    main { display:grid; grid-template-columns:minmax(300px,380px) 1fr; gap:14px;
      padding:14px; } .panel { background:var(--panel); border:1px solid var(--line);
      border-radius:10px; padding:14px; } h1 { font-size:20px; margin:0; }
    h2 { font-size:15px; margin:18px 0 8px; } p,li,label { font-size:13px; }
    .row { display:flex; gap:7px; margin:7px 0; flex-wrap:wrap; }
    input,button,select { background:#0d141e; color:#edf3fa; border:1px solid #43546c;
      border-radius:6px; padding:8px; } input[type=number] { width:88px; }
    button { cursor:pointer; } button.primary { border-color:var(--blue); }
    button.good { border-color:var(--green); } button.warn { border-color:var(--orange); }
    button.active { background:#234d73; border-color:var(--blue); }
    #canvasWrap { position:relative; width:100%; line-height:0; }
    canvas { width:100%; height:auto; border-radius:8px; border:1px solid var(--line);
      cursor:crosshair; background:#05070a; }
    #preview { max-width:100%; border:1px solid var(--line); border-radius:8px; }
    #status { white-space:pre-wrap; background:#0d141e; padding:10px; min-height:56px;
      border-radius:6px; color:#b9c8da; }
    .tag { display:inline-block; padding:3px 7px; border-radius:12px;
      background:#263448; margin:2px; font-size:12px; }
    .small { color:#aebcd0; font-size:12px; line-height:1.45; }
    table { width:100%; border-collapse:collapse; font-size:12px; }
    td,th { border-bottom:1px solid #2c394c; padding:5px; text-align:right; }
    th:first-child,td:first-child { text-align:left; }
    @media(max-width:900px){ main{grid-template-columns:1fr;} }
  </style>
</head>
<body>
<header><h1>Adaptive Valet Bot · BEV / 주차면 등록</h1></header>
<main>
  <section class="panel">
    <div class="row">
      <button class="primary" onclick="takeSnapshot()">1. 현재 영상 정지</button>
      <button onclick="clearSelection()">선택 점 초기화</button>
    </div>
    <p class="small">반드시 왜곡 보정된 <code>/cctv/image_rect</code> 영상에서 진행합니다.</p>

    <h2>2. 바닥 기준점</h2>
    <div class="row">
      <button id="modeRef" onclick="setMode('reference')">기준점 클릭</button>
      <label>X(m) <input id="worldX" type="number" step="0.01"></label>
      <label>Y(m) <input id="worldY" type="number" step="0.01"></label>
      <button onclick="addReference()">현재 점 등록</button>
    </div>
    <div id="referenceList" class="small">기준점 0개</div>
    <div class="row">
      <button class="good" onclick="computeHomography()">Homography 계산</button>
      <button onclick="clearReferences()">기준점 초기화</button>
    </div>

    <h2>3. 주차면</h2>
    <div class="row">
      <label>ID <input id="slotId" value="P1" style="width:80px"></label>
      <button id="modeSlot" onclick="setMode('slot')">모서리 4개 + 통로점</button>
      <button class="good" onclick="registerSlot()">주차면 등록</button>
    </div>
    <p class="small">모서리 4개는 순서 무관. 마지막 5번째 점은 차량이 들어오는 통로 쪽에 찍습니다.</p>
    <div id="slotList" class="small">등록 슬롯 없음</div>

    <h2>4. 차량 대기영역</h2>
    <div class="row">
      <button id="modeWaiting" onclick="setMode('waiting')">대기영역 모서리 4개</button>
      <button class="good" onclick="registerWaiting()">대기영역 등록</button>
    </div>

    <h2>5. 저장</h2>
    <div class="row">
      <label>맵 폭(m) <input id="mapW" type="number" value="4.40" step="0.01"></label>
      <label>맵 높이(m) <input id="mapH" type="number" value="3.83" step="0.01"></label>
      <label>해상도(m) <input id="mapRes" type="number" value="0.05" step="0.01"></label>
      <label>출차 최종 Yaw(°) <input id="waitingYaw" type="number" value="180" step="1"></label>
    </div>
    <div class="row"><button class="warn" onclick="saveAll()">H + YAML 저장</button></div>
    <div id="status">먼저 현재 영상을 정지하세요.</div>
  </section>

  <section class="panel">
    <h2 style="margin-top:0">정지된 원본 영상</h2>
    <div id="canvasWrap"><canvas id="canvas"></canvas></div>
    <p class="small">브라우저에서 축소되어 보여도 클릭점은 원본 영상 픽셀로 환산됩니다.</p>
    <h2>BEV 미리보기</h2>
    <img id="preview" alt="Homography 계산 후 BEV 미리보기가 표시됩니다">
  </section>
</main>
<script>
const canvas = document.getElementById('canvas');
const ctx = canvas.getContext('2d');
const image = new Image();
let snapshotSeq = 0;
let mode = 'reference';
let pending = null;
let references = [], slotClicks = [], waitingClicks = [];

function status(message, bad = false) {
  const el = document.getElementById('status');
  el.textContent = message;
  el.style.color = bad ? 'var(--red)' : '#b9c8da';
}
async function api(url, options = {}) {
  const response = await fetch(url, options);
  const body = await response.json().catch(() => ({error: '응답 해석 실패'}));
  if (!response.ok) throw new Error(body.error || response.statusText);
  return body;
}
function post(url, payload) {
  return api(url, {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify(payload)
  });
}
function setMode(next) {
  mode = next; pending = null; slotClicks = []; waitingClicks = [];
  for (const id of ['modeRef', 'modeSlot', 'modeWaiting']) {
    document.getElementById(id).classList.remove('active');
  }
  const modeId = next === 'reference' ? 'modeRef' :
    next === 'slot' ? 'modeSlot' : 'modeWaiting';
  document.getElementById(modeId).classList.add('active');
  draw();
  const prompt = next === 'reference' ? '영상의 바닥 기준점을 클릭하세요.' :
    next === 'slot' ? '주차면 모서리 4개와 마지막 통로점 1개를 클릭하세요.' :
    '대기영역 모서리 4개를 클릭하세요.';
  status(prompt);
}

async function takeSnapshot() {
  try {
    const snapshot = await post('/api/snapshot', {});
    snapshotSeq = snapshot.sequence;
    image.onload = () => {
      canvas.width = image.naturalWidth;
      canvas.height = image.naturalHeight;
      draw();
    };
    image.src = '/api/snapshot.jpg?sequence=' + snapshotSeq + '&t=' + Date.now();
    references = []; slotClicks = []; waitingClicks = []; pending = null;
    renderReferences();
    status(`영상 정지 완료: ${snapshot.width}×${snapshot.height}px`);
  } catch (error) { status(error.message, true); }
}
canvas.addEventListener('click', event => {
  if (!image.complete || !image.naturalWidth) return;
  const rect = canvas.getBoundingClientRect();
  // CSS 표시 좌표가 아니라 canvas 원본 픽셀 좌표로 되돌린다.
  const point = [
    (event.clientX - rect.left) * canvas.width / rect.width,
    (event.clientY - rect.top) * canvas.height / rect.height
  ];
  if (mode === 'reference') pending = point;
  else if (mode === 'slot' && slotClicks.length < 5) slotClicks.push(point);
  else if (mode === 'waiting' && waitingClicks.length < 4) {
    waitingClicks.push(point);
  }
  draw();
});
function draw() {
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  if (image.complete && image.naturalWidth) {
    ctx.drawImage(image, 0, 0, canvas.width, canvas.height);
  }
  const marks = [];
  references.forEach((reference, index) =>
    marks.push([reference.pixel, `R${index + 1}`, '#48a7ff']));
  if (pending) marks.push([pending, 'R?', '#48a7ff']);
  slotClicks.forEach((point, index) =>
    marks.push([point, index < 4 ? `S${index + 1}` : '통로', '#49d17d']));
  waitingClicks.forEach((point, index) =>
    marks.push([point, `W${index + 1}`, '#ffb454']));
  ctx.font = '16px system-ui'; ctx.lineWidth = 3;
  for (const [point, label, color] of marks) {
    ctx.beginPath(); ctx.arc(point[0], point[1], 7, 0, Math.PI * 2);
    ctx.fillStyle = color; ctx.fill();
    ctx.fillText(label, point[0] + 10, point[1] - 8);
  }
}
function addReference() {
  if (!pending) { status('먼저 영상의 기준점을 클릭하세요.', true); return; }
  const rawX = document.getElementById('worldX').value.trim();
  const rawY = document.getElementById('worldY').value.trim();
  if (rawX === '' || rawY === '') {
    status('실측 X,Y metre 값을 둘 다 입력하세요.', true); return;
  }
  const x = Number(rawX), y = Number(rawY);
  if (!Number.isFinite(x) || !Number.isFinite(y)) {
    status('실측 X,Y metre 값을 입력하세요.', true); return;
  }
  references.push({pixel: pending, world: [x, y]});
  pending = null; renderReferences(); draw();
}
function renderReferences() {
  document.getElementById('referenceList').innerHTML = references.length ?
    references.map((reference, index) =>
      `<span class="tag">R${index + 1}: ` +
      `(${reference.pixel[0].toFixed(1)},${reference.pixel[1].toFixed(1)}) → ` +
      `(${reference.world[0]},${reference.world[1]})m</span>`).join('') :
    '기준점 0개';
}
function clearReferences() {
  references = []; pending = null; renderReferences(); draw();
}
function clearSelection() {
  pending = null; slotClicks = []; waitingClicks = []; draw();
  status('현재 선택 점을 초기화했습니다.');
}
async function computeHomography() {
  try {
    const output = await post('/api/homography', {references});
    status(`Homography 계산 완료\nRMS ${output.rms_m.toFixed(4)}m / ` +
      `최대 ${output.max_error_m.toFixed(4)}m`);
    refreshPreview();
  } catch (error) { status(error.message, true); }
}
async function registerSlot() {
  try {
    if (slotClicks.length !== 5) {
      throw new Error('모서리 4개와 통로점 1개가 필요합니다.');
    }
    const id = document.getElementById('slotId').value.trim();
    if (!id) throw new Error('슬롯 ID를 입력하세요.');
    const output = await post('/api/slot', {
      slot_id: id,
      pixel_corners: slotClicks.slice(0, 4),
      aisle_pixel: slotClicks[4]
    });
    slotClicks = []; draw(); await loadState(); refreshPreview();
    status(`${output.slot.slot_id} 등록: ` +
      `${output.slot.length_m.toFixed(2)}×${output.slot.width_m.toFixed(2)}m, ` +
      `yaw ${output.entry_yaw_deg.toFixed(1)}°`);
  } catch (error) { status(error.message, true); }
}
async function registerWaiting() {
  try {
    if (waitingClicks.length !== 4) {
      throw new Error('대기영역 모서리 4개가 필요합니다.');
    }
    await post('/api/waiting', {pixel_corners: waitingClicks});
    waitingClicks = []; draw(); refreshPreview();
    status('대기영역 등록 완료');
  } catch (error) { status(error.message, true); }
}
async function loadState() {
  try {
    const state = await api('/api/state');
    document.getElementById('slotList').innerHTML = state.slots.length ?
      state.slots.map(value =>
        `<span class="tag">${value.slot_id} ` +
        `${value.length_m.toFixed(2)}×${value.width_m.toFixed(2)}m / ` +
        `${value.entry_yaw_deg.toFixed(1)}°</span>`).join('') :
      '등록 슬롯 없음';
  } catch (error) { status(error.message, true); }
}
function refreshPreview() {
  document.getElementById('preview').src =
    '/api/preview.jpg?sequence=' + snapshotSeq + '&t=' + Date.now();
}
async function saveAll() {
  try {
    const output = await post('/api/save', {
      map_width_m: Number(document.getElementById('mapW').value),
      map_height_m: Number(document.getElementById('mapH').value),
      map_resolution_m: Number(document.getElementById('mapRes').value),
      waiting_yaw_deg: Number(document.getElementById('waitingYaw').value)
    });
    status(`저장 완료\nHomography: ${output.homography_file}\n` +
      `Layout: ${output.layout_file}\n주행 시 homography_scale_to_m:=1.0`);
  } catch (error) { status(error.message, true); }
}
setMode('reference');
loadState();
</script>
</body></html>'''


class BevLayoutCalibratorNode(Node):
    """정지 영상과 클릭 API를 제공하고 H/YAML을 명시된 경로에 저장한다."""

    def __init__(self):
        super().__init__('bev_layout_calibrator_node')
        self.declare_parameter('image_topic', '/cctv/image_rect')
        self.declare_parameter(
            'homography_output_file',
            '~/.ros/adaptive_valet_bot/homography_rectified.npy')
        self.declare_parameter(
            'layout_output_file',
            '~/.ros/adaptive_valet_bot/parking_layout.yaml')
        self.declare_parameter('web_host', '0.0.0.0')
        self.declare_parameter('web_port', 5001)
        self.declare_parameter('jpeg_quality', 88)
        self.declare_parameter('preview_pixels_per_m', 120)
        self.declare_parameter('default_map_width_m', 4.40)
        self.declare_parameter('default_map_height_m', 3.83)
        # --- v1.11 천장 카메라 2대 등록 ---
        # 화면 제목과 로그에 표시할 카메라 이름(어느 카메라를 등록 중인지 혼동 방지)
        self.declare_parameter('camera_label', 'cam0')
        # true면 저장 시 기존 parking_layout.yaml의 슬롯을 읽어 합친다.
        self.declare_parameter('append_existing_layout', False)

        if not DEPS_OK:
            raise RuntimeError(
                'BEV calibration dependencies missing: cv2, numpy, cv_bridge, '
                'flask, werkzeug')

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.homography_path = Path(str(
            self.get_parameter('homography_output_file').value)).expanduser()
        self.layout_path = Path(str(
            self.get_parameter('layout_output_file').value)).expanduser()
        self.web_host = str(self.get_parameter('web_host').value)
        self.web_port = int(self.get_parameter('web_port').value)
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self.preview_ppm = int(
            self.get_parameter('preview_pixels_per_m').value)
        self.map_width_m = float(
            self.get_parameter('default_map_width_m').value)
        self.map_height_m = float(
            self.get_parameter('default_map_height_m').value)
        self.camera_label = str(
            self.get_parameter('camera_label').value).strip() or 'cam0'
        self.append_existing_layout = bool(
            self.get_parameter('append_existing_layout').value)
        if not self.image_topic:
            raise ValueError('image_topic must not be empty')
        if not 1 <= self.web_port <= 65535:
            raise ValueError('web_port must be in [1,65535]')
        if not 1 <= self.jpeg_quality <= 100 or self.preview_ppm <= 0:
            raise ValueError('invalid jpeg_quality/preview_pixels_per_m')

        self.bridge = CvBridge()
        self._lock = threading.RLock()
        self._latest_frame = None
        self._snapshot = None
        self._snapshot_sequence = 0
        self._homography = None
        self._references: List[Dict] = []
        self._slots: Dict[str, ParkingSlot] = {}
        self._slot_metadata: Dict[str, Dict] = {}
        self._waiting_world: Optional[List[List[float]]] = None

        self.create_subscription(
            Image, self.image_topic, self.image_cb, qos_profile_sensor_data)
        self._flask_app = self._make_flask_app()
        self._web_server = make_server(
            self.web_host, self.web_port, self._flask_app, threaded=True)
        self._web_thread = threading.Thread(
            target=self._web_server.serve_forever,
            name='bev-layout-calibration-web', daemon=True)
        self._web_thread.start()
        self.get_logger().warn(
            f'[{self.camera_label}] BEV 등록 화면: '
            f'http://{self.web_host}:{self.web_port}/ '
            '(신뢰 가능한 내부망에서만 사용)')
        self.get_logger().info(
            f'[{self.camera_label}] {self.image_topic} -> '
            f'H={self.homography_path}, layout={self.layout_path} '
            f'(append={self.append_existing_layout})')
        if self.append_existing_layout:
            self.get_logger().warn(
                'append 모드 — 기존 parking_layout.yaml의 슬롯을 유지한 채 '
                '이번에 등록한 슬롯만 추가/갱신합니다. 기준점은 반드시 '
                '1회차와 "같은 바닥 점 / 같은 실측 (X,Y)m"로 찍으세요.')

    def image_cb(self, message: Image):
        frame = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        with self._lock:
            self._latest_frame = frame.copy()

    def _require_snapshot(self):
        with self._lock:
            if self._snapshot is None:
                raise ValueError('먼저 현재 영상을 정지하세요')
            return self._snapshot.copy()

    def _require_homography(self):
        with self._lock:
            if self._homography is None:
                raise ValueError('먼저 Homography를 계산하세요')
            return self._homography.copy()

    @staticmethod
    def _json_error(exc, status=400):
        return jsonify({'error': str(exc)}), status

    def _make_flask_app(self):
        app = Flask('bev_layout_calibrator')

        @app.get('/')
        def index():
            return Response(_HTML, mimetype='text/html; charset=utf-8')

        @app.get('/health')
        def health():
            with self._lock:
                return jsonify({
                    'ready': self._latest_frame is not None,
                    'snapshot': self._snapshot is not None,
                    'homography': self._homography is not None,
                    'slots': len(self._slots),
                    'image_topic': self.image_topic,
                })

        @app.post('/api/snapshot')
        def snapshot():
            with self._lock:
                if self._latest_frame is None:
                    return self._json_error('아직 rectified ROS 영상을 받지 못했습니다', 503)
                self._snapshot = self._latest_frame.copy()
                self._snapshot_sequence += 1
                # 다른 프레임에서 얻은 픽셀 좌표를 섞지 않도록 새 snapshot마다 초기화.
                self._homography = None
                self._references = []
                self._slots = {}
                self._slot_metadata = {}
                self._waiting_world = None
                height, width = self._snapshot.shape[:2]
                return jsonify({
                    'sequence': self._snapshot_sequence,
                    'width': width, 'height': height,
                })

        @app.get('/api/snapshot.jpg')
        def snapshot_jpg():
            try:
                frame = self._require_snapshot()
                return self._jpeg_response(frame)
            except ValueError as exc:
                return self._json_error(exc, 404)

        @app.post('/api/homography')
        def homography():
            try:
                self._require_snapshot()
                payload = request.get_json(force=True) or {}
                raw_references = payload.get('references', [])
                pairs = validate_reference_pairs(raw_references)
                source = np.asarray(
                    [(item[0], item[1]) for item in pairs], dtype=np.float64)
                destination = np.asarray(
                    [(item[2], item[3]) for item in pairs], dtype=np.float64)
                matrix, _mask = cv2.findHomography(source, destination, 0)
                if matrix is None or matrix.shape != (3, 3):
                    raise ValueError('기준점이 공선이거나 배치가 잘못되어 H를 계산할 수 없습니다')
                if not np.all(np.isfinite(matrix)) or abs(np.linalg.det(matrix)) < 1e-12:
                    raise ValueError('Homography가 특이행렬입니다. 기준점을 다시 선택하세요')
                errors, rms, maximum = homography_reprojection_errors(
                    matrix, raw_references)
                with self._lock:
                    self._homography = matrix
                    self._references = list(raw_references)
                    # H를 바꾸면 이전 H에서 계산한 슬롯 metre 좌표는 폐기해야 한다.
                    self._slots = {}
                    self._slot_metadata = {}
                    self._waiting_world = None
                return jsonify({
                    'matrix': matrix.tolist(),
                    'errors_m': errors,
                    'rms_m': rms,
                    'max_error_m': maximum,
                    'unit': 'metre',
                })
            except (TypeError, ValueError) as exc:
                return self._json_error(exc)

        @app.post('/api/slot')
        def slot():
            try:
                payload = request.get_json(force=True) or {}
                slot_id = str(payload.get('slot_id', '')).strip()
                pixel_corners = payload.get('pixel_corners', [])
                aisle_pixel = payload.get('aisle_pixel', [])
                if not slot_id:
                    raise ValueError('slot_id가 비어 있습니다')
                if len(pixel_corners) != 4 or len(aisle_pixel) != 2:
                    raise ValueError('주차면 모서리 4개와 통로점 1개가 필요합니다')
                matrix = self._require_homography()
                world_corners = transform_points(matrix, pixel_corners)
                aisle_world = transform_points(matrix, [aisle_pixel])[0]
                registered = slot_from_corners(
                    slot_id, world_corners, aisle_world)
                metadata = {
                    'pixel_corners': pixel_corners,
                    'world_corners_m': world_corners,
                    'aisle_pixel': aisle_pixel,
                    'aisle_world_m': aisle_world,
                }
                with self._lock:
                    self._slots[slot_id] = registered
                    self._slot_metadata[slot_id] = metadata
                result = asdict(registered)
                return jsonify({
                    'slot': result,
                    'entry_yaw_deg': math.degrees(registered.entry_yaw_rad),
                })
            except (TypeError, ValueError) as exc:
                return self._json_error(exc)

        @app.post('/api/waiting')
        def waiting():
            try:
                payload = request.get_json(force=True) or {}
                pixel_corners = payload.get('pixel_corners', [])
                if len(pixel_corners) != 4:
                    raise ValueError('대기영역 모서리 4개가 필요합니다')
                world = transform_points(
                    self._require_homography(), pixel_corners)
                with self._lock:
                    self._waiting_world = [list(point) for point in world]
                return jsonify({'world_corners_m': world})
            except (TypeError, ValueError) as exc:
                return self._json_error(exc)

        @app.get('/api/state')
        def state():
            with self._lock:
                slots = []
                for registered in self._slots.values():
                    data = asdict(registered)
                    data['entry_yaw_deg'] = math.degrees(
                        registered.entry_yaw_rad)
                    slots.append(data)
                return jsonify({
                    'snapshot_sequence': self._snapshot_sequence,
                    'homography_ready': self._homography is not None,
                    'slots': slots,
                    'waiting_ready': self._waiting_world is not None,
                })

        @app.get('/api/preview.jpg')
        def preview():
            try:
                return self._jpeg_response(self._render_preview())
            except ValueError as exc:
                return self._json_error(exc, 404)

        @app.post('/api/save')
        def save():
            try:
                payload = request.get_json(force=True) or {}
                map_width = float(payload.get('map_width_m', self.map_width_m))
                map_height = float(payload.get('map_height_m', self.map_height_m))
                map_resolution = float(payload.get('map_resolution_m', 0.05))
                waiting_yaw_deg = float(payload.get('waiting_yaw_deg', 0.0))
                with self._lock:
                    matrix = self._require_homography()
                    slots = list(self._slots.values())
                    waiting_world = self._waiting_world
                    references = list(self._references)
                    slot_metadata = dict(self._slot_metadata)
                    snapshot = self._snapshot.copy()
                # append 모드에서는 이 카메라에서 슬롯을 하나도 등록하지
                # 않아도 된다(예: cam2가 주차면을 하나도 못 보는 배치).
                existing_layout = None
                if self.append_existing_layout:
                    existing_layout = load_layout_yaml(str(self.layout_path))
                if not slots and existing_layout is None:
                    raise ValueError('주차면을 한 개 이상 등록하세요')
                if waiting_world is None and existing_layout is None:
                    raise ValueError('차량 대기영역을 등록하세요')

                new_polygons = [
                    slot_metadata[slot.slot_id]['world_corners_m']
                    for slot in slots]
                merged_slots, merged_polygons, merged_waiting = (
                    merge_layout_registrations(
                        existing_layout, slots, new_polygons, waiting_world))
                layout_text = render_parking_layout_yaml(
                    merged_slots, merged_waiting,
                    slot_polygons=merged_polygons,
                    map_width_m=map_width,
                    map_height_m=map_height,
                    map_resolution_m=map_resolution,
                    waiting_yaw_deg=waiting_yaw_deg)
                self._save_npy_atomic(self.homography_path, matrix)
                write_text_atomic(str(self.layout_path), layout_text)
                metadata_path = self.homography_path.with_suffix('.json')
                errors, rms, maximum = homography_reprojection_errors(
                    matrix, references)
                metadata = {
                    'format': 'pixel_to_map_metre_homography_v1',
                    'camera_label': self.camera_label,
                    'appended_to_existing_layout': bool(
                        self.append_existing_layout and
                        existing_layout is not None),
                    'image_topic': self.image_topic,
                    'image_width_px': int(snapshot.shape[1]),
                    'image_height_px': int(snapshot.shape[0]),
                    'homography_scale_to_m': 1.0,
                    'references': references,
                    'reprojection_errors_m': errors,
                    'reprojection_rms_m': rms,
                    'reprojection_max_m': maximum,
                    'slots': slot_metadata,
                    'waiting_polygon_m': waiting_world,
                    'layout_file': str(self.layout_path),
                }
                write_text_atomic(
                    str(metadata_path),
                    json.dumps(metadata, ensure_ascii=False, indent=2) + '\n')
                return jsonify({
                    'homography_file': str(self.homography_path),
                    'metadata_file': str(metadata_path),
                    'layout_file': str(self.layout_path),
                    'homography_scale_to_m': 1.0,
                    'camera_label': self.camera_label,
                    'slot_count': len(merged_slots),
                    'new_slot_count': len(slots),
                })
            except (OSError, TypeError, ValueError) as exc:
                return self._json_error(exc)

        return app

    def _jpeg_response(self, frame):
        ok, encoded = cv2.imencode(
            '.jpg', frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if not ok:
            raise ValueError('JPEG 인코딩 실패')
        return Response(encoded.tobytes(), mimetype='image/jpeg')

    @staticmethod
    def _save_npy_atomic(path: Path, matrix):
        """``np.save``의 자동 확장자 추가를 피하면서 원자적으로 저장한다."""
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_name(path.name + '.tmp')
        with temporary.open('wb') as stream:
            np.save(stream, matrix, allow_pickle=False)
            stream.flush()
        temporary.replace(path)

    def _render_preview(self):
        frame = self._require_snapshot()
        matrix = self._require_homography()
        width = max(1, int(round(self.map_width_m * self.preview_ppm)))
        height = max(1, int(round(self.map_height_m * self.preview_ppm)))
        # ROS map처럼 원점은 좌하단, +Y는 위쪽으로 보이도록 표시 좌표만 뒤집는다.
        metre_to_preview = np.array([
            [self.preview_ppm, 0.0, 0.0],
            [0.0, -self.preview_ppm, height - 1.0],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)
        preview = cv2.warpPerspective(
            frame, metre_to_preview @ matrix, (width, height))

        def to_px(point):
            return (int(round(point[0] * self.preview_ppm)),
                    int(round(height - 1.0 - point[1] * self.preview_ppm)))

        # 1m 격자와 0.5m 보조선을 그려 축척이 맞는지 눈으로 확인하게 한다.
        half_step = max(1, int(round(0.5 * self.preview_ppm)))
        for x in range(0, width, half_step):
            major = (x % max(1, self.preview_ppm)) == 0
            cv2.line(preview, (x, 0), (x, height - 1),
                     (95, 95, 95) if major else (45, 45, 45), 1)
        for y in range(0, height, half_step):
            major = (y % max(1, self.preview_ppm)) == 0
            cv2.line(preview, (0, y), (width - 1, y),
                     (95, 95, 95) if major else (45, 45, 45), 1)

        with self._lock:
            slot_metadata = dict(self._slot_metadata)
            slots = dict(self._slots)
            waiting = None if self._waiting_world is None else list(
                self._waiting_world)
        for slot_id, registered in slots.items():
            corners = slot_metadata[slot_id]['world_corners_m']
            polygon = np.asarray([to_px(point) for point in corners], np.int32)
            hull = cv2.convexHull(polygon)
            cv2.polylines(preview, [hull], True, (80, 230, 100), 2)
            center = to_px(registered.center)
            arrow_end = to_px((
                registered.center_x_m + 0.35 * math.cos(registered.entry_yaw_rad),
                registered.center_y_m + 0.35 * math.sin(registered.entry_yaw_rad)))
            cv2.arrowedLine(preview, center, arrow_end, (80, 230, 100), 2)
            cv2.putText(preview, slot_id, center,
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (80, 230, 100), 1)
        if waiting is not None:
            polygon = np.asarray([to_px(point) for point in waiting], np.int32)
            cv2.polylines(preview, [cv2.convexHull(polygon)], True,
                          (40, 170, 255), 2)
        cv2.putText(preview, 'origin (0,0)', (6, height - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        return preview

    def destroy_node(self):
        try:
            self._web_server.shutdown()
        except Exception:
            pass
        if self._web_thread.is_alive():
            self._web_thread.join(timeout=2.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BevLayoutCalibratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
