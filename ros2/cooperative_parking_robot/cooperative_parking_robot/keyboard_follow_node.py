#!/usr/bin/env python3
"""Keyboard/web control for a bounded ArUco-held virtual rigid robot pair.

One node owns both manual command channels.  It captures the currently
observed ArUco forward/lateral/yaw as the exact reference when armed, commands
Front and Rear as a virtual rigid pair, and stops both on stale telemetry,
graph conflicts, relative-pose deviation, or a bounded session-distance limit.
No gripper command is exposed in this mode.
"""

from __future__ import annotations

from collections import deque
import json
import math
import os
import queue
import select
import sys
import termios
import threading
import time
import tty

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, String

from cooperative_parking_robot.rigid_pair_teleop_core import (
    RigidPairTeleopLimits,
    angle_norm,
    capture_pair_reference,
    evaluate_rigid_pair,
    split_pair_centre_twist,
    is_zero,
    median_relative_pose,
    OdomPathAccumulator,
    relative_pose_is_stable,
    request_origin_is_same_host,
    evaluate_placement_guide,
)
from cooperative_parking_robot.command_qos import CMD_VEL_QOS
from cooperative_parking_robot.freshness import StampGate, stamp_to_ns
from cooperative_parking_robot.manual_control import (
    DEFAULT_LINEAR_SPEED_MPS,
    KeyboardTeleopState,
)
from cooperative_parking_robot.vehicle_entry import DEFAULT_WHEELBASE_M

try:
    from flask import Flask, jsonify, request
    from werkzeug.serving import make_server
    WEB_OK = True
except ImportError:
    WEB_OK = False


_STATE_LABELS = {
    'IDLE': '정지 · 제어권 없음',
    'ARMING': '양쪽 제어권 확인 중',
    'ARMED': '강체 쌍 제어 준비 완료',
    'FAULT': '안전 조건 위반 · 정지 유지',
    'LIMIT': '세션 거리 제한 · 정지 유지',
    'ESTOP': '비상정지 고정',
}


_HTML = r'''<!doctype html><html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>강체 쌍 키보드 제어</title><style>
:root{color-scheme:dark;--bg:#0c1118;--panel:#151e29;--line:#304157;
--text:#eef4fb;--muted:#9fb0c4;--ok:#45d483;--bad:#ff6b70;--warn:#ffba55}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);
font-family:system-ui,sans-serif}header{display:flex;gap:12px;align-items:center;
padding:14px 18px;border-bottom:1px solid var(--line);flex-wrap:wrap}h1{font-size:19px;
margin:0}.pill{padding:6px 10px;border-radius:999px;background:#253246;font-size:13px}
.ok{color:var(--ok)}.bad{color:var(--bad)}.warn{color:var(--warn)}main{display:grid;
grid-template-columns:minmax(420px,1.2fr) minmax(360px,1fr);gap:14px;padding:14px}
@media(max-width:900px){main{grid-template-columns:1fr}}.panel{background:var(--panel);
border:1px solid var(--line);border-radius:12px;padding:14px}.camera{width:100%;display:block;
border:1px solid var(--line);border-radius:8px;background:#05070a}.decision{font-size:17px;
font-weight:700;padding:11px;background:#0d151f;border-radius:8px}.actions{display:grid;
grid-template-columns:1fr 1fr;gap:8px;margin:11px 0}button{border:1px solid #445b78;
border-radius:8px;padding:11px;background:#223047;color:var(--text);font-weight:700;
cursor:pointer}.arm{background:#195d3c}.stop{background:#6b3a20}.estop{background:#7b2028;
grid-column:1/-1}.keys{display:grid;grid-template-columns:repeat(3,66px);gap:7px;
justify-content:center;margin:13px}.key{padding:14px 4px;font-size:17px}.blank{visibility:hidden}
.rows{display:grid;grid-template-columns:1fr 1fr;gap:9px}.card{background:#0e1620;
padding:10px;border-radius:8px}.card h3{font-size:14px;margin:0 0 7px}.row{display:flex;
justify-content:space-between;gap:8px;font-size:13px;padding:3px 0}.value{text-align:right;
font-variant-numeric:tabular-nums}.small{font-size:12px;line-height:1.45;color:var(--muted)}
.blockers{color:var(--warn);font-size:13px;padding-left:20px}</style></head><body>
<header><h1>가상 강체 쌍 키보드 제어</h1><span id="state" class="pill">연결 중…</span>
<span class="small">페이지를 클릭한 뒤 WASD / Q·E / Space</span></header><main>
<section class="panel"><img id="camera" class="camera" alt="Rear ArUco camera">
<div class="keys">
<button class="blank">.</button>
<button class="key" onpointerdown="holdKey('w')" onpointerup="releaseKey('w')"
onpointercancel="releaseKey('w')" onpointerleave="releaseKey('w')">W</button>
<button class="blank">.</button>
<button class="key" onpointerdown="holdKey('a')" onpointerup="releaseKey('a')"
onpointercancel="releaseKey('a')" onpointerleave="releaseKey('a')">A</button>
<button class="key stop" onpointerdown="stopHeld()">■</button>
<button class="key" onpointerdown="holdKey('d')" onpointerup="releaseKey('d')"
onpointercancel="releaseKey('d')" onpointerleave="releaseKey('d')">D</button>
<button class="key" onpointerdown="holdKey('q')" onpointerup="releaseKey('q')"
onpointercancel="releaseKey('q')" onpointerleave="releaseKey('q')">Q</button>
<button class="key" onpointerdown="holdKey('s')" onpointerup="releaseKey('s')"
onpointercancel="releaseKey('s')" onpointerleave="releaseKey('s')">S</button>
<button class="key" onpointerdown="holdKey('e')" onpointerup="releaseKey('e')"
onpointercancel="releaseKey('e')" onpointerleave="releaseKey('e')">E</button>
</div>
<p class="small">W/S 전후 · A/D 횡이동 · Q/E 두 로봇 중점 회전 · Space 정지. 키를
누르는 동안 브라우저 반복 입력이 들어오며, 입력이 0.30초 끊기면 자동 정지합니다.</p></section>
<section class="panel"><div id="decision" class="decision">상태 수신 중…</div>
<div class="actions"><button class="arm" onclick="act('arm')">현재 자세 기준 준비 (후보 무관)</button>
<button class="stop" onclick="act('disarm')">정지·제어권 해제</button>
<button class="estop" onclick="emergency()">양쪽 비상정지</button></div>
<div class="rows"><div class="card"><h3>기준과 현재</h3><div id="pose"></div></div>
<div class="card"><h3>배치 안내 (카메라 후보)</h3><div id="placement"></div>
<p class="small">표시 전용이며 Arm gate가 아니고 물리 정렬을 보증하지 않습니다.</p></div>
<div class="card"><h3>현재 명령</h3><div id="commands"></div></div></div>
<ul id="blockers" class="blockers"></ul><p class="small">그리퍼 키는 이 모드에서
비활성화되어 있습니다. 위험하면 웹보다 물리 E-STOP을 먼저 사용하세요.</p></section></main>
<script>
const entities = {
  '&':'&amp;', '<':'&lt;', '>':'&gt;', '"':'&quot;', "'":'&#39;'
};
const esc = s => String(s ?? '—').replace(/[&<>"']/g, c => entities[c]);
const row = (k, v) =>
  `<div class="row"><span>${esc(k)}</span>` +
  `<span class="value">${v}</span></div>`;
const n = (v, d=1, u='') => v == null ? '—' : Number(v).toFixed(d) + u;
const signed = (v, d=1, u='') => v == null ? '—' :
  (Number(v) >= 0 ? '+' : '') + Number(v).toFixed(d) + u;
let keySending = false;
let pendingKey = null;
async function key(k) {
  // 한 요청이 느릴 때 W 요청 여러 개가 쌓인 뒤 Space보다 늦게 도착하지
  // 않도록, 전송 중에는 가장 최신 키 하나만 남긴다.
  pendingKey = k;
  if (keySending) return;
  keySending = true;
  while (pendingKey !== null) {
    const next = pendingKey;
    pendingKey = null;
    try {
      await fetch('/api/key', {
        method:'POST',
        headers:{'Content-Type':'application/json'},
        body:JSON.stringify({key:next})
      });
    } catch (e) {
      // deadman이 0.30초 뒤 정지한다. 연결 복구 시 최신 pendingKey만 보낸다.
    }
  }
  keySending = false;
}
async function act(a) {
  if (a === 'disarm') stopHeld();
  await fetch('/api/' + a, {method:'POST'});
}
let heldKey = null;
function holdKey(k) {
  if (k === ' ') return stopHeld();
  heldKey = k;
  key(k);
}
function releaseKey(k) {
  if (heldKey === k) stopHeld();
}
function stopHeld() {
  heldKey = null;
  key(' ');
}
setInterval(() => {
  if (heldKey !== null) key(heldKey);
}, 100);
function emergency() {
  stopHeld();
  if (confirm('양쪽 STM32에 고정 비상정지를 전송할까요?')) act('estop');
}
document.addEventListener('keydown', e => {
  const k = e.key.toLowerCase();
  if (['w','a','s','d','q','e',' '].includes(k)) {
    e.preventDefault();
    if (e.repeat) return;
    if (k === ' ') stopHeld(); else holdKey(k);
  }
});
document.addEventListener('keyup', e => {
  const k = e.key.toLowerCase();
  if (['w','a','s','d','q','e'].includes(k)) {
    e.preventDefault();
    releaseKey(k);
  }
});
window.addEventListener('blur', stopHeld);
document.addEventListener('visibilitychange', () => {
  if (document.hidden) stopHeld();
});
let statusInFlight = false;
const STATUS_FETCH_TIMEOUT_MS = 800;
const STATUS_WATCHDOG_MS = 1000;
let statusGeneration = 0;
let lastSuccessfulStatusMs = 0;
let statusWatchdogStartedMs = Date.now();
let disconnectedDisplayed = false;
function renderDisconnected() {
  const st = document.getElementById('state');
  st.textContent = '연결 끊김'; st.className = 'pill bad';
  document.getElementById('decision').textContent =
    '상태 연결이 끊겼습니다. 표시값은 사용할 수 없습니다.';
  document.getElementById('pose').innerHTML = row('상태', '—');
  document.getElementById('placement').innerHTML =
    row('상태', '표시 사용 불가') + row('추정값', '—');
  document.getElementById('commands').innerHTML =
    row('Front x/y/ω', '—') + row('Rear x/y/ω', '—');
  document.getElementById('blockers').innerHTML = '<li>5007 상태 수신 실패</li>';
  disconnectedDisplayed = true;
}
function invalidateAndRenderDisconnected() {
  statusGeneration += 1;
  renderDisconnected();
}
function statusWatchdog() {
  const reference = lastSuccessfulStatusMs || statusWatchdogStartedMs;
  if (Date.now() - reference > STATUS_WATCHDOG_MS && !disconnectedDisplayed) {
    invalidateAndRenderDisconnected();
  }
}
async function tick() {
  if (statusInFlight) return;
  statusInFlight = true;
  const generation = ++statusGeneration;
  const controller = new AbortController();
  const timeout = setTimeout(
    () => controller.abort(), STATUS_FETCH_TIMEOUT_MS);
  let s;
  try {
    const response = await fetch('/api/status', {
      cache:'no-store', signal: controller.signal});
    if (!response.ok) throw new Error('status HTTP ' + response.status);
    s = await response.json();
    if (!s || typeof s.state !== 'string' || !s.placement || !s.pose ||
        !s.commands || !Array.isArray(s.commands.front) ||
        !Array.isArray(s.commands.rear) || !s.distance ||
        !Array.isArray(s.blockers)) {
      throw new Error('invalid status payload');
    }
  } catch (e) {
    if (generation === statusGeneration) invalidateAndRenderDisconnected();
    return;
  } finally {
    clearTimeout(timeout);
    statusInFlight = false;
  }
  if (generation !== statusGeneration) return;
  lastSuccessfulStatusMs = Date.now();
  disconnectedDisplayed = false;
  const st = document.getElementById('state');
  st.textContent = s.state_label;
  st.className = 'pill ' + (s.state === 'ARMED' ? 'ok' :
    s.state === 'FAULT' || s.state === 'ESTOP' ? 'bad' : 'warn');
  document.getElementById('decision').textContent = s.decision;
  const p = s.pose;
  const g = s.placement;
  const c = s.commands;
  document.getElementById('pose').innerHTML =
    row('ID0 raw 기준', n(p.reference_forward_cm, 1, ' cm')) +
    row('ID0 raw 현재', n(p.forward_cm, 1, ' cm')) +
    row('간격 오차', n(p.gap_error_cm, 1, ' cm')) +
    row('좌우 오차', n(p.lateral_error_cm, 1, ' cm')) +
    row('각도 오차(3프레임)', n(p.yaw_error_deg, 1, '°')) +
    row('각도 원시값', n(p.raw_yaw_error_deg, 1, '°'));
  document.getElementById('commands').innerHTML =
    row('키 입력', esc(s.key_intent)) +
    row('Front x/y/ω', c.front.map(v => Number(v).toFixed(3)).join(' / ')) +
    row('Rear x/y/ω', c.rear.map(v => Number(v).toFixed(3)).join(' / ')) +
    row('Front 이동', n(s.distance.front_cm, 1, ' cm')) +
    row('Rear 이동', n(s.distance.rear_cm, 1, ' cm'));
  document.getElementById('placement').innerHTML =
    row('상태', esc(g.state)) +
    row('ID0 raw forward(카메라→마커)', n(g.raw_forward_cm, 1, ' cm')) +
    row('추정 중심 종방향 간격(raw x+offset)', n(g.centre_distance_cm, 1, ' cm')) +
    row('종방향 기준 오차(+ = 추정값 큼)', signed(g.centre_error_cm, 1, ' cm')) +
    row('raw lateral 오차(+ = Front ID0가 Rear 기준 왼쪽, 목표 0)', signed(g.raw_lateral_cm, 1, ' cm')) +
    row('raw yaw 오차(+ = 위에서 볼 때 Front가 Rear보다 CCW, 목표 0)', signed(g.raw_yaw_deg, 1, '°')) +
    row('목표 중심 간격', n(g.target_centre_cm, 1, ' cm')) +
    row('forward offset', n(g.offset_cm, 1, ' cm')) +
    row('허용오차 중심/lat/yaw', g.tolerances) +
    row('보정 YAML', g.calibration_available ? '사용 가능' : '없음') +
    row('추정값', g.estimate_available ? '사용 가능' : '대기');
  document.getElementById('blockers').innerHTML =
    s.blockers.map(x => '<li>' + esc(x) + '</li>').join('');
}
async function boot() {
  try {
    const response = await fetch('/api/config', {cache:'no-store'});
    if (!response.ok) throw new Error('config HTTP ' + response.status);
    const c = await response.json();
    if (!c || !Number.isFinite(Number(c.preview_port)) ||
        typeof c.preview_path !== 'string') throw new Error('invalid config');
    document.getElementById('camera').src = location.protocol + '//' +
      location.hostname + ':' + c.preview_port + c.preview_path;
  } catch (e) {
    renderDisconnected();
  }
  tick();
  setInterval(tick, 250);
  setInterval(statusWatchdog, 100);
}
boot();
</script></body></html>'''


def _yaw_from_quaternion(q):
    values = (float(q.x), float(q.y), float(q.z), float(q.w))
    if not all(math.isfinite(value) for value in values):
        raise ValueError('non-finite quaternion')
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 1e-9:
        raise ValueError('zero quaternion')
    qx, qy, qz, qw = (value / norm for value in values)
    return math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz))


class RigidPairTeleopNode(Node):
    CONTROL_STATES = {'ARMING', 'ARMED', 'FAULT', 'LIMIT'}
    MOVEMENT_KEYS = {'w', 'a', 's', 'd', 'q', 'e', ' '}

    def __init__(self):
        super().__init__('rigid_pair_teleop_node')
        self.declare_parameter('linear_speed_mps', DEFAULT_LINEAR_SPEED_MPS)
        self.declare_parameter('angular_speed_rps', 0.12)
        self.declare_parameter('deadman_s', 0.30)
        self.declare_parameter('hardware_timeout_s', 0.60)
        self.declare_parameter('manual_timeout_s', 0.60)
        self.declare_parameter('odom_timeout_s', 0.50)
        self.declare_parameter('source_future_tolerance_s', 0.10)
        self.declare_parameter('odom_max_step_m', 0.10)
        self.declare_parameter('relative_stable_forward_span_m', 0.010)
        self.declare_parameter('relative_stable_lateral_span_m', 0.010)
        self.declare_parameter('relative_stable_yaw_span_deg', 2.0)
        self.declare_parameter('require_fused_odom', False)
        self.declare_parameter('fused_odom_timeout_s', 0.50)
        self.declare_parameter('require_cctv_marker', False)
        self.declare_parameter('cctv_marker_timeout_s', 0.50)
        self.declare_parameter('marker_timeout_s', 0.35)
        self.declare_parameter('pair_separation_m', DEFAULT_WHEELBASE_M)
        self.declare_parameter('aruco_distance_offset_m', 0.0)
        self.declare_parameter('placement_centre_tolerance_m', 0.015)
        self.declare_parameter('placement_lateral_tolerance_m', 0.015)
        self.declare_parameter('placement_yaw_tolerance_deg', 2.0)
        self.declare_parameter('arm_timeout_s', 10.0)
        self.declare_parameter('min_marker_distance_m', 0.10)
        self.declare_parameter('max_marker_distance_m', 1.00)
        self.declare_parameter('initial_lateral_limit_m', 0.10)
        self.declare_parameter('initial_yaw_limit_deg', 15.0)
        self.declare_parameter('gap_stop_m', 0.03)
        self.declare_parameter('lateral_stop_m', 0.03)
        self.declare_parameter('yaw_stop_deg', 5.0)
        self.declare_parameter('max_session_distance_m', 0.30)
        self.declare_parameter('web_host', '127.0.0.1')
        self.declare_parameter('web_port', 5007)
        self.declare_parameter('preview_port', 5005)
        self.declare_parameter('preview_path', '/video/0')

        gp = self.get_parameter
        self.teleop = KeyboardTeleopState(
            linear_speed=float(gp('linear_speed_mps').value),
            angular_speed=float(gp('angular_speed_rps').value),
            deadman_s=float(gp('deadman_s').value))
        self.limits = RigidPairTeleopLimits(
            gap_stop_m=float(gp('gap_stop_m').value),
            lateral_stop_m=float(gp('lateral_stop_m').value),
            yaw_stop_rad=math.radians(float(gp('yaw_stop_deg').value)),
            max_session_distance_m=float(
                gp('max_session_distance_m').value))
        self.limits.validate()
        self.hardware_timeout = float(gp('hardware_timeout_s').value)
        self.manual_timeout = float(gp('manual_timeout_s').value)
        self.odom_timeout = float(gp('odom_timeout_s').value)
        self.source_future_tolerance = float(
            gp('source_future_tolerance_s').value)
        self.odom_max_step_m = float(gp('odom_max_step_m').value)
        self.relative_stable_forward_span = float(
            gp('relative_stable_forward_span_m').value)
        self.relative_stable_lateral_span = float(
            gp('relative_stable_lateral_span_m').value)
        self.relative_stable_yaw_span = math.radians(float(
            gp('relative_stable_yaw_span_deg').value))
        self.require_fused_odom = bool(gp('require_fused_odom').value)
        self.fused_odom_timeout = float(gp('fused_odom_timeout_s').value)
        self.require_cctv_marker = bool(
            gp('require_cctv_marker').value)
        self.cctv_marker_timeout = float(
            gp('cctv_marker_timeout_s').value)
        self.marker_timeout = float(gp('marker_timeout_s').value)
        self.pair_separation_m = float(gp('pair_separation_m').value)
        self.aruco_distance_offset_m = float(
            gp('aruco_distance_offset_m').value)
        self.placement_centre_tolerance = float(
            gp('placement_centre_tolerance_m').value)
        self.placement_lateral_tolerance = float(
            gp('placement_lateral_tolerance_m').value)
        self.placement_yaw_tolerance = math.radians(float(
            gp('placement_yaw_tolerance_deg').value))
        self.arm_timeout = float(gp('arm_timeout_s').value)
        self.min_marker_distance = float(gp('min_marker_distance_m').value)
        self.max_marker_distance = float(gp('max_marker_distance_m').value)
        self.initial_lateral_limit = float(gp('initial_lateral_limit_m').value)
        self.initial_yaw_limit = math.radians(
            float(gp('initial_yaw_limit_deg').value))
        if (not math.isfinite(self.pair_separation_m) or
                self.pair_separation_m <= 0.0):
            raise ValueError('pair_separation_m must be finite and positive')
        if (self.source_future_tolerance < 0.0 or
                self.odom_max_step_m <= 0.0 or
                self.relative_stable_forward_span <= 0.0 or
                self.relative_stable_lateral_span <= 0.0 or
                self.relative_stable_yaw_span <= 0.0):
            raise ValueError('invalid rigid-pair sensor safety parameter')
        if (not all(math.isfinite(value) and value > 0.0 for value in (
                    self.placement_centre_tolerance,
                    self.placement_lateral_tolerance,
                    self.placement_yaw_tolerance))):
            raise ValueError('placement guide tolerances must be positive')

        self.state = 'IDLE'
        self.decision = '정지 상태입니다. 강체 쌍 준비를 누르세요.'
        self.reason = ''
        self.estop = False
        self.arm_deadline = 0.0
        self.reference = None
        self.relative = None
        self.raw_relative = None
        self.relative_samples = deque(maxlen=3)
        self.relative_sample_times = deque(maxlen=3)
        self.relative_time = 0.0
        self.marker_visible = False
        self.marker_true_time = 0.0
        self.marker_false_time = 0.0
        self.ready = {role: {'value': False, 'time': 0.0}
                      for role in ('front', 'rear')}
        self.manual = {role: {'value': False, 'time': 0.0}
                       for role in ('front', 'rear')}
        self.odom = {role: {'pose': None, 'time': 0.0}
                     for role in ('front', 'rear')}
        self.fused_odom = {role: {'pose': None, 'time': 0.0}
                           for role in ('front', 'rear')}
        self.cctv_marker = {role: {'value': False, 'time': 0.0}
                            for role in ('front', 'rear')}
        self.start_odom = {'front': None, 'rear': None}
        self.distance = {'front': 0.0, 'rear': 0.0}
        self.odom_path = {role: OdomPathAccumulator(self.odom_max_step_m)
                          for role in ('front', 'rear')}
        self.odom_path_ok = {role: True for role in ('front', 'rear')}
        self.wheel_odom_gate = {
            role: StampGate(self.odom_timeout, self.source_future_tolerance)
            for role in ('front', 'rear')}
        self.fused_odom_gate = {
            role: StampGate(self.fused_odom_timeout,
                            self.source_future_tolerance)
            for role in ('front', 'rear')}
        self.relative_gate = StampGate(
            self.marker_timeout, self.source_future_tolerance)
        self.last_commands = {'front': (0.0, 0.0, 0.0),
                              'rear': (0.0, 0.0, 0.0)}
        self.key_intent = '정지'
        self._requests = queue.SimpleQueue()
        self._last_enable_publish = 0.0
        self._terminal_old = None
        self._terminal_enabled = bool(sys.stdin.isatty())
        if self._terminal_enabled:
            self._terminal_old = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())

        self.pub_enable = {
            role: self.create_publisher(
                Bool, f'/{role}/manual_enable', 10)
            for role in ('front', 'rear')}
        self.pub_command = {
            role: self.create_publisher(
                TwistStamped, f'/{role}/manual_cmd_vel', CMD_VEL_QOS)
            for role in ('front', 'rear')}
        self.pub_estop = self.create_publisher(Bool, '/emergency_stop', 10)
        self.pub_status = self.create_publisher(
            String, '/rigid_pair_teleop/status', 10)
        # Kept only for dashboards/scripts that still use the old name.
        self.pub_legacy_status = self.create_publisher(
            String, '/keyboard_follow/status', 10)

        for role in ('front', 'rear'):
            self.create_subscription(
                Bool, f'/{role}/hardware_ready',
                lambda msg, r=role: self._bool_cb(self.ready[r], msg), 10)
            self.create_subscription(
                Bool, f'/{role}/manual_active',
                lambda msg, r=role: self._bool_cb(self.manual[r], msg), 10)
            self.create_subscription(
                Odometry, f'/{role}/wheel_odom',
                lambda msg, r=role: self._odom_cb(r, msg),
                qos_profile_sensor_data)
            self.create_subscription(
                Odometry, f'/{role}/odom',
                lambda msg, r=role: self._fused_odom_cb(r, msg),
                qos_profile_sensor_data)
            self.create_subscription(
                Bool, f'/{role}/cctv_marker_visible',
                lambda msg, r=role: self._bool_cb(self.cctv_marker[r], msg),
                qos_profile_sensor_data)
        self.create_subscription(
            PoseStamped, '/sync/relative_pose', self._relative_cb,
            qos_profile_sensor_data)
        self.create_subscription(
            Bool, '/sync/marker_visible', self._marker_cb,
            qos_profile_sensor_data)
        self.create_subscription(Bool, '/emergency_stop', self._estop_cb, 10)

        self.create_timer(0.02, self._control_loop)
        self.create_timer(0.25, self._publish_status)
        if self._terminal_enabled:
            self.create_timer(0.02, self._poll_terminal)
        self._start_web()
        self.get_logger().warn(
            f'강체 쌍 키보드 제어: http://{str(gp("web_host").value)}:'
            f'{int(gp("web_port").value)}/ '
            '(ID0 상대 pose 기준을 캡처, 기본 정지)')

    @staticmethod
    def _bool_cb(sample, msg):
        sample['value'] = bool(msg.data)
        sample['time'] = time.monotonic()

    def _odom_cb(self, role, msg):
        if self._pose_cb(
                self.odom[role], msg, self.wheel_odom_gate[role], role):
            if not self.odom_path[role].add(self.odom[role]['pose']):
                self.odom_path_ok[role] = False

    def _fused_odom_cb(self, role, msg):
        self._pose_cb(
            self.fused_odom[role], msg, self.fused_odom_gate[role], role)

    def _pose_cb(self, sample, msg, gate, role):
        if (msg.header.frame_id != 'map' or
                msg.child_frame_id != f'{role}_base'):
            self.get_logger().warn(
                f'{role} odom frame rejected: '
                f'{msg.header.frame_id}/{msg.child_frame_id}',
                throttle_duration_sec=1.0)
            return False
        try:
            pose = (
                float(msg.pose.pose.position.x),
                float(msg.pose.pose.position.y),
                _yaw_from_quaternion(msg.pose.pose.orientation))
        except ValueError:
            return False
        if not all(math.isfinite(value) for value in pose):
            return False
        now_ns = self.get_clock().now().nanoseconds
        accepted, _reason = gate.accept(stamp_to_ns(msg.header.stamp), now_ns)
        if not accepted:
            return False
        sample.update({'pose': pose, 'time': time.monotonic()})
        return True

    def _relative_cb(self, msg):
        if msg.header.frame_id != 'rear_base':
            self.get_logger().warn(
                f'ID0 pose frame rejected: {msg.header.frame_id}',
                throttle_duration_sec=1.0)
            return
        try:
            values = (
                float(msg.pose.position.x),
                float(msg.pose.position.y),
                _yaw_from_quaternion(msg.pose.orientation))
        except ValueError:
            return
        if not all(math.isfinite(value) for value in values):
            return
        now = time.monotonic()
        accepted, _reason = self.relative_gate.accept(
            stamp_to_ns(msg.header.stamp), self.get_clock().now().nanoseconds)
        if not accepted:
            return
        if (self.relative_time <= 0.0 or
                now - self.relative_time > self.marker_timeout):
            self.relative_samples.clear()
            self.relative_sample_times.clear()
        self.raw_relative = values
        self.relative_samples.append(values)
        self.relative_sample_times.append(now)
        self.relative = median_relative_pose(self.relative_samples)
        self.relative_time = now
        # A valid, fresh ID0 pose is positive visibility evidence.  This
        # prevents Bool/Pose callback ordering from falsely stopping a pair.
        self.marker_visible = True
        self.marker_true_time = now

    def _marker_cb(self, msg):
        self.marker_visible = bool(msg.data)
        if self.marker_visible:
            self.marker_true_time = time.monotonic()
        else:
            # A camera-confirmed disappearance is immediate fail-closed.
            self.marker_false_time = time.monotonic()
            self.relative_samples.clear()
            self.relative_sample_times.clear()

    def _estop_cb(self, msg):
        if msg.data:
            self.estop = True
            self.teleop.stop()
            self.state = 'ESTOP'
            self.reason = '비상정지 신호 감지'
            self.decision = '비상정지가 고정되었습니다. 전원을 재인가해야 합니다.'

    def _poll_terminal(self):
        while select.select([sys.stdin], [], [], 0.0)[0]:
            key = os.read(sys.stdin.fileno(), 1).decode(
                'ascii', errors='ignore').lower()
            if key == '\x03':
                raise KeyboardInterrupt
            self._requests.put(('key', key))

    @staticmethod
    def _fresh(sample, timeout, now):
        return (bool(sample['value']) and sample['time'] > 0.0 and
                0.0 <= now - sample['time'] <= timeout)

    def _odom_fresh(self, role, now):
        sample = self.odom[role]
        return (sample['pose'] is not None and sample['time'] > 0.0 and
                0.0 <= now - sample['time'] <= self.odom_timeout)

    def _fused_odom_fresh(self, role, now):
        sample = self.fused_odom[role]
        return (sample['pose'] is not None and sample['time'] > 0.0 and
                0.0 <= now - sample['time'] <= self.fused_odom_timeout)

    def _cctv_marker_fresh(self, role, now):
        return self._fresh(
            self.cctv_marker[role], self.cctv_marker_timeout, now)

    def _marker_fresh(self, now):
        return (self.marker_visible and self.relative is not None and
                0.0 <= now - self.relative_time <= self.marker_timeout and
                0.0 <= now - self.marker_true_time <= self.marker_timeout and
                self.relative_time >= self.marker_false_time)

    def _relative_stable_for_arm(self, now):
        return (
            len(self.relative_samples) >= 3 and
            len(self.relative_sample_times) >= 3 and
            now - self.relative_sample_times[0] <= self.marker_timeout and
            relative_pose_is_stable(
                self.relative_samples,
                forward_span_m=self.relative_stable_forward_span,
                lateral_span_m=self.relative_stable_lateral_span,
                yaw_span_rad=self.relative_stable_yaw_span))

    def _graph_conflicts(self):
        conflicts = []
        try:
            for role in ('front', 'rear'):
                auto_topic = f'/{role}/cmd_vel'
                if self.get_publishers_info_by_topic(auto_topic):
                    conflicts.append(auto_topic)
                manual_topic = f'/{role}/manual_cmd_vel'
                # This controller itself is the one allowed manual publisher.
                if len(self.get_publishers_info_by_topic(manual_topic)) > 1:
                    conflicts.append(manual_topic)
        except Exception:
            conflicts.append('ROS graph 조회 실패')
        return conflicts

    def _blockers(self, now, require_manual=False):
        blockers = []
        if self.estop:
            blockers.append('비상정지가 고정되어 있음')
        for role, label in (('front', 'robot-2 Front'),
                            ('rear', 'robot-1 Rear')):
            if not self._fresh(self.ready[role], self.hardware_timeout, now):
                blockers.append(f'{label} hardware_ready가 없거나 오래됨')
            if not self._odom_fresh(role, now):
                blockers.append(f'{label} wheel_odom이 없거나 오래됨')
            if (self.require_fused_odom and
                    not self._fused_odom_fresh(role, now)):
                blockers.append(f'{label} fused odom이 없거나 오래됨')
            if (self.require_cctv_marker and
                    not self._cctv_marker_fresh(role, now)):
                blockers.append(f'{label} CCTV marker가 없거나 오래됨')
            if require_manual and not self._fresh(
                    self.manual[role], self.manual_timeout, now):
                blockers.append(f'{label} 수동 제어권 확인 안 됨')
        if not self._marker_fresh(now):
            blockers.append('ID0 ArUco pose가 보이지 않거나 오래됨')
        elif not self._relative_stable_for_arm(now):
            blockers.append('ID0 ArUco pose 3개가 아직 안정화되지 않음')
        elif self.relative is not None:
            forward, lateral, yaw = self.relative
            if not self.min_marker_distance <= forward <= self.max_marker_distance:
                blockers.append(
                    f'ArUco 간격 {forward * 100.0:.1f} cm가 허용 범위 밖')
            if abs(lateral) > self.initial_lateral_limit:
                blockers.append('초기 좌우 정렬이 10 cm보다 큼')
            if abs(yaw) > self.initial_yaw_limit:
                blockers.append('초기 상대 각도가 15°보다 큼')
        conflicts = self._graph_conflicts()
        if conflicts:
            blockers.append('다른 주행 발행자 존재: ' + ', '.join(conflicts))
        return blockers

    def _process_requests(self, now):
        while True:
            try:
                kind, value = self._requests.get_nowait()
            except queue.Empty:
                return
            if kind == 'estop':
                self._publish_zero()
                self.pub_estop.publish(Bool(data=True))
                self.estop = True
                self.state = 'ESTOP'
                self.reason = '사용자가 비상정지를 요청함'
                self.decision = '양쪽 STM32 비상정지가 고정되었습니다.'
                continue
            if kind == 'arm':
                value = 'r'
            elif kind == 'disarm':
                value = 'x'
            if kind not in {'key', 'arm', 'disarm'}:
                continue
            key = str(value).lower()
            if key == 'r':
                if self.state != 'IDLE':
                    self.decision = '먼저 X 또는 제어권 해제로 IDLE로 돌아가세요.'
                    continue
                self.state = 'ARMING'
                self.reason = ''
                self.arm_deadline = now + self.arm_timeout
                self.decision = '현재 ArUco 간격과 양쪽 수동 제어권을 확인 중입니다.'
                continue
            if key == 'x':
                if self.state != 'ESTOP':
                    self.teleop.stop()
                    self._publish_zero()
                    self.state = 'IDLE'
                    self.reference = None
                    self.reason = ''
                    self.decision = '정지하고 양쪽 제어권을 해제했습니다.'
                continue
            if key in {'t', 'g'}:
                self.teleop.stop()
                self.decision = '이 모드에서는 그리퍼 명령이 비활성화되어 있습니다.'
                continue
            if key not in self.MOVEMENT_KEYS:
                continue
            if key == ' ':
                self.teleop.stop()
                self.key_intent = '정지'
                self._publish_zero()
                self.decision = 'Space 정지 · 강체 쌍 준비 상태는 유지합니다.'
                continue
            if self.state != 'ARMED':
                self.teleop.stop()
                self.decision = '강체 쌍 준비가 완료된 뒤에만 이동할 수 있습니다.'
                continue
            self.teleop.handle_key(key, now)
            self.key_intent = key.upper()
            self.decision = f'{key.upper()} 입력 · ID0 상대 자세 유지 중'

    def _publish_enable(self, enabled):
        msg = Bool(data=bool(enabled))
        for publisher in self.pub_enable.values():
            publisher.publish(msg)

    def _publish_pair_commands(self, front_command, rear_command):
        """Publish one centre-control cycle with one shared ROS timestamp.

        DDS cannot make two STM32s physically sample at exactly the same
        instant, but the pair is computed once and both command messages carry
        the same time.  This is deliberately not a leader/follower sequence.
        """
        stamp = self.get_clock().now().to_msg()
        commands = {'front': front_command, 'rear': rear_command}
        for role in ('front', 'rear'):
            self._publish_command(role, commands[role], stamp)

    def _publish_command(self, role, command, stamp):
        msg = TwistStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = f'{role}_base'
        (msg.twist.linear.x, msg.twist.linear.y,
         msg.twist.angular.z) = tuple(command)
        self.pub_command[role].publish(msg)
        self.last_commands[role] = tuple(float(value) for value in command)

    def _publish_zero(self):
        self._publish_pair_commands(
            (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        self.last_commands = {'front': (0.0, 0.0, 0.0),
                              'rear': (0.0, 0.0, 0.0)}

    def _session_distances(self):
        for role in ('front', 'rear'):
            self.distance[role] = self.odom_path[role].distance_m
        return self.distance['front'], self.distance['rear']

    def _fault(self, state, reason):
        self.teleop.stop()
        self._publish_zero()
        self.state = state
        self.reason = str(reason)
        self.decision = _STATE_LABELS[state] + ': ' + str(reason)
        self.get_logger().error(self.decision)

    def _control_loop(self):
        now = time.monotonic()
        self._process_requests(now)
        owns_control = self.state in self.CONTROL_STATES
        if now - self._last_enable_publish >= 0.20:
            self._publish_enable(owns_control)
            self._last_enable_publish = now

        if self.state == 'ARMING':
            self._publish_zero()
            blockers = self._blockers(now, require_manual=True)
            if not blockers:
                # Raw ID0 pose is the feedback reference.  The calibrated
                # robot-centre lever arm is pair_separation_m, used below.
                self.reference = capture_pair_reference(self.relative)
                self.start_odom = {
                    role: self.odom[role]['pose'] for role in ('front', 'rear')}
                self.distance = {'front': 0.0, 'rear': 0.0}
                for role in ('front', 'rear'):
                    self.odom_path[role].reset()
                    self.odom_path[role].add(self.odom[role]['pose'])
                    self.odom_path_ok[role] = True
                self.state = 'ARMED'
                self.decision = (
                    f'준비 완료 · 현재 ArUco 간격 '
                    f'{self.reference[0] * 100.0:.1f} cm를 목표로 유지합니다.')
            elif now >= self.arm_deadline:
                self._fault('FAULT', '준비 시간 초과: ' + blockers[0])
            else:
                self.decision = '준비 중: ' + blockers[0]
            return

        if self.state != 'ARMED':
            if owns_control:
                self._publish_zero()
            return
        if self.reference is None or self.relative is None:
            self._fault('FAULT', 'ArUco 기준값이 없음')
            return

        forward, lateral, yaw = self.relative
        gap_error = forward - self.reference[0]
        lateral_error = lateral - self.reference[1]
        yaw_error = angle_norm(yaw - self.reference[2])
        front_distance, rear_distance = self._session_distances()
        conflicts = self._graph_conflicts()
        decision = evaluate_rigid_pair(
            self.limits,
            gap_error_m=gap_error,
            lateral_error_m=lateral_error,
            yaw_error_rad=yaw_error,
            front_distance_m=front_distance,
            rear_distance_m=rear_distance,
            hardware_ok=all(self._fresh(
                self.ready[role], self.hardware_timeout, now)
                for role in ('front', 'rear')),
            manual_ok=all(self._fresh(
                self.manual[role], self.manual_timeout, now)
                for role in ('front', 'rear')),
            marker_ok=self._marker_fresh(now),
            odom_ok=(
                all(self._odom_fresh(role, now)
                    for role in ('front', 'rear')) and
                all(self.odom_path_ok[role] for role in ('front', 'rear')) and
                (not self.require_fused_odom or
                 all(self._fused_odom_fresh(role, now)
                     for role in ('front', 'rear'))) and
                (not self.require_cctv_marker or
                 all(self._cctv_marker_fresh(role, now)
                     for role in ('front', 'rear')))),
            graph_ok=not conflicts,
            estop=self.estop)
        if decision.outcome != 'CONTINUE':
            state = decision.outcome if decision.outcome in {
                'LIMIT', 'ESTOP'} else 'FAULT'
            self._fault(state, decision.reason)
            return

        intent = self.teleop.velocity(now)
        if is_zero(intent):
            self.key_intent = '정지'
            self._publish_zero()
            return
        # ID0's raw forward value is the relative-pose reference only.  Its
        # camera-to-marker offset is not the Front/Rear centre lever arm.
        front_cmd, rear_cmd = split_pair_centre_twist(
            self.limits, intent, self.pair_separation_m,
            gap_error_m=gap_error,
            lateral_error_m=lateral_error,
            yaw_error_rad=yaw_error)
        self._publish_pair_commands(front_cmd, rear_cmd)

    def _status_payload(self):
        now = time.monotonic()
        relative = self.relative if self._marker_fresh(now) else None
        reference = self.reference
        gap_error = None if relative is None or reference is None else (
            relative[0] - reference[0])
        lateral_error = None if relative is None or reference is None else (
            relative[1] - reference[1])
        yaw_error = None if relative is None or reference is None else angle_norm(
            relative[2] - reference[2])
        raw_yaw_error = (
            None if relative is None or reference is None
            else angle_norm(relative[2] - reference[2]))
        placement = evaluate_placement_guide(
            relative_pose=relative, marker_fresh=self._marker_fresh(now),
            stable=self._relative_stable_for_arm(now),
            pair_separation_m=self.pair_separation_m,
            aruco_distance_offset_m=self.aruco_distance_offset_m,
            centre_tolerance_m=self.placement_centre_tolerance,
            lateral_tolerance_m=self.placement_lateral_tolerance,
            yaw_tolerance_rad=self.placement_yaw_tolerance)
        return {
            'state': self.state,
            'state_label': _STATE_LABELS[self.state],
            'decision': self.decision,
            'reason': self.reason,
            'blockers': self._blockers(
                now, require_manual=self.state in self.CONTROL_STATES),
            'key_intent': self.key_intent,
            'pose': {
                'reference_forward_cm': None if reference is None else
                reference[0] * 100.0,
                'forward_cm': None if relative is None else relative[0] * 100.0,
                'gap_error_cm': None if gap_error is None else gap_error * 100.0,
                'lateral_error_cm': None if lateral_error is None else
                lateral_error * 100.0,
                'yaw_error_deg': None if yaw_error is None else
                math.degrees(yaw_error),
                'raw_yaw_error_deg': None if raw_yaw_error is None else
                math.degrees(raw_yaw_error),
                'filter_samples': len(self.relative_samples),
            },
            'commands': {
                'front': self.last_commands['front'],
                'rear': self.last_commands['rear'],
            },
            'distance': {
                'front_cm': self.distance['front'] * 100.0,
                'rear_cm': self.distance['rear'] * 100.0,
            },
            'placement': {
                'state': placement.state,
                'raw_forward_cm': None if placement.raw_forward_m is None
                else placement.raw_forward_m * 100.0,
                'centre_distance_cm': None if placement.centre_distance_m is None
                else placement.centre_distance_m * 100.0,
                'centre_error_cm': None if placement.centre_error_m is None
                else placement.centre_error_m * 100.0,
                'raw_lateral_cm': None if placement.raw_lateral_error_m is None
                else placement.raw_lateral_error_m * 100.0,
                'raw_yaw_deg': None if placement.raw_yaw_error_rad is None
                else math.degrees(placement.raw_yaw_error_rad),
                'calibration_available': placement.calibration_available,
                'estimate_available': placement.estimate_available,
                'target_centre_cm': self.pair_separation_m * 100.0,
                'offset_cm': self.aruco_distance_offset_m * 100.0
                if placement.calibration_available else None,
                'tolerances': (
                    f'±{self.placement_centre_tolerance * 100.0:.1f}cm / '
                    f'±{self.placement_lateral_tolerance * 100.0:.1f}cm / '
                    f'±{math.degrees(self.placement_yaw_tolerance):.1f}°'
                ),
            },
        }

    def _publish_status(self):
        try:
            data = json.dumps(
                self._status_payload(), ensure_ascii=False, allow_nan=False)
        except (TypeError, ValueError):
            return
        message = String(data=data)
        self.pub_status.publish(message)
        self.pub_legacy_status.publish(message)

    def _start_web(self):
        if not WEB_OK:
            raise RuntimeError('Flask/Werkzeug가 없어 강체 쌍 UI를 열 수 없음')
        app = Flask('rigid_pair_teleop')

        @app.get('/')
        def index():
            return _HTML

        @app.get('/api/status')
        def status():
            return jsonify(self._status_payload())

        @app.get('/api/config')
        def config():
            return jsonify({
                'preview_port': int(self.get_parameter('preview_port').value),
                'preview_path': str(self.get_parameter('preview_path').value),
                'deadman_s': self.teleop.deadman_s,
            })

        @app.post('/api/key')
        def key():
            if not request_origin_is_same_host(
                    request.headers.get('Origin'), request.host):
                return jsonify({'accepted': False, 'reason': 'cross-origin'}), 403
            payload = request.get_json(silent=True) or {}
            value = str(payload.get('key', '')).lower()
            if value not in self.MOVEMENT_KEYS | {'r', 'x'}:
                return jsonify({'accepted': False}), 400
            self._requests.put(('key', value))
            return jsonify({'accepted': True}), 202

        @app.post('/api/arm')
        def arm():
            if not request_origin_is_same_host(
                    request.headers.get('Origin'), request.host):
                return jsonify({'accepted': False, 'reason': 'cross-origin'}), 403
            self._requests.put(('arm', ''))
            return jsonify({'accepted': True}), 202

        @app.post('/api/disarm')
        def disarm():
            if not request_origin_is_same_host(
                    request.headers.get('Origin'), request.host):
                return jsonify({'accepted': False, 'reason': 'cross-origin'}), 403
            self._requests.put(('disarm', ''))
            return jsonify({'accepted': True}), 202

        @app.post('/api/estop')
        def estop():
            if not request_origin_is_same_host(
                    request.headers.get('Origin'), request.host):
                return jsonify({'accepted': False, 'reason': 'cross-origin'}), 403
            self._requests.put(('estop', ''))
            return jsonify({'accepted': True}), 202

        host = str(self.get_parameter('web_host').value)
        port = int(self.get_parameter('web_port').value)
        self._server = make_server(host, port, app, threaded=True)
        self._web_thread = threading.Thread(
            target=self._server.serve_forever,
            name='keyboard-follow-web', daemon=True)
        self._web_thread.start()

    def destroy_node(self):
        self.teleop.stop()
        self._publish_zero()
        self._publish_enable(False)
        try:
            self._server.shutdown()
        except Exception:
            pass
        if getattr(self, '_web_thread', None) is not None:
            self._web_thread.join(timeout=2.0)
        if self._terminal_old is not None:
            termios.tcsetattr(
                sys.stdin, termios.TCSADRAIN, self._terminal_old)
            self._terminal_old = None
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = RigidPairTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


# ``keyboard_follow`` is a legacy executable name.  The class alias avoids
# breaking direct imports while making new call sites use RigidPairTeleopNode.
KeyboardFollowNode = RigidPairTeleopNode
