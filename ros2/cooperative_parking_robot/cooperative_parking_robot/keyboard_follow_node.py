#!/usr/bin/env python3
"""Keyboard/web control for a bounded Front-led, ArUco-held robot pair.

One node owns both manual command channels.  It captures the currently
observed ArUco forward/lateral/yaw as the exact reference when armed, commands
Front and Rear as a virtual rigid pair, and stops both on stale telemetry,
graph conflicts, relative-pose deviation, or a bounded session-distance limit.
No gripper command is exposed in this mode.
"""

from __future__ import annotations

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

from cooperative_parking_robot.keyboard_follow_core import (
    KeyboardFollowLimits,
    angle_norm,
    capture_aruco_reference,
    evaluate_follow,
    follow_pair_commands,
    is_zero,
)
from cooperative_parking_robot.manual_control import (
    DEFAULT_LINEAR_SPEED_MPS,
    KeyboardTeleopState,
)

try:
    from flask import Flask, jsonify, request
    from werkzeug.serving import make_server
    WEB_OK = True
except ImportError:
    WEB_OK = False


_STATE_LABELS = {
    'IDLE': '정지 · 제어권 없음',
    'ARMING': '양쪽 제어권 확인 중',
    'ARMED': '키보드 추종 준비 완료',
    'FAULT': '안전 조건 위반 · 정지 유지',
    'LIMIT': '세션 거리 제한 · 정지 유지',
    'ESTOP': '비상정지 고정',
}


_HTML = r'''<!doctype html><html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>키보드 협동 추종</title><style>
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
<header><h1>robot-2 키보드 · robot-1 ArUco 추종</h1><span id="state" class="pill">연결 중…</span>
<span class="small">페이지를 클릭한 뒤 WASD / Q·E / Space</span></header><main>
<section class="panel"><img id="camera" class="camera" alt="Rear ArUco camera">
<div class="keys">
<button class="blank">.</button>
<button class="key" onpointerdown="key('w')">W</button>
<button class="blank">.</button>
<button class="key" onpointerdown="key('a')">A</button>
<button class="key stop" onpointerdown="key(' ')">■</button>
<button class="key" onpointerdown="key('d')">D</button>
<button class="key" onpointerdown="key('q')">Q</button>
<button class="key" onpointerdown="key('s')">S</button>
<button class="key" onpointerdown="key('e')">E</button>
</div>
<p class="small">W/S 전후 · A/D 횡이동 · Q/E 두 로봇 중점 회전 · Space 정지. 키를
누르는 동안 브라우저 반복 입력이 들어오며, 입력이 0.30초 끊기면 자동 정지합니다.</p></section>
<section class="panel"><div id="decision" class="decision">상태 수신 중…</div>
<div class="actions"><button class="arm" onclick="act('arm')">추종 준비</button>
<button class="stop" onclick="act('disarm')">정지·제어권 해제</button>
<button class="estop" onclick="emergency()">양쪽 비상정지</button></div>
<div class="rows"><div class="card"><h3>기준과 현재</h3><div id="pose"></div></div>
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
async function key(k) {
  await fetch('/api/key', {
    method:'POST',
    headers:{'Content-Type':'application/json'},
    body:JSON.stringify({key:k})
  });
}
async function act(a) {
  await fetch('/api/' + a, {method:'POST'});
}
function emergency() {
  if (confirm('양쪽 STM32에 고정 비상정지를 전송할까요?')) act('estop');
}
document.addEventListener('keydown', e => {
  const k = e.key.toLowerCase();
  if (['w','a','s','d','q','e',' '].includes(k)) {
    e.preventDefault();
    key(k);
  }
});
async function tick() {
  let s;
  try {
    s = await (await fetch('/api/status', {cache:'no-store'})).json();
  } catch (e) {
    return;
  }
  const st = document.getElementById('state');
  st.textContent = s.state_label;
  st.className = 'pill ' + (s.state === 'ARMED' ? 'ok' :
    s.state === 'FAULT' || s.state === 'ESTOP' ? 'bad' : 'warn');
  document.getElementById('decision').textContent = s.decision;
  const p = s.pose;
  const c = s.commands;
  document.getElementById('pose').innerHTML =
    row('목표 간격', n(p.reference_forward_cm, 1, ' cm')) +
    row('현재 간격', n(p.forward_cm, 1, ' cm')) +
    row('간격 오차', n(p.gap_error_cm, 1, ' cm')) +
    row('좌우 오차', n(p.lateral_error_cm, 1, ' cm')) +
    row('각도 오차', n(p.yaw_error_deg, 1, '°'));
  document.getElementById('commands').innerHTML =
    row('키 입력', esc(s.key_intent)) +
    row('Front x/y/ω', c.front.map(v => Number(v).toFixed(3)).join(' / ')) +
    row('Rear x/y/ω', c.rear.map(v => Number(v).toFixed(3)).join(' / ')) +
    row('Front 이동', n(s.distance.front_cm, 1, ' cm')) +
    row('Rear 이동', n(s.distance.rear_cm, 1, ' cm'));
  document.getElementById('blockers').innerHTML =
    s.blockers.map(x => '<li>' + esc(x) + '</li>').join('');
}
async function boot() {
  const c = await (await fetch('/api/config')).json();
  document.getElementById('camera').src = location.protocol + '//' +
    location.hostname + ':' + c.preview_port + c.preview_path;
  tick();
  setInterval(tick, 250);
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


class KeyboardFollowNode(Node):
    CONTROL_STATES = {'ARMING', 'ARMED', 'FAULT', 'LIMIT'}
    MOVEMENT_KEYS = {'w', 'a', 's', 'd', 'q', 'e', ' '}

    def __init__(self):
        super().__init__('keyboard_follow_node')
        self.declare_parameter('linear_speed_mps', DEFAULT_LINEAR_SPEED_MPS)
        self.declare_parameter('angular_speed_rps', 0.12)
        self.declare_parameter('deadman_s', 0.30)
        self.declare_parameter('hardware_timeout_s', 0.60)
        self.declare_parameter('manual_timeout_s', 0.60)
        self.declare_parameter('odom_timeout_s', 0.50)
        self.declare_parameter('marker_timeout_s', 0.35)
        self.declare_parameter('arm_timeout_s', 10.0)
        self.declare_parameter('min_marker_distance_m', 0.10)
        self.declare_parameter('max_marker_distance_m', 1.00)
        self.declare_parameter('initial_lateral_limit_m', 0.10)
        self.declare_parameter('initial_yaw_limit_deg', 15.0)
        self.declare_parameter('gap_stop_m', 0.03)
        self.declare_parameter('lateral_stop_m', 0.03)
        self.declare_parameter('yaw_stop_deg', 5.0)
        self.declare_parameter('max_session_distance_m', 0.30)
        self.declare_parameter('web_host', '0.0.0.0')
        self.declare_parameter('web_port', 5007)
        self.declare_parameter('preview_port', 5005)
        self.declare_parameter('preview_path', '/video/0')

        gp = self.get_parameter
        self.teleop = KeyboardTeleopState(
            linear_speed=float(gp('linear_speed_mps').value),
            angular_speed=float(gp('angular_speed_rps').value),
            deadman_s=float(gp('deadman_s').value))
        self.limits = KeyboardFollowLimits(
            gap_stop_m=float(gp('gap_stop_m').value),
            lateral_stop_m=float(gp('lateral_stop_m').value),
            yaw_stop_rad=math.radians(float(gp('yaw_stop_deg').value)),
            max_session_distance_m=float(
                gp('max_session_distance_m').value))
        self.limits.validate()
        self.hardware_timeout = float(gp('hardware_timeout_s').value)
        self.manual_timeout = float(gp('manual_timeout_s').value)
        self.odom_timeout = float(gp('odom_timeout_s').value)
        self.marker_timeout = float(gp('marker_timeout_s').value)
        self.arm_timeout = float(gp('arm_timeout_s').value)
        self.min_marker_distance = float(gp('min_marker_distance_m').value)
        self.max_marker_distance = float(gp('max_marker_distance_m').value)
        self.initial_lateral_limit = float(gp('initial_lateral_limit_m').value)
        self.initial_yaw_limit = math.radians(
            float(gp('initial_yaw_limit_deg').value))

        self.state = 'IDLE'
        self.decision = '정지 상태입니다. 추종 준비를 누르세요.'
        self.reason = ''
        self.estop = False
        self.arm_deadline = 0.0
        self.reference = None
        self.relative = None
        self.relative_time = 0.0
        self.marker_visible = False
        self.marker_true_time = 0.0
        self.ready = {role: {'value': False, 'time': 0.0}
                      for role in ('front', 'rear')}
        self.manual = {role: {'value': False, 'time': 0.0}
                       for role in ('front', 'rear')}
        self.odom = {role: {'pose': None, 'time': 0.0}
                     for role in ('front', 'rear')}
        self.start_odom = {'front': None, 'rear': None}
        self.distance = {'front': 0.0, 'rear': 0.0}
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
                TwistStamped, f'/{role}/manual_cmd_vel', 10)
            for role in ('front', 'rear')}
        self.pub_estop = self.create_publisher(Bool, '/emergency_stop', 10)
        self.pub_status = self.create_publisher(
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
            f'키보드 협동 추종: http://0.0.0.0:{int(gp("web_port").value)}/ '
            '(초기 ArUco 간격을 정확한 목표로 캡처, 기본 정지)')

    @staticmethod
    def _bool_cb(sample, msg):
        sample['value'] = bool(msg.data)
        sample['time'] = time.monotonic()

    def _odom_cb(self, role, msg):
        try:
            pose = (
                float(msg.pose.pose.position.x),
                float(msg.pose.pose.position.y),
                _yaw_from_quaternion(msg.pose.pose.orientation))
        except ValueError:
            return
        if not all(math.isfinite(value) for value in pose):
            return
        self.odom[role] = {'pose': pose, 'time': time.monotonic()}

    def _relative_cb(self, msg):
        try:
            values = (
                float(msg.pose.position.x),
                float(msg.pose.position.y),
                _yaw_from_quaternion(msg.pose.orientation))
        except ValueError:
            return
        if not all(math.isfinite(value) for value in values):
            return
        self.relative = values
        self.relative_time = time.monotonic()

    def _marker_cb(self, msg):
        self.marker_visible = bool(msg.data)
        if self.marker_visible:
            self.marker_true_time = time.monotonic()

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

    def _marker_fresh(self, now):
        return (self.marker_visible and self.relative is not None and
                0.0 <= now - self.relative_time <= self.marker_timeout and
                0.0 <= now - self.marker_true_time <= self.marker_timeout)

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
            if require_manual and not self._fresh(
                    self.manual[role], self.manual_timeout, now):
                blockers.append(f'{label} 수동 제어권 확인 안 됨')
        if not self._marker_fresh(now):
            blockers.append('ID0 ArUco pose가 보이지 않거나 오래됨')
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
                self.decision = 'Space 정지 · 추종 준비 상태는 유지합니다.'
                continue
            if self.state != 'ARMED':
                self.teleop.stop()
                self.decision = '추종 준비가 완료된 뒤에만 이동할 수 있습니다.'
                continue
            self.teleop.handle_key(key, now)
            self.key_intent = key.upper()
            self.decision = f'{key.upper()} 입력 · ArUco 기준 간격 유지 중'

    def _publish_enable(self, enabled):
        msg = Bool(data=bool(enabled))
        for publisher in self.pub_enable.values():
            publisher.publish(msg)

    def _publish_command(self, role, command):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f'{role}_base'
        (msg.twist.linear.x, msg.twist.linear.y,
         msg.twist.angular.z) = tuple(command)
        self.pub_command[role].publish(msg)
        self.last_commands[role] = tuple(float(value) for value in command)

    def _publish_zero(self):
        for role in ('front', 'rear'):
            self._publish_command(role, (0.0, 0.0, 0.0))
        self.last_commands = {'front': (0.0, 0.0, 0.0),
                              'rear': (0.0, 0.0, 0.0)}

    def _session_distances(self):
        for role in ('front', 'rear'):
            start = self.start_odom[role]
            current = self.odom[role]['pose']
            if start is not None and current is not None:
                self.distance[role] = math.hypot(
                    current[0] - start[0], current[1] - start[1])
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
                # This raw ArUco forward value is the user's exact axle-gap
                # reference.  Do not add robot length or a nominal wheelbase.
                self.reference = capture_aruco_reference(self.relative)
                self.start_odom = {
                    role: self.odom[role]['pose'] for role in ('front', 'rear')}
                self.distance = {'front': 0.0, 'rear': 0.0}
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
        decision = evaluate_follow(
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
            odom_ok=all(self._odom_fresh(role, now)
                        for role in ('front', 'rear')),
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
        front_cmd, rear_cmd = follow_pair_commands(
            self.limits, intent, self.reference[0],
            gap_error_m=gap_error,
            lateral_error_m=lateral_error,
            yaw_error_rad=yaw_error)
        self._publish_command('front', front_cmd)
        self._publish_command('rear', rear_cmd)

    def _status_payload(self):
        now = time.monotonic()
        relative = self.relative
        reference = self.reference
        gap_error = None if relative is None or reference is None else (
            relative[0] - reference[0])
        lateral_error = None if relative is None or reference is None else (
            relative[1] - reference[1])
        yaw_error = None if relative is None or reference is None else angle_norm(
            relative[2] - reference[2])
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
            },
            'commands': {
                'front': self.last_commands['front'],
                'rear': self.last_commands['rear'],
            },
            'distance': {
                'front_cm': self.distance['front'] * 100.0,
                'rear_cm': self.distance['rear'] * 100.0,
            },
        }

    def _publish_status(self):
        try:
            data = json.dumps(
                self._status_payload(), ensure_ascii=False, allow_nan=False)
        except (TypeError, ValueError):
            return
        self.pub_status.publish(String(data=data))

    def _start_web(self):
        if not WEB_OK:
            raise RuntimeError('Flask/Werkzeug가 없어 키보드 추종 화면을 열 수 없음')
        app = Flask('keyboard_follow')

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
            payload = request.get_json(silent=True) or {}
            value = str(payload.get('key', '')).lower()
            if value not in self.MOVEMENT_KEYS | {'r', 'x'}:
                return jsonify({'accepted': False}), 400
            self._requests.put(('key', value))
            return jsonify({'accepted': True}), 202

        @app.post('/api/arm')
        def arm():
            self._requests.put(('arm', ''))
            return jsonify({'accepted': True}), 202

        @app.post('/api/disarm')
        def disarm():
            self._requests.put(('disarm', ''))
            return jsonify({'accepted': True}), 202

        @app.post('/api/estop')
        def estop():
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
    node = KeyboardFollowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
