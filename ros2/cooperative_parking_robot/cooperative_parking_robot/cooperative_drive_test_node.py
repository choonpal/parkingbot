#!/usr/bin/env python3
"""Bounded straight-line cooperative drive test with a browser dashboard.

The node uses the STM32 bridge's manual channel, never the production mission
state machine.  It starts disarmed, requires explicit ARM then START requests,
and continuously publishes zero commands whenever it is not RUNNING.
"""

from __future__ import annotations

import json
import math
import queue
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, String

from cooperative_parking_robot.cooperative_drive_test_core import (
    DriveTestLimits,
    angle_norm,
    evaluate_running,
    odom_progress,
    pair_commands,
)

try:
    from flask import Flask, jsonify
    from werkzeug.serving import make_server
    WEB_OK = True
except ImportError:
    WEB_OK = False


_STATE_LABELS = {
    'IDLE': '정지 · 제어권 없음',
    'ARMING': '양쪽 제어권 확인 중',
    'ARMED': '준비 완료 · 시작 대기',
    'RUNNING': '협동 직진 중',
    'COMPLETED': '시험 완료 · 정지 유지',
    'STOPPED': '사용자 정지 · 정지 유지',
    'FAULT': '안전 조건 위반 · 정지 유지',
    'ESTOP': '비상정지 고정',
}


_HTML = r'''<!doctype html>
<html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>협동 주행 시험</title>
<style>
:root{color-scheme:dark;--bg:#0c1118;--panel:#151e29;--line:#304157;
--text:#eef4fb;--muted:#9fb0c4;--green:#45d483;--red:#ff6b70;
--amber:#ffba55;--blue:#54aaff}*{box-sizing:border-box}body{margin:0;
font-family:system-ui,sans-serif;background:var(--bg);color:var(--text)}
header{padding:16px 20px;border-bottom:1px solid var(--line);display:flex;
align-items:center;gap:12px;flex-wrap:wrap}h1{margin:0;font-size:19px}.pill{
padding:6px 11px;border-radius:999px;background:#253246;font-size:13px}
.ok{color:var(--green)}.bad{color:var(--red)}.warn{color:var(--amber)}
main{display:grid;grid-template-columns:minmax(420px,1.35fr) minmax(350px,1fr);
gap:14px;padding:14px}@media(max-width:900px){main{grid-template-columns:1fr}}
.panel{background:var(--panel);border:1px solid var(--line);border-radius:12px;
padding:14px}.camera{width:100%;border-radius:8px;background:#05070a;
border:1px solid var(--line);display:block}.decision{font-size:18px;font-weight:700;
padding:12px;border-radius:8px;background:#0d151f;margin-bottom:10px}.actions{
display:grid;grid-template-columns:1fr 1fr;gap:8px;margin:12px 0}button{
border:1px solid #445b78;border-radius:8px;padding:12px;color:var(--text);
background:#223047;font-weight:700;cursor:pointer}button:disabled{opacity:.35;
cursor:not-allowed}.start{background:#195d3c}.stop{background:#6b3a20}.estop{
background:#7b2028}.reset{background:#2b3b51}.cards{display:grid;
grid-template-columns:1fr 1fr;gap:9px}.card{background:#0e1620;border-radius:8px;
padding:10px}.card h3{font-size:14px;margin:0 0 8px}.row{display:flex;
justify-content:space-between;gap:10px;font-size:13px;padding:3px 0}.value{
font-variant-numeric:tabular-nums;text-align:right}.blockers{margin:10px 0 0;
padding-left:21px;color:var(--amber);font-size:13px}.small{font-size:12px;
color:var(--muted);line-height:1.45}.wide{grid-column:1/-1}
</style></head><body>
<header><h1>두 로봇 협동 직진 시험</h1><span id="state" class="pill">연결 중…</span>
<span class="small">robot-1 Rear 카메라 · robot-2 Front 마커</span></header>
<main><section class="panel"><img id="camera" class="camera" alt="ArUco camera">
<p class="small">영상의 거리·좌우·각도 오버레이와 아래 판단값을 함께 확인하세요.</p>
</section><section class="panel"><div id="decision" class="decision">상태 수신 중…</div>
<div class="actions"><button id="arm" onclick="act('arm')">1. 시험 준비</button>
<button id="start" class="start" onclick="act('start')">2. 10 cm 시작</button>
<button id="stop" class="stop" onclick="act('stop')">정지</button>
<button id="reset" class="reset" onclick="act('reset')">제어권 해제</button>
<button id="estop" class="estop wide" onclick="emergency()">양쪽 비상정지</button></div>
<div class="cards"><div class="card"><h3>robot-2 · Front</h3><div id="front"></div></div>
<div class="card"><h3>robot-1 · Rear</h3><div id="rear"></div></div>
<div class="card wide"><h3>ArUco와 시험 판단</h3><div id="vision"></div></div></div>
<ul id="blockers" class="blockers"></ul><p class="small">웹 정지는 정상 정지용입니다.
위험하면 웹 화면보다 물리 E-STOP과 모터 전원 차단을 먼저 사용하세요. 비상정지는
STM32에 고정되므로 원인을 제거한 뒤 전원을 다시 인가해야 합니다.</p></section></main>
<script>
const esc=s=>String(s??'—').replace(/[&<>"']/g,c=>(
{'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const yn=v=>v?'<span class="ok">정상</span>':'<span class="bad">아님</span>';
const row=(k,v)=>`<div class="row"><span>${esc(k)}</span><span class="value">${v}</span></div>`;
const num=(v,d=1,u='')=>v==null?'—':Number(v).toFixed(d)+u;
async function act(name){await fetch('/api/'+name,{method:'POST'});setTimeout(tick,80)}
function emergency(){if(confirm('양쪽 STM32에 고정 비상정지를 전송할까요?'))act('estop')}
function robot(r){return row('하드웨어',yn(r.hardware_ready))+row('수동 제어권',yn(r.manual_active))+
row('odometry',yn(r.odom_fresh))+row('이동거리',num(r.progress_cm,1,' cm'))+
row('최근 상태',esc(r.hardware_status||'—'))}
async function tick(){let s;try{s=await(await fetch(
'/api/status',{cache:'no-store'})).json()}catch(e){return}
let st=document.getElementById('state');st.textContent=s.state_label;
st.className='pill '+(s.state==='RUNNING'||s.state==='ARMED'?'ok':
s.state==='FAULT'||s.state==='ESTOP'?'bad':'warn');
document.getElementById('decision').textContent=s.decision;
document.getElementById('front').innerHTML=robot(s.front);document.getElementById('rear').innerHTML=robot(s.rear);
let v=s.vision,m=s.motion;document.getElementById('vision').innerHTML=row('마커 현재',yn(v.visible))+
row('관측 신선함',yn(v.fresh))+row('전방 거리',num(v.forward_cm,1,' cm'))+
row('좌우',num(v.lateral_cm,1,' cm'))+row('상대 각도',num(v.yaw_deg,1,'°'))+
row('간격 변화',num(m.gap_error_cm,1,' cm'))+row('좌우 변화',num(m.lateral_drift_cm,1,' cm'))+
row('각도 변화',num(m.yaw_error_deg,1,'°'))+
row('명령 Front/Rear',num(m.front_command_mps,3)+' / '+
num(m.rear_command_mps,3)+' m/s');
document.getElementById('blockers').innerHTML=s.blockers.map(x=>'<li>'+esc(x)+'</li>').join('');
for(const k of ['arm','start','stop','reset','estop'])
document.getElementById(k).disabled=!s.actions[k];}
async function boot(){let c=await(await fetch('/api/config')).json();
document.getElementById('camera').src=location.protocol+'//'+
location.hostname+':'+c.preview_port+c.preview_path;
document.getElementById('start').textContent='2. '+
Math.round(c.distance_m*100)+' cm 시작';tick();setInterval(tick,250)}boot();
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


class CooperativeDriveTestNode(Node):
    CONTROL_STATES = {'ARMING', 'ARMED', 'RUNNING', 'COMPLETED',
                      'STOPPED', 'FAULT'}

    def __init__(self):
        super().__init__('cooperative_drive_test_node')
        self.declare_parameter('speed_mps', 0.0628)
        self.declare_parameter('distance_m', 0.10)
        self.declare_parameter('max_duration_s', 4.0)
        self.declare_parameter('hardware_timeout_s', 0.60)
        self.declare_parameter('manual_timeout_s', 0.60)
        self.declare_parameter('odom_timeout_s', 0.50)
        self.declare_parameter('marker_timeout_s', 0.35)
        self.declare_parameter('arm_timeout_s', 10.0)
        self.declare_parameter('min_marker_distance_m', 0.15)
        self.declare_parameter('max_marker_distance_m', 1.00)
        self.declare_parameter('initial_lateral_limit_m', 0.10)
        self.declare_parameter('initial_yaw_limit_deg', 15.0)
        self.declare_parameter('web_host', '0.0.0.0')
        self.declare_parameter('web_port', 5006)
        self.declare_parameter('preview_port', 5005)
        self.declare_parameter('preview_path', '/video/0')

        gp = self.get_parameter
        self.limits = DriveTestLimits(
            speed_mps=float(gp('speed_mps').value),
            distance_m=float(gp('distance_m').value),
            max_duration_s=float(gp('max_duration_s').value))
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
        if min(self.hardware_timeout, self.manual_timeout, self.odom_timeout,
               self.marker_timeout, self.arm_timeout) <= 0.0:
            raise ValueError('freshness and arm timeouts must be positive')
        if not 0.0 < self.min_marker_distance < self.max_marker_distance:
            raise ValueError('invalid marker distance gate')

        self.state = 'IDLE'
        self.decision = '정지 상태입니다. 시험 준비를 누르세요.'
        self.reason = ''
        self.estop = False
        self.arm_deadline = 0.0
        self.start_time = None
        self.start_odom = {'front': None, 'rear': None}
        self.start_relative = None
        self.last_commands = {'front': (0.0, 0.0, 0.0),
                              'rear': (0.0, 0.0, 0.0)}
        self.progress = {'front': 0.0, 'rear': 0.0}
        self.ready = {role: {'value': False, 'time': 0.0}
                      for role in ('front', 'rear')}
        self.manual = {role: {'value': False, 'time': 0.0}
                       for role in ('front', 'rear')}
        self.odom = {role: {'pose': None, 'time': 0.0}
                     for role in ('front', 'rear')}
        self.hardware_status = {'front': '', 'rear': ''}
        self.motor_status = {'front': '', 'rear': ''}
        self.relative = None
        self.relative_time = 0.0
        self.marker_visible = False
        self.marker_time = 0.0
        self.marker_true_time = 0.0
        self._last_enable_publish = 0.0
        self._requests = queue.SimpleQueue()
        self._stop_event = threading.Event()

        self.pub_manual_enable = {
            role: self.create_publisher(Bool, f'/{role}/manual_enable', 10)
            for role in ('front', 'rear')}
        self.pub_command = {
            role: self.create_publisher(
                TwistStamped, f'/{role}/manual_cmd_vel', 10)
            for role in ('front', 'rear')}
        self.pub_estop = self.create_publisher(Bool, '/emergency_stop', 10)
        self.pub_status = self.create_publisher(
            String, '/cooperative_test/status', 10)

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
                String, f'/{role}/hardware_status',
                lambda msg, r=role: self._text_cb(
                    self.hardware_status, r, msg), 10)
            self.create_subscription(
                String, f'/{role}/motor_diagnostics',
                lambda msg, r=role: self._text_cb(
                    self.motor_status, r, msg), 10)
        self.create_subscription(
            PoseStamped, '/sync/relative_pose', self._relative_cb,
            qos_profile_sensor_data)
        self.create_subscription(
            Bool, '/sync/marker_visible', self._marker_cb,
            qos_profile_sensor_data)
        self.create_subscription(Bool, '/emergency_stop', self._estop_cb, 10)

        self.create_timer(0.02, self._control_loop)
        self.create_timer(0.10, self._publish_status)
        self._start_web()
        self.get_logger().warn(
            f'협동 주행 시험 대시보드: http://{gp("web_host").value}:'
            f'{gp("web_port").value}/ (기본 정지, ARM 후 START 필요)')

    def _bool_cb(self, target, msg):
        target['value'] = bool(msg.data)
        target['time'] = time.monotonic()

    @staticmethod
    def _text_cb(target, role, msg):
        target[role] = str(msg.data)[:160]

    def _odom_cb(self, role, msg):
        try:
            pose = (float(msg.pose.pose.position.x),
                    float(msg.pose.pose.position.y),
                    _yaw_from_quaternion(msg.pose.pose.orientation))
        except (TypeError, ValueError):
            return
        if not all(math.isfinite(value) for value in pose):
            return
        self.odom[role] = {'pose': pose, 'time': time.monotonic()}

    def _relative_cb(self, msg):
        try:
            relative = (float(msg.pose.position.x),
                        float(msg.pose.position.y),
                        _yaw_from_quaternion(msg.pose.orientation))
        except (TypeError, ValueError):
            return
        if not all(math.isfinite(value) for value in relative):
            return
        self.relative = relative
        self.relative_time = time.monotonic()

    def _marker_cb(self, msg):
        now = time.monotonic()
        self.marker_visible = bool(msg.data)
        self.marker_time = now
        if self.marker_visible:
            self.marker_true_time = now

    def _estop_cb(self, msg):
        if msg.data:
            self.estop = True
            self.state = 'ESTOP'
            self.decision = '비상정지가 고정되었습니다. 원인 제거 후 전원을 재인가하세요.'
            self.reason = '비상정지 신호 감지'

    @staticmethod
    def _fresh(sample, timeout, now):
        return (bool(sample['value']) and sample['time'] > 0.0 and
                0.0 <= now - sample['time'] <= timeout)

    def _marker_fresh(self, now, require_current=False):
        pose_fresh = (self.relative is not None and self.relative_time > 0.0 and
                      0.0 <= now - self.relative_time <= self.marker_timeout)
        visible_fresh = (self.marker_true_time > 0.0 and
                         0.0 <= now - self.marker_true_time <=
                         self.marker_timeout)
        if require_current:
            visible_fresh = visible_fresh and self.marker_visible
        return pose_fresh and visible_fresh

    def _odom_fresh(self, role, now):
        sample = self.odom[role]
        return (sample['pose'] is not None and sample['time'] > 0.0 and
                0.0 <= now - sample['time'] <= self.odom_timeout)

    def _automatic_publishers(self):
        topics = []
        for role in ('front', 'rear'):
            topic = f'/{role}/cmd_vel'
            try:
                if self.get_publishers_info_by_topic(topic):
                    topics.append(topic)
            except Exception:
                # Discovery inspection is diagnostic. Other mandatory gates
                # still remain fail-closed if the graph API is unavailable.
                pass
        return topics

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
        if not self._marker_fresh(now, require_current=True):
            blockers.append('ID0 ArUco pose가 보이지 않거나 오래됨')
        elif self.relative is not None:
            forward, lateral, yaw = self.relative
            if not self.min_marker_distance <= forward <= self.max_marker_distance:
                blockers.append(
                    f'ArUco 거리 {forward * 100.0:.1f} cm가 시작 범위 밖')
            if abs(lateral) > self.initial_lateral_limit:
                blockers.append(
                    f'초기 좌우 오차 {lateral * 100.0:+.1f} cm가 너무 큼')
            if abs(yaw) > self.initial_yaw_limit:
                blockers.append(
                    f'초기 상대 각도 {math.degrees(yaw):+.1f}°가 너무 큼')
        publishers = self._automatic_publishers()
        if publishers:
            blockers.append('자동 cmd_vel 발행자 존재: ' + ', '.join(publishers))
        return blockers

    def _process_requests(self, now):
        while True:
            try:
                action = self._requests.get_nowait()
            except queue.Empty:
                return
            if action == 'arm':
                if self.estop:
                    self.decision = '비상정지가 고정되어 준비할 수 없습니다.'
                    continue
                self.state = 'ARMING'
                self.reason = ''
                self.arm_deadline = now + self.arm_timeout
                self.decision = '양쪽 STM32 수동 제어권을 확인하고 있습니다.'
            elif action == 'start':
                if self.state != 'ARMED':
                    self.decision = '먼저 시험 준비가 완료되어야 시작할 수 있습니다.'
                    continue
                blockers = self._blockers(now, require_manual=True)
                if blockers:
                    self._fault('시작 조건 상실: ' + blockers[0])
                    continue
                self.start_time = now
                self.start_odom = {
                    role: self.odom[role]['pose'] for role in ('front', 'rear')}
                self.start_relative = tuple(self.relative)
                self.progress = {'front': 0.0, 'rear': 0.0}
                self.state = 'RUNNING'
                self.reason = ''
                self.decision = '10 cm 협동 직진 중 · 조건을 50 Hz로 확인합니다.'
            elif action == 'stop':
                if self.state in self.CONTROL_STATES:
                    self._stop('STOPPED', '사용자가 웹에서 정지함')
            elif action == 'reset':
                if self.state != 'ESTOP':
                    self._publish_zero()
                    self.state = 'IDLE'
                    self.reason = ''
                    self.decision = '제어권을 해제했습니다. 다시 준비할 수 있습니다.'
            elif action == 'estop':
                self._publish_zero()
                self.pub_estop.publish(Bool(data=True))
                self.estop = True
                self.state = 'ESTOP'
                self.reason = '사용자가 웹 비상정지를 누름'
                self.decision = '양쪽 STM32 비상정지가 고정되었습니다.'

    def _publish_enable(self, enabled):
        msg = Bool(data=bool(enabled))
        for publisher in self.pub_manual_enable.values():
            publisher.publish(msg)

    def _publish_command(self, role, command):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f'{role}_base'
        (msg.twist.linear.x, msg.twist.linear.y,
         msg.twist.angular.z) = command
        self.pub_command[role].publish(msg)
        self.last_commands[role] = tuple(command)

    def _publish_zero(self):
        for role in ('front', 'rear'):
            self._publish_command(role, (0.0, 0.0, 0.0))

    def _stop(self, state, reason):
        self._publish_zero()
        self.state = state
        self.reason = str(reason)
        self.decision = _STATE_LABELS[state] + ': ' + str(reason)

    def _fault(self, reason):
        self._stop('FAULT', reason)
        self.get_logger().error(reason)

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
                self.state = 'ARMED'
                self.decision = '준비 완료. 주변을 확인하고 시작을 누르세요.'
            elif now >= self.arm_deadline:
                self._fault('준비 시간 초과: ' + blockers[0])
            else:
                self.decision = '준비 중: ' + blockers[0]
            return

        if self.state == 'ARMED':
            self._publish_zero()
            blockers = self._blockers(now, require_manual=True)
            if blockers:
                self._fault('대기 중 조건 상실: ' + blockers[0])
            return

        if self.state != 'RUNNING':
            if owns_control:
                self._publish_zero()
            return

        if self.start_time is None or self.start_relative is None:
            self._fault('시험 시작 기준값이 없음')
            return
        for role in ('front', 'rear'):
            if self.start_odom[role] is not None and self.odom[role]['pose'] is not None:
                self.progress[role] = odom_progress(
                    self.start_odom[role], self.odom[role]['pose'])
        forward, lateral, yaw = self.relative or self.start_relative
        gap_error = forward - self.start_relative[0]
        lateral_drift = lateral - self.start_relative[1]
        yaw_error = angle_norm(yaw - self.start_relative[2])
        hardware_ok = all(self._fresh(
            self.ready[role], self.hardware_timeout, now)
            for role in ('front', 'rear'))
        manual_ok = all(self._fresh(
            self.manual[role], self.manual_timeout, now)
            for role in ('front', 'rear'))
        odom_ok = all(self._odom_fresh(role, now)
                      for role in ('front', 'rear'))
        decision = evaluate_running(
            self.limits,
            elapsed_s=now - self.start_time,
            front_progress_m=self.progress['front'],
            rear_progress_m=self.progress['rear'],
            gap_error_m=gap_error,
            lateral_drift_m=lateral_drift,
            yaw_error_rad=yaw_error,
            hardware_ok=hardware_ok,
            manual_ok=manual_ok,
            marker_ok=self._marker_fresh(now),
            odom_ok=odom_ok,
            estop=self.estop)
        if decision.outcome == 'COMPLETED':
            self._stop('COMPLETED', decision.reason)
            return
        if decision.outcome == 'ESTOP':
            self._stop('ESTOP', decision.reason)
            return
        if decision.outcome == 'FAULT':
            self._fault(decision.reason)
            return
        front_cmd, rear_cmd = pair_commands(
            self.limits, gap_error, yaw_error)
        self._publish_command('front', front_cmd)
        self._publish_command('rear', rear_cmd)

    def _status_payload(self):
        now = time.monotonic()
        blockers = self._blockers(
            now, require_manual=self.state in self.CONTROL_STATES)
        relative = self.relative
        start = self.start_relative
        gap_error = None if relative is None or start is None else relative[0] - start[0]
        lateral = None if relative is None or start is None else relative[1] - start[1]
        yaw_error = None if relative is None or start is None else angle_norm(
            relative[2] - start[2])

        def robot(role):
            return {
                'hardware_ready': self._fresh(
                    self.ready[role], self.hardware_timeout, now),
                'manual_active': self._fresh(
                    self.manual[role], self.manual_timeout, now),
                'odom_fresh': self._odom_fresh(role, now),
                'progress_cm': self.progress[role] * 100.0,
                'hardware_status': self.hardware_status[role],
                'motor_status': self.motor_status[role],
            }

        return {
            'state': self.state,
            'state_label': _STATE_LABELS[self.state],
            'decision': self.decision,
            'reason': self.reason,
            'blockers': blockers,
            'front': robot('front'),
            'rear': robot('rear'),
            'vision': {
                'visible': self.marker_visible,
                'fresh': self._marker_fresh(now),
                'forward_cm': None if relative is None else relative[0] * 100.0,
                'lateral_cm': None if relative is None else relative[1] * 100.0,
                'yaw_deg': None if relative is None else math.degrees(relative[2]),
            },
            'motion': {
                'gap_error_cm': None if gap_error is None else gap_error * 100.0,
                'lateral_drift_cm': None if lateral is None else lateral * 100.0,
                'yaw_error_deg': None if yaw_error is None else math.degrees(yaw_error),
                'front_command_mps': self.last_commands['front'][0],
                'rear_command_mps': self.last_commands['rear'][0],
            },
            'actions': {
                'arm': self.state != 'RUNNING' and not self.estop,
                'start': self.state == 'ARMED' and not blockers,
                'stop': self.state in {'ARMING', 'ARMED', 'RUNNING'},
                'reset': self.state in {'COMPLETED', 'STOPPED', 'FAULT', 'ARMED'},
                'estop': self.state != 'ESTOP',
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
            raise RuntimeError('Flask/Werkzeug가 없어 시험 대시보드를 시작할 수 없음')
        app = Flask('cooperative_drive_test')

        @app.get('/')
        def index():
            return _HTML

        @app.get('/api/status')
        def status():
            return jsonify(self._status_payload())

        @app.get('/api/config')
        def config():
            return jsonify({
                'distance_m': self.limits.distance_m,
                'speed_mps': self.limits.speed_mps,
                'preview_port': int(self.get_parameter('preview_port').value),
                'preview_path': str(self.get_parameter('preview_path').value),
            })

        for action in ('arm', 'start', 'stop', 'reset', 'estop'):
            app.add_url_rule(
                f'/api/{action}', endpoint=f'action_{action}',
                view_func=lambda a=action: self._queue_action(a),
                methods=['POST'])

        host = str(self.get_parameter('web_host').value)
        port = int(self.get_parameter('web_port').value)
        self._server = make_server(host, port, app, threaded=True)
        self._web_thread = threading.Thread(
            target=self._server.serve_forever,
            name='cooperative-drive-test-web', daemon=True)
        self._web_thread.start()

    def _queue_action(self, action):
        self._requests.put(str(action))
        return jsonify({'accepted': True, 'action': action}), 202

    def destroy_node(self):
        self._publish_zero()
        self._publish_enable(False)
        self._stop_event.set()
        try:
            self._server.shutdown()
        except Exception:
            pass
        if getattr(self, '_web_thread', None) is not None:
            self._web_thread.join(timeout=2.0)
        return super().destroy_node()


def main(args=None):
    # The default rclpy SIGINT handler invalidates the context before
    # destroy_node(), which prevents the final zero/manual-off messages from
    # being published.  Let Python raise KeyboardInterrupt first, matching the
    # validated command-owner shutdown sequence.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = CooperativeDriveTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
