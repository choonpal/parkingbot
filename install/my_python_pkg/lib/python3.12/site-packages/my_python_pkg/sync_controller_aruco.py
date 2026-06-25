#!/usr/bin/env python3
"""
==================================================
sync_controller_aruco.py
==================================================
강체 기구학(엔코더) + ArUco 마커 보정 통합 동기 제어

구조:
  [주력] 엔코더 강체 기구학 (50Hz) → 빠른 실시간 속도 분배
  [보정] ArUco 상대 위치 (10Hz)   → 드리프트 보정
  [융합] 칼만 필터로 두 소스 결합

마커가 안 보일 때:
  → 엔코더 예측만으로 계속 주행 (fallback)
마커가 보일 때:
  → ArUco 측정으로 두 로봇 간 실제 거리 보정

구독:
  /front/odom, /rear/odom (엔코더)
  /sync/aruco_relative (ArUco 상대 위치)
  /sync/aruco_visible (마커 가시성)

발행:
  /front/cmd_vel, /rear/cmd_vel
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
import math
import json
import time
import numpy as np


# ==================================================
# PID 제어기 (앞서 학습한 것)
# ==================================================
class PID:
    def __init__(self, Kp, Ki, Kd, out_limit=0.05):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0.0
        self.prev_error = 0.0
        self.out_limit = out_limit

    def compute(self, error, dt):
        # P
        p = self.Kp * error
        # I (적분 와인드업 방지)
        self.integral += error * dt
        self.integral = max(-1.0, min(1.0, self.integral))
        i = self.Ki * self.integral
        # D
        d = self.Kd * (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error

        out = p + i + d
        return max(-self.out_limit, min(self.out_limit, out))

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0


# ==================================================
# 거리 칼만 필터 (엔코더 + ArUco 융합)
# ==================================================
class DistanceKalman:
    """
    두 로봇 간 거리를 추정하는 1D 칼만 필터.
    엔코더 추정 거리와 ArUco 측정 거리를 융합.
    """
    def __init__(self, initial_dist=0.25):
        self.dist = initial_dist   # 추정 거리
        self.P = 0.1               # 불확실성
        self.Q = 0.0001            # 프로세스 노이즈
        self.R_aruco = 0.0004      # ArUco 측정 노이즈 (정확, 2cm)

    def predict(self, encoder_dist):
        """엔코더 기반 예측"""
        self.dist = encoder_dist
        self.P += self.Q

    def update_aruco(self, measured_dist):
        """ArUco 측정으로 보정"""
        K = self.P / (self.P + self.R_aruco)
        self.dist = self.dist + K * (measured_dist - self.dist)
        self.P = (1 - K) * self.P
        return K


# ==================================================
# 메인 동기 컨트롤러
# ==================================================
class SyncControllerAruco(Node):
    def __init__(self):
        super().__init__('sync_controller_aruco')

        # 파라미터
        self.declare_parameter('wheelbase', 0.25)
        self.declare_parameter('max_speed', 0.08)
        self.declare_parameter('max_omega', 0.3)
        self.declare_parameter('arrival_dist', 0.03)

        self.wheelbase = self.get_parameter('wheelbase').value
        self.half_L = self.wheelbase / 2
        self.max_speed = self.get_parameter('max_speed').value
        self.max_omega = self.get_parameter('max_omega').value
        self.arrival_dist = self.get_parameter('arrival_dist').value

        # 상태
        self.state = 'IDLE'
        self.target = None
        self.front = {'x': 0.0, 'y': 0.0, 'theta': 0.0, 't': 0.0}
        self.rear = {'x': 0.0, 'y': 0.0, 'theta': 0.0, 't': 0.0}

        # ArUco
        self.aruco_dist = None       # 마커로 측정한 거리
        self.aruco_lateral = None    # 좌우 오프셋
        self.aruco_yaw = None
        self.aruco_visible = False
        self.aruco_time = 0.0

        # 융합 도구
        self.dist_kalman = DistanceKalman(self.wheelbase)
        self.rear_pid_x = PID(Kp=1.2, Ki=0.1, Kd=0.05, out_limit=self.max_speed)
        self.rear_pid_y = PID(Kp=1.2, Ki=0.1, Kd=0.05, out_limit=self.max_speed)

        self.front_ready = False
        self.rear_ready = False

        # 구독
        self.create_subscription(PoseStamped, '/parking/target_pose',
                                 self.target_cb, 10)
        self.create_subscription(Odometry, '/front/odom',
                                 self.front_cb, 10)
        self.create_subscription(Odometry, '/rear/odom',
                                 self.rear_cb, 10)
        self.create_subscription(PoseStamped, '/sync/aruco_relative',
                                 self.aruco_cb, 10)
        self.create_subscription(Bool, '/sync/aruco_visible',
                                 self.aruco_vis_cb, 10)

        # 발행
        self.pub_fc = self.create_publisher(Twist, '/front/cmd_vel', 10)
        self.pub_rc = self.create_publisher(Twist, '/rear/cmd_vel', 10)
        self.pub_status = self.create_publisher(String, '/sync/status', 10)

        # 제어 루프 50Hz
        self.create_timer(0.02, self.control_loop)
        self.create_timer(1.0, self.log_status)

        self.get_logger().info('ArUco 융합 동기 컨트롤러 시작')

    # ===== 콜백 =====
    def target_cb(self, msg):
        x, y = msg.pose.position.x, msg.pose.position.y
        q = msg.pose.orientation
        t = math.atan2(2*(q.w*q.z), 1-2*q.z*q.z)
        self.target = (x, y, t)
        if self.front_ready and self.rear_ready:
            self.state = 'MOVING'
        self.get_logger().info(f'목표: ({x:.3f}, {y:.3f})')

    def front_cb(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.front = {'x': p.x, 'y': p.y,
                      'theta': math.atan2(2*q.w*q.z, 1-2*q.z*q.z),
                      't': time.time()}
        self.front_ready = True

    def rear_cb(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.rear = {'x': p.x, 'y': p.y,
                     'theta': math.atan2(2*q.w*q.z, 1-2*q.z*q.z),
                     't': time.time()}
        self.rear_ready = True

    def aruco_cb(self, msg):
        """ArUco 상대 위치 수신"""
        self.aruco_dist = msg.pose.position.x      # 전방 거리
        self.aruco_lateral = msg.pose.position.y   # 좌우
        q = msg.pose.orientation
        self.aruco_yaw = math.atan2(2*q.w*q.z, 1-2*q.z*q.z)
        self.aruco_time = time.time()

    def aruco_vis_cb(self, msg):
        self.aruco_visible = msg.data

    # ===== 가상 강체 중심 =====
    def get_virtual_pose(self):
        cx = (self.front['x'] + self.rear['x']) / 2
        cy = (self.front['y'] + self.rear['y']) / 2
        dx = self.front['x'] - self.rear['x']
        dy = self.front['y'] - self.rear['y']
        return cx, cy, math.atan2(dy, dx)

    def get_encoder_distance(self):
        """엔코더로 추정한 두 로봇 간 거리"""
        dx = self.front['x'] - self.rear['x']
        dy = self.front['y'] - self.rear['y']
        return math.sqrt(dx*dx + dy*dy)

    # ===== 메인 제어 루프 =====
    def control_loop(self):
        if not self.front_ready or not self.rear_ready:
            return
        now = time.time()
        if now - self.front['t'] > 0.5 or now - self.rear['t'] > 0.5:
            self.send_stop()
            return
        if self.state != 'MOVING' or self.target is None:
            return

        # === Step 1: 거리 융합 (엔코더 + ArUco) ===
        encoder_dist = self.get_encoder_distance()
        self.dist_kalman.predict(encoder_dist)

        aruco_fresh = (now - self.aruco_time) < 0.3  # 0.3초 이내 마커
        if self.aruco_visible and aruco_fresh and self.aruco_dist:
            # ArUco가 측정한 거리로 보정
            K = self.dist_kalman.update_aruco(self.aruco_dist)
            correction_active = True
        else:
            # 마커 안 보임 → 엔코더만 (fallback)
            correction_active = False

        fused_dist = self.dist_kalman.dist

        # === Step 2: 가상 강체 위치 + 목표 속도 ===
        cx, cy, ct = self.get_virtual_pose()
        tx, ty, tt = self.target
        dist_to_target = math.sqrt((tx-cx)**2 + (ty-cy)**2)

        if dist_to_target < self.arrival_dist:
            self.state = 'ARRIVED'
            self.send_stop()
            self.get_logger().info(f'도착! 오차 {dist_to_target*100:.1f}cm')
            return

        cc, ss = math.cos(ct), math.sin(ct)
        ldx = (tx-cx)*cc + (ty-cy)*ss
        ldy = -(tx-cx)*ss + (ty-cy)*cc
        dth = math.atan2(math.sin(tt-ct), math.cos(tt-ct))

        vx = max(-self.max_speed, min(self.max_speed, 0.8*ldx))
        vy = max(-self.max_speed, min(self.max_speed, 0.8*ldy))
        om = max(-self.max_omega, min(self.max_omega, 1.0*dth))
        if dist_to_target < 0.08:
            s = dist_to_target / 0.08
            vx *= s; vy *= s

        # === Step 3: 강체 기구학 분배 ===
        front_vel = (vx, vy + om*self.half_L, om)
        rear_vel = (vx, vy - om*self.half_L, om)

        # === Step 4: 거리 오차 보정 (PID) ===
        # 융합된 거리가 휠베이스와 다르면 보정
        dist_error = fused_dist - self.wheelbase
        dt = 0.02

        # 거리가 멀어지면(+) 뒤 로봇을 앞으로 당김
        rear_correction = self.rear_pid_x.compute(dist_error, dt)

        # ArUco 좌우 오프셋 보정 (마커 보일 때만)
        lateral_correction = 0.0
        if correction_active and self.aruco_lateral is not None:
            # 슬레이브가 좌우로 치우치면 보정
            target_lateral = 0.0  # 정렬되어야 할 위치
            lateral_error = self.aruco_lateral - target_lateral
            lateral_correction = self.rear_pid_y.compute(lateral_error, dt)

        # === Step 5: 명령 발행 ===
        fc = Twist()
        fc.linear.x = front_vel[0]
        fc.linear.y = front_vel[1]
        fc.angular.z = front_vel[2]
        self.pub_fc.publish(fc)

        rc = Twist()
        rc.linear.x = rear_vel[0] + rear_correction
        rc.linear.y = rear_vel[1] + lateral_correction
        rc.angular.z = rear_vel[2]
        self.pub_rc.publish(rc)

        self._last_info = {
            'encoder_dist': round(encoder_dist*100, 1),
            'aruco_dist': round(self.aruco_dist*100, 1) if self.aruco_dist else None,
            'fused_dist': round(fused_dist*100, 1),
            'correction': 'ARUCO' if correction_active else 'ENCODER_ONLY',
            'dist_error_mm': round(dist_error*1000, 1),
        }

    def send_stop(self):
        s = Twist()
        self.pub_fc.publish(s)
        self.pub_rc.publish(s)
        self.rear_pid_x.reset()
        self.rear_pid_y.reset()

    def log_status(self):
        info = getattr(self, '_last_info', {})
        info['state'] = self.state
        info['aruco_visible'] = self.aruco_visible
        m = String()
        m.data = json.dumps(info)
        self.pub_status.publish(m)
        if self.state == 'MOVING':
            self.get_logger().info(
                f"엔코더:{info.get('encoder_dist')}cm | "
                f"ArUco:{info.get('aruco_dist')}cm | "
                f"융합:{info.get('fused_dist')}cm | "
                f"{info.get('correction')}")


def main(args=None):
    rclpy.init(args=args)
    node = SyncControllerAruco()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
