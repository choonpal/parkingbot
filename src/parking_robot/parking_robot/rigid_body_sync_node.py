#!/usr/bin/env python3
"""
==================================================
rigid_body_sync_node.py [2-3]
==================================================
Nav2와 두 로봇 동기화를 잇는 핵심 다리.

흐름:
  Nav2가 "가상 로봇 1대"라고 생각하고 /cmd_vel을 발행
    ↓
  이 노드가 그 cmd_vel을 받아서
    ↓
  강체 기구학으로 front/rear 속도로 분배
    ↓
  ArUco + 칼만으로 거리 오차 보정
    ↓
  /front/cmd_vel, /rear/cmd_vel 발행

핵심: Nav2는 두 로봇의 존재를 모름. 가상 중심점 하나만 제어.
      두 로봇으로 나누는 건 전적으로 이 노드 담당.

구독:
  /cmd_vel               (Nav2가 주는 가상 로봇 속도)
  /front/odom, /rear/odom (각 로봇 위치)
  /sync/aruco_relative   (ArUco 상대 위치, 보조)
  /sync/aruco_visible    (마커 가시성)

발행:
  /front/cmd_vel, /rear/cmd_vel  (분배된 속도)
  /virtual_robot/odom    (가상 중심점 위치 → Nav2가 구독)
  /sync/status           (모니터링)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
import math
import json
import time


# ==================================================
# PID
# ==================================================
class PID:
    def __init__(self, Kp, Ki, Kd, limit=0.04):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.integral = 0.0
        self.prev = 0.0
        self.limit = limit

    def compute(self, error, dt):
        p = self.Kp * error
        self.integral += error * dt
        self.integral = max(-1, min(1, self.integral))
        i = self.Ki * self.integral
        d = self.Kd * (error - self.prev) / dt if dt > 0 else 0
        self.prev = error
        return max(-self.limit, min(self.limit, p + i + d))

    def reset(self):
        self.integral = 0.0
        self.prev = 0.0


# ==================================================
# 거리 칼만 필터 (엔코더 + ArUco)
# ==================================================
class DistanceKalman:
    def __init__(self, init=0.25):
        self.dist = init
        self.P = 0.1
        self.Q = 0.0001
        self.R = 0.0004

    def predict(self, enc_dist):
        self.dist = enc_dist
        self.P += self.Q

    def update(self, aruco_dist):
        K = self.P / (self.P + self.R)
        self.dist += K * (aruco_dist - self.dist)
        self.P = (1 - K) * self.P
        return K


# ==================================================
# 메인 분배 노드
# ==================================================
class RigidBodySyncNode(Node):
    def __init__(self):
        super().__init__('rigid_body_sync_node')

        # ===== 파라미터 =====
        self.declare_parameter('wheelbase', 0.25)
        self.declare_parameter('max_speed', 0.08)
        self.declare_parameter('max_omega', 0.3)

        self.wheelbase = self.get_parameter('wheelbase').value
        self.half_L = self.wheelbase / 2
        self.max_speed = self.get_parameter('max_speed').value
        self.max_omega = self.get_parameter('max_omega').value

        # ===== 상태 =====
        self.cmd_vel = (0.0, 0.0, 0.0)   # Nav2가 준 가상 속도
        self.cmd_time = 0.0
        self.front = {'x': 0.0, 'y': 0.0, 'theta': 0.0, 't': 0.0}
        self.rear = {'x': 0.0, 'y': 0.0, 'theta': 0.0, 't': 0.0}

        # ArUco
        self.aruco_dist = None
        self.aruco_lateral = None
        self.aruco_visible = False
        self.aruco_time = 0.0

        self.front_ready = False
        self.rear_ready = False

        # 융합 도구
        self.dist_kalman = DistanceKalman(self.wheelbase)
        self.rear_pid = PID(1.2, 0.1, 0.05, limit=self.max_speed)
        self.lateral_pid = PID(1.0, 0.05, 0.03, limit=self.max_speed)

        # ===== 구독 =====
        # Nav2가 주는 가상 로봇 속도
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        # 각 로봇 위치
        self.create_subscription(Odometry, '/front/odom', self.front_cb, 10)
        self.create_subscription(Odometry, '/rear/odom', self.rear_cb, 10)
        # ArUco 보조
        self.create_subscription(PoseStamped, '/sync/relative_pose',
                                 self.aruco_cb, 10)
        self.create_subscription(Bool, '/sync/marker_visible',
                                 self.aruco_vis_cb, 10)

        # ===== 발행 =====
        self.pub_fc = self.create_publisher(Twist, '/front/cmd_vel', 10)
        self.pub_rc = self.create_publisher(Twist, '/rear/cmd_vel', 10)
        # 가상 로봇 odom → Nav2가 위치 추정에 사용
        self.pub_virtual = self.create_publisher(
            Odometry, '/virtual_robot/odom', 10)
        self.pub_status = self.create_publisher(String, '/sync/status', 10)

        # ===== 루프 =====
        self.create_timer(0.02, self.control_loop)   # 50Hz 분배
        self.create_timer(0.05, self.publish_virtual_odom)  # 20Hz 가상 odom
        self.create_timer(1.0, self.log_status)

        self.get_logger().info('rigid_body_sync_node 시작 [2-3]')

    # ================================================
    # 콜백
    # ================================================
    def cmd_vel_cb(self, msg):
        """Nav2가 주는 가상 로봇 속도"""
        self.cmd_vel = (msg.linear.x, msg.linear.y, msg.angular.z)
        self.cmd_time = time.time()

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
        self.aruco_dist = msg.pose.position.x
        self.aruco_lateral = msg.pose.position.y
        self.aruco_time = time.time()

    def aruco_vis_cb(self, msg):
        self.aruco_visible = msg.data

    # ================================================
    # 가상 강체 중심
    # ================================================
    def get_virtual_pose(self):
        cx = (self.front['x'] + self.rear['x']) / 2
        cy = (self.front['y'] + self.rear['y']) / 2
        dx = self.front['x'] - self.rear['x']
        dy = self.front['y'] - self.rear['y']
        theta = math.atan2(dy, dx)
        return cx, cy, theta

    def get_encoder_distance(self):
        dx = self.front['x'] - self.rear['x']
        dy = self.front['y'] - self.rear['y']
        return math.sqrt(dx*dx + dy*dy)

    # ================================================
    # 가상 로봇 odom 발행 (Nav2용)
    # ================================================
    def publish_virtual_odom(self):
        """
        두 로봇의 중심점을 '가상 로봇 한 대'로 만들어 발행.
        Nav2는 이 odom을 보고 위치를 추정하여 경로를 계획.
        """
        if not (self.front_ready and self.rear_ready):
            return

        cx, cy, ct = self.get_virtual_pose()

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.child_frame_id = 'virtual_base'
        msg.pose.pose.position.x = cx
        msg.pose.pose.position.y = cy
        msg.pose.pose.orientation.z = math.sin(ct/2)
        msg.pose.pose.orientation.w = math.cos(ct/2)
        self.pub_virtual.publish(msg)

    # ================================================
    # 메인 분배 루프
    # ================================================
    def control_loop(self):
        if not (self.front_ready and self.rear_ready):
            return

        now = time.time()
        # 워치독: odom 끊김
        if now - self.front['t'] > 0.5 or now - self.rear['t'] > 0.5:
            self.send_stop()
            return

        # Nav2 cmd_vel 끊김 → 정지 (목표 도달 또는 통신 끊김)
        if now - self.cmd_time > 0.3:
            self.send_stop()
            return

        # === Step 1: Nav2 속도 = 가상 중심점 목표 속도 ===
        vx, vy, omega = self.cmd_vel
        vx = max(-self.max_speed, min(self.max_speed, vx))
        vy = max(-self.max_speed, min(self.max_speed, vy))
        omega = max(-self.max_omega, min(self.max_omega, omega))

        # === Step 2: 강체 기구학 분배 ===
        # 회전 시 앞은 +ω×L/2, 뒤는 -ω×L/2 횡속도 보정
        front_vel = [vx, vy + omega * self.half_L, omega]
        rear_vel = [vx, vy - omega * self.half_L, omega]

        # === Step 3: 거리 융합 (엔코더 + ArUco) ===
        enc_dist = self.get_encoder_distance()
        self.dist_kalman.predict(enc_dist)

        aruco_fresh = (now - self.aruco_time) < 0.3
        if self.aruco_visible and aruco_fresh and self.aruco_dist:
            self.dist_kalman.update(self.aruco_dist)
            correction_src = 'ARUCO'
        else:
            correction_src = 'ENCODER'

        fused_dist = self.dist_kalman.dist

        # === Step 4: 거리 오차 보정 (PID) ===
        dist_error = fused_dist - self.wheelbase
        rear_corr = self.rear_pid.compute(dist_error, 0.02)
        rear_vel[0] += rear_corr  # 뒤 로봇 전후 보정

        # ArUco 좌우 정렬 보정 (마커 보일 때)
        if correction_src == 'ARUCO' and self.aruco_lateral is not None:
            lat_corr = self.lateral_pid.compute(self.aruco_lateral, 0.02)
            rear_vel[1] += lat_corr

        # === Step 5: 명령 발행 ===
        fc = Twist()
        fc.linear.x, fc.linear.y, fc.angular.z = front_vel
        self.pub_fc.publish(fc)

        rc = Twist()
        rc.linear.x, rc.linear.y, rc.angular.z = rear_vel
        self.pub_rc.publish(rc)

        self._info = {
            'nav_cmd': [round(vx, 3), round(vy, 3), round(omega, 3)],
            'enc_dist': round(enc_dist*100, 1),
            'fused_dist': round(fused_dist*100, 1),
            'dist_err_mm': round(dist_error*1000, 1),
            'correction': correction_src,
        }

    def send_stop(self):
        s = Twist()
        self.pub_fc.publish(s)
        self.pub_rc.publish(s)
        self.rear_pid.reset()
        self.lateral_pid.reset()

    def log_status(self):
        info = getattr(self, '_info', {})
        info['aruco_visible'] = self.aruco_visible
        m = String()
        m.data = json.dumps(info)
        self.pub_status.publish(m)
        if info.get('nav_cmd') and any(info['nav_cmd']):
            self.get_logger().info(
                f"Nav2:{info['nav_cmd']} | "
                f"거리 엔코더:{info.get('enc_dist')}cm "
                f"융합:{info.get('fused_dist')}cm | "
                f"{info.get('correction')}")


def main(args=None):
    rclpy.init(args=args)
    node = RigidBodySyncNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
