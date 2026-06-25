#!/usr/bin/env python3
"""
네비게이션 컨트롤러
===================
CCTV에서 빈 슬롯 좌표를 받아서
두 로봇을 동기화하여 목표로 이동시킵니다.

핵심: 가상 강체 모델 + 3중 안전장치
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
import math
import json
import time


class NavController(Node):
    def __init__(self):
        super().__init__('nav_controller')

        # 파라미터
        self.declare_parameter('wheelbase', 0.25)      # m
        self.declare_parameter('max_speed', 0.08)       # m/s
        self.declare_parameter('max_omega', 0.3)        # rad/s
        self.declare_parameter('arrival_dist', 0.03)    # 3cm 도착 판정
        self.declare_parameter('arrival_angle', 0.1)    # ~6도

        self.wheelbase = self.get_parameter('wheelbase').value
        self.max_speed = self.get_parameter('max_speed').value
        self.max_omega = self.get_parameter('max_omega').value
        self.arrival_dist = self.get_parameter('arrival_dist').value
        self.arrival_angle = self.get_parameter('arrival_angle').value
        self.half_L = self.wheelbase / 2.0

        # ---- 상태 ----
        self.state = 'IDLE'  # IDLE → MOVING → ARRIVED
        self.target = None    # (x, y, theta)

        self.front = {'x': 0.0, 'y': 0.0, 'theta': 0.0, 'time': 0.0}
        self.rear = {'x': 0.0, 'y': 0.0, 'theta': 0.0, 'time': 0.0}

        # 오차 보정
        self.error_level = 0
        self.speed_scale = 1.0

        # ---- 구독 ----
        self.sub_target = self.create_subscription(
            PoseStamped, '/parking/target_pose',
            self.target_cb, 10)
        self.sub_front = self.create_subscription(
            Odometry, '/front/odom', self.front_odom_cb, 10)
        self.sub_rear = self.create_subscription(
            Odometry, '/rear/odom', self.rear_odom_cb, 10)

        # ---- 발행 ----
        self.pub_front_cmd = self.create_publisher(
            Twist, '/front/cmd_vel', 10)
        self.pub_rear_cmd = self.create_publisher(
            Twist, '/rear/cmd_vel', 10)
        self.pub_estop = self.create_publisher(
            Bool, '/emergency_stop', 10)
        self.pub_status = self.create_publisher(
            String, '/nav/status', 10)

        # ---- 제어 루프 50Hz ----
        self.timer = self.create_timer(0.02, self.control_loop)
        self.log_timer = self.create_timer(1.0, self.log_status)

        self.get_logger().info('네비게이션 컨트롤러 시작 — 목표 대기 중')

    # ===== 콜백 =====

    def target_cb(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        q = msg.pose.orientation
        theta = math.atan2(2*(q.w*q.z + q.x*q.y),
                           1 - 2*(q.y*q.y + q.z*q.z))
        self.target = (x, y, theta)
        self.state = 'MOVING'
        self.get_logger().info(
            f'[목표] ({x:.3f}, {y:.3f}) θ={math.degrees(theta):.0f}°')

    def front_odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.front['x'] = p.x
        self.front['y'] = p.y
        self.front['theta'] = math.atan2(2*(q.w*q.z), 1 - 2*q.z*q.z)
        self.front['time'] = time.time()

    def rear_odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.rear['x'] = p.x
        self.rear['y'] = p.y
        self.rear['theta'] = math.atan2(2*(q.w*q.z), 1 - 2*q.z*q.z)
        self.rear['time'] = time.time()

    # ===== 가상 강체 중심 =====

    def get_virtual_pose(self):
        cx = (self.front['x'] + self.rear['x']) / 2
        cy = (self.front['y'] + self.rear['y']) / 2
        dx = self.front['x'] - self.rear['x']
        dy = self.front['y'] - self.rear['y']
        theta = math.atan2(dy, dx)
        return cx, cy, theta

    # ===== 기구학: 중심 속도 → 앞/뒤 분배 =====

    def split_velocity(self, vx, vy, omega):
        vy_f_extra = omega * self.half_L
        vy_r_extra = -omega * self.half_L
        front_vel = (vx, vy + vy_f_extra, omega)
        rear_vel = (vx, vy + vy_r_extra, omega)
        return front_vel, rear_vel

    # ===== 오차 감지 + 보정 =====

    def check_sync_error(self):
        dx = self.front['x'] - self.rear['x']
        dy = self.front['y'] - self.rear['y']
        actual_dist = math.sqrt(dx*dx + dy*dy)
        dist_err = abs(actual_dist - self.wheelbase)

        if dist_err < 0.003:       # 3mm 미만
            self.error_level = 0
            self.speed_scale = 1.0
        elif dist_err < 0.010:     # 10mm 미만
            self.error_level = 1
            self.speed_scale = 0.7
        elif dist_err < 0.020:     # 20mm 미만
            self.error_level = 2
            self.speed_scale = 0.3
        else:                      # 20mm 초과
            self.error_level = 3
            self.speed_scale = 0.0

        return dist_err

    def compensate_rear(self, rear_vx, rear_vy, rear_omega):
        if self.error_level >= 3:
            return 0.0, 0.0, 0.0

        # 뒤 로봇이 있어야 할 위치
        exp_x = self.front['x'] - self.wheelbase * math.cos(self.front['theta'])
        exp_y = self.front['y'] - self.wheelbase * math.sin(self.front['theta'])

        err_x = exp_x - self.rear['x']
        err_y = exp_y - self.rear['y']

        # 로컬 좌표로 변환
        ct = math.cos(self.rear['theta'])
        st = math.sin(self.rear['theta'])
        local_ex = err_x * ct + err_y * st
        local_ey = -err_x * st + err_y * ct

        Kp = [0.0, 0.5, 1.5, 0.0][self.error_level]

        comp_vx = (rear_vx + Kp * local_ex) * self.speed_scale
        comp_vy = (rear_vy + Kp * local_ey) * self.speed_scale
        comp_w = rear_omega * self.speed_scale

        return comp_vx, comp_vy, comp_w

    # ===== 메인 제어 루프 =====

    def control_loop(self):
        # 워치독: 오도메트리 수신 체크
        now = time.time()
        if now - self.front['time'] > 0.5 or now - self.rear['time'] > 0.5:
            if self.state == 'MOVING':
                self.send_stop()
            return

        if self.state != 'MOVING' or self.target is None:
            return

        # 가상 강체 현재 위치
        cx, cy, ct = self.get_virtual_pose()
        tx, ty, tt = self.target

        # 도착 판정
        dist = math.sqrt((tx - cx)**2 + (ty - cy)**2)
        if dist < self.arrival_dist:
            self.state = 'ARRIVED'
            self.send_stop()
            self.get_logger().info(
                f'[도착!] 오차: {dist*100:.1f}cm')
            return

        # Step 1: 가상 강체 목표 속도
        cos_c = math.cos(ct)
        sin_c = math.sin(ct)
        dx = tx - cx
        dy = ty - cy
        local_dx = dx * cos_c + dy * sin_c
        local_dy = -dx * sin_c + dy * cos_c
        dtheta = math.atan2(math.sin(tt - ct), math.cos(tt - ct))

        Kp_lin = 0.8
        Kp_ang = 1.0
        vx = max(-self.max_speed, min(self.max_speed, Kp_lin * local_dx))
        vy = max(-self.max_speed, min(self.max_speed, Kp_lin * local_dy))
        omega = max(-self.max_omega, min(self.max_omega, Kp_ang * dtheta))

        # 도착 근처 감속
        if dist < 0.08:
            scale = dist / 0.08
            vx *= scale
            vy *= scale

        # Step 2: 기구학 분배
        front_vel, rear_vel = self.split_velocity(vx, vy, omega)

        # Step 3: 오차 감지 + 보정
        sync_err = self.check_sync_error()
        rear_comp = self.compensate_rear(*rear_vel)

        # 앞 로봇도 감속
        fv = front_vel
        s = self.speed_scale
        front_scaled = (fv[0]*s, fv[1]*s, fv[2]*s)

        # Step 4: 비상 정지 체크
        if self.error_level >= 3:
            self.send_stop()
            estop = Bool()
            estop.data = True
            self.pub_estop.publish(estop)
            self.get_logger().error(f'비상정지! 오차: {sync_err*1000:.0f}mm')
            return

        # Step 5: 명령 발행
        fc = Twist()
        fc.linear.x = front_scaled[0]
        fc.linear.y = front_scaled[1]
        fc.angular.z = front_scaled[2]
        self.pub_front_cmd.publish(fc)

        rc = Twist()
        rc.linear.x = rear_comp[0]
        rc.linear.y = rear_comp[1]
        rc.angular.z = rear_comp[2]
        self.pub_rear_cmd.publish(rc)

    def send_stop(self):
        stop = Twist()
        self.pub_front_cmd.publish(stop)
        self.pub_rear_cmd.publish(stop)

    def log_status(self):
        cx, cy, ct = self.get_virtual_pose()
        dist = 0.0
        if self.target:
            tx, ty, _ = self.target
            dist = math.sqrt((tx-cx)**2 + (ty-cy)**2)

        level_names = ['OK', 'WARNING', 'DANGER', 'E_STOP']
        status = {
            'state': self.state,
            'virtual_pose': {
                'x': round(cx, 3), 'y': round(cy, 3),
                'theta_deg': round(math.degrees(ct), 1)},
            'dist_to_target_cm': round(dist * 100, 1),
            'sync_level': level_names[self.error_level],
            'speed_scale': self.speed_scale,
        }
        msg = String()
        msg.data = json.dumps(status)
        self.pub_status.publish(msg)

        if self.state == 'MOVING':
            self.get_logger().info(
                f'[{self.state}] target: {dist*100:.1f}cm | '
                f'sync: {level_names[self.error_level]} | '
                f'speed: {self.speed_scale*100:.0f}%')


def main(args=None):
    rclpy.init(args=args)
    node = NavController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
