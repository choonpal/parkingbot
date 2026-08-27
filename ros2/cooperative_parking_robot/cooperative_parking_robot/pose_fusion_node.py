#!/usr/bin/env python3
"""
==================================================
pose_fusion_node.py (role: front/rear)
==================================================
로봇 1대의 절대 world-frame pose(x,y,yaw)를 만드는 노드.

배경: stm32_bridge_node가 내는 /{role}/wheel_odom은 엔코더만으로
쌓은 순수 dead-reckoning이라 반드시 드리프트한다(특히 yaw). 지금까지는
이 드리프트를 보정할 절대 기준이 없었다 — rigid_body_sync_node의
ArUco+Kalman은 Front-Rear "상대" 오차만 보고, CCTV 절대보정(α=0.3)은
가상중심 위치에만 적용돼 개별 로봇 yaw는 무보정 상태로 흘러갔다.

이 노드는 Front와 Rear 상판 마커에서 얻은 각 역할의 절대
(x,y,yaw)를 PoseEKF로 융합한다. 두 상판 마커가 가려진 구간의 상대 보정은
rigid_body_sync_node가 Front 후면 ID0 관측으로 수행한다.
rigid_body_sync_node는 코드 변경 없이 그대로
/front/odom, /rear/odom을 구독하면 된다 — 이 노드가 stm32_bridge 대신
그 토픽의 최종 발행자가 된다.

입력:
  /{role}/wheel_odom (nav_msgs/Odometry)
      — stm32_bridge_node 발행. pose 필드는 참고용 dead-reckoning 누적치.
        twist.linear.x/y = 이번 구간 body-frame 변위(dx_body,dy_body) [m]
        twist.angular.z  = 이번 구간 yaw 변위(dtheta) [rad]
        (주의: 관례적 의미의 "속도"가 아니라 "변위"다 — 이 노드가 직접
        정의한 두 노드 간 내부 계약이다.)
  /{role}/cctv_pose (geometry_msgs/PoseStamped, frame_id='map')
      — CCTV가 해당 역할의 상판 ArUco를 읽어 계산한 절대 pose.
        cctv_robot_marker_node가 입력 이미지의 촬영시각을 보존해 발행한다.
  /{role}/cctv_marker_visible (std_msgs/Bool)

출력:
  /{role}/odom (nav_msgs/Odometry)              — 융합된 최종 pose
  /{role}/localization_status (std_msgs/String)  — 진단(JSON)
"""

import math
import json

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String

from cooperative_parking_robot.kalman_filter import PoseEKF
from cooperative_parking_robot.latest_qos import SENSOR_LATEST_QOS


def yaw_from_quat(q):
    values = (q.x, q.y, q.z, q.w)
    if not all(math.isfinite(v) for v in values):
        return None
    norm = math.sqrt(sum(v * v for v in values))
    if norm < 1e-9:
        return None
    x, y, z, w = (v / norm for v in values)
    return math.atan2(2.0 * (w * z + x * y),
                      1.0 - 2.0 * (y * y + z * z))


class PoseFusionNode(Node):
    def __init__(self):
        super().__init__('pose_fusion_node')

        # ===== 파라미터 =====
        self.declare_parameter('role', 'front')
        # 전원 인가 시 로봇의 알려진 초기 pose (도킹 홈 위치 기준. 실측 후 확정)
        self.declare_parameter('init_x', 0.0)
        self.declare_parameter('init_y', 0.0)
        self.declare_parameter('init_yaw', 0.0)
        # CCTV 측정 신선도 — rigid_body_sync의 odom_timeout(0.5s)과 동일 기준
        self.declare_parameter('cctv_timeout', 0.5)
        self.declare_parameter('max_future_skew', 0.1)
        self.declare_parameter('max_predict_dt', 0.2)  # 비정상 큰 dt 방지
        self.declare_parameter('default_predict_dt', 0.02)
        # 프로세스 노이즈 (엔코더 슬립 비례) — placeholder, 실측 후 튜닝
        self.declare_parameter('pos_process_gain', 0.05)
        self.declare_parameter('yaw_process_gain', 0.05)
        self.declare_parameter('pos_process_floor', 0.0005)
        self.declare_parameter('yaw_process_floor', 0.001)
        # 측정 노이즈 (CCTV homography + ArUco pose 재현오차) — placeholder
        self.declare_parameter('meas_sigma_xy_m', 0.02)
        self.declare_parameter('meas_sigma_yaw_deg', 3.0)
        self.declare_parameter('gate_chi2', 11.34)
        self.declare_parameter('max_reject_streak', 5)
        self.declare_parameter('max_reacquire_pos_error_m', 0.5)
        self.declare_parameter('max_reacquire_yaw_error_deg', 45.0)

        self.role = self.get_parameter('role').value
        if self.role not in ('front', 'rear'):
            raise ValueError("role must be 'front' or 'rear'")

        self.ekf = PoseEKF(
            x=self.get_parameter('init_x').value,
            y=self.get_parameter('init_y').value,
            yaw=self.get_parameter('init_yaw').value,
            pos_process_gain=self.get_parameter('pos_process_gain').value,
            yaw_process_gain=self.get_parameter('yaw_process_gain').value,
            pos_process_floor=self.get_parameter('pos_process_floor').value,
            yaw_process_floor=self.get_parameter('yaw_process_floor').value,
            gate_chi2=self.get_parameter('gate_chi2').value,
            max_reject_streak=self.get_parameter('max_reject_streak').value,
            max_reacquire_pos_error=self.get_parameter(
                'max_reacquire_pos_error_m').value,
            max_reacquire_yaw_error=math.radians(self.get_parameter(
                'max_reacquire_yaw_error_deg').value),
        )
        sxy = self.get_parameter('meas_sigma_xy_m').value
        syaw = math.radians(self.get_parameter('meas_sigma_yaw_deg').value)
        self.R = [[sxy ** 2, 0.0, 0.0],
                  [0.0, sxy ** 2, 0.0],
                  [0.0, 0.0, syaw ** 2]]

        self.cctv_timeout = self.get_parameter('cctv_timeout').value
        self.max_future_skew = self.get_parameter('max_future_skew').value
        self.max_dt = self.get_parameter('max_predict_dt').value
        self.default_dt = self.get_parameter('default_predict_dt').value

        self._last_wheel_stamp = None
        self._last_cctv_stamp = None
        self._cctv_visible = False
        self._last_source = 'INIT'
        self._last_cctv_age = None

        # ===== 구독 =====
        self.create_subscription(
            Odometry, f'/{self.role}/wheel_odom', self.wheel_odom_cb,
            SENSOR_LATEST_QOS)
        self.create_subscription(
            PoseStamped, f'/{self.role}/cctv_pose', self.cctv_pose_cb,
            SENSOR_LATEST_QOS)
        self.create_subscription(
            Bool, f'/{self.role}/cctv_marker_visible', self.cctv_visible_cb,
            SENSOR_LATEST_QOS)

        # A superseded 50 Hz pose must never queue behind as much as 0.4 s of
        # reliable history. Every control consumer uses this depth-1 contract.
        self.pub_odom = self.create_publisher(
            Odometry, f'/{self.role}/odom', SENSOR_LATEST_QOS)
        self.pub_status = self.create_publisher(
            String, f'/{self.role}/localization_status', 10)

        self.create_timer(0.2, self.publish_status)  # 5Hz 진단

        self.get_logger().info(f'pose_fusion_node 시작 [{self.role}]')

    # ===== 콜백 =====
    def wheel_odom_cb(self, msg):
        """엔코더 델타 도착 → predict만 수행.

        v1.7 수정: 이전엔 이 콜백(엔코더 도착률, 최대 수십Hz)마다 캐시된
        CCTV pose로 correct()를 매번 다시 호출했다. cctv_timeout(0.5s) 동안
        같은 CCTV 프레임 하나가 최대 수십 번 "새 독립측정"인 것처럼 반영되어
        공분산이 허위로 빠르게 줄어들고, reject-streak 강제재수렴(§5)까지
        한 CCTV 프레임 내에서 소진돼버리는 버그가 있었다. correct()는
        cctv_pose_cb에서 메시지 1개당 정확히 1번만 호출한다.
        """
        now = self.get_clock().now()
        stamp = Time.from_msg(msg.header.stamp)
        if stamp.nanoseconds == 0:
            stamp = now
        if self._last_wheel_stamp is None:
            dt = self.default_dt
        else:
            raw_dt = (stamp - self._last_wheel_stamp).nanoseconds * 1e-9
            if raw_dt <= 0.0:
                self.get_logger().warn('역행/중복 wheel_odom timestamp — 측정 폐기')
                return
            dt = min(raw_dt, self.max_dt)

        dx_body = msg.twist.twist.linear.x
        dy_body = msg.twist.twist.linear.y
        dtheta = msg.twist.twist.angular.z
        self.ekf.predict(dx_body, dy_body, dtheta, dt)
        self._last_wheel_stamp = stamp
        self.publish_odom(msg.header.stamp)

    def cctv_pose_cb(self, msg):
        """CCTV+ArUco 절대측정 도착 → correct() 1회.

        신선도는 '수신 시각'이 아니라 header.stamp(촬영/검출 시각) 기준으로
        판단한다. 수신 시각 기준이면 네트워크·Jetson 처리 지연으로 오래된
        pose도 "방금 온 새 측정"으로 오인해 그대로 반영해버린다.
        (지연 자체를 보상하는 재적분(retro-correction)은 하지 않는다 — 이
        속도 영역에서는 무시 가능하다고 판단했으나, 상세 트레이드오프는
        design doc §8 참조. 실측 지연이 커지면 추가 작업 필요.)
        """
        stamp = Time.from_msg(msg.header.stamp)
        age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
        self._last_cctv_age = age

        if msg.header.frame_id != 'map':
            self.ekf.reset_reject_streak()
            self._last_source = 'ENCODER_BAD_CCTV_FRAME'
            return
        if age < -self.max_future_skew:
            self.ekf.reset_reject_streak()
            self._last_source = 'ENCODER_FUTURE_CCTV'
            return
        if age > self.cctv_timeout:
            self.ekf.reset_reject_streak()
            self._last_source = 'ENCODER_STALE_CCTV'
            return

        yaw = yaw_from_quat(msg.pose.orientation)
        if yaw is None:
            self.ekf.reset_reject_streak()
            self._last_source = 'ENCODER_INVALID_CCTV'
            return
        values = (msg.pose.position.x, msg.pose.position.y, yaw)
        if not all(math.isfinite(v) for v in values):
            self.ekf.reset_reject_streak()
            self._last_source = 'ENCODER_INVALID_CCTV'
            return
        if (self._last_cctv_stamp is not None and
                stamp.nanoseconds <= self._last_cctv_stamp.nanoseconds):
            self.ekf.reset_reject_streak()
            self._last_source = 'ENCODER_DUPLICATE_CCTV'
            return
        self._last_cctv_stamp = stamp
        # Pose가 도착했다는 사실 자체가 해당 촬영 프레임의 marker-visible
        # 증거다. 별도 Bool 토픽의 교차토픽 전달 순서에 의존하지 않는다.
        self._cctv_visible = True
        was_initialized = self.ekf.initialized
        accepted = self.ekf.correct(
            msg.pose.position.x, msg.pose.position.y, yaw, R=self.R)
        if accepted and not was_initialized:
            # P1-2: 첫 측정은 보정이 아니라 절대 초기화다. init_* 명목값과의
            # 차이를 로그로 남겨 배치 오차를 현장에서 바로 확인할 수 있게 한다.
            self._last_source = 'CCTV_INITIALIZED'
            self.get_logger().info(
                f'[{self.role}] EKF 절대 초기화: '
                f'({msg.pose.position.x:.3f}, {msg.pose.position.y:.3f}, '
                f'{math.degrees(yaw):.1f}deg)')
        else:
            self._last_source = (
                'CCTV_ARUCO' if accepted else 'CCTV_REJECTED_GATE')

    def cctv_visible_cb(self, msg):
        self._cctv_visible = msg.data
        if not msg.data:
            self.ekf.reset_reject_streak()

    def publish_odom(self, stamp):
        x, y, yaw = self.ekf.pose()
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = 'map'
        msg.child_frame_id = f'{self.role}_base'
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        # 6x6 row-major, x/y/yaw만 채움 (나머지는 미사용 자유도)
        cov = [0.0] * 36
        cov[0] = self.ekf.P[0][0]
        cov[7] = self.ekf.P[1][1]
        cov[35] = self.ekf.P[2][2]
        msg.pose.covariance = cov
        self.pub_odom.publish(msg)

    def publish_status(self):
        pos_std = self.ekf.position_std()
        info = {
            'role': self.role,
            'source': self._last_source,
            'initialized': self.ekf.initialized,
            'pos_std_cm': [round(pos_std[0] * 100, 1), round(pos_std[1] * 100, 1)],
            'yaw_std_deg': round(math.degrees(self.ekf.yaw_std()), 2),
            'mahalanobis': round(self.ekf.last_mahalanobis, 2),
            'forced_reacquire': self.ekf.forced_reacquire,
            'cctv_visible': self._cctv_visible,
            'cctv_age_s': None if self._last_cctv_age is None else round(self._last_cctv_age, 2),
        }
        m = String()
        m.data = json.dumps(info)
        self.pub_status.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = PoseFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
