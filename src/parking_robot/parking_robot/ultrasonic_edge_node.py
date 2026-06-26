#!/usr/bin/env python3
"""
==================================================
[2-1] ultrasonic_edge_node
==================================================
초음파 엣지 검출기. 차 바퀴를 지나는 순간 포착.

거리값이 줄었다가(바퀴 접근) 늘어나는(바퀴 통과) 변곡점을
바퀴 중앙으로 판단하여 정렬 완료 신호 발행.

입력:
  /robot/ultrasonic (sensor_msgs/Range) — 초음파 거리
  (또는 라즈베리파이 GPIO 직접 읽기)
출력:
  /robot/wheel_aligned (std_msgs/Bool) — 바퀴 정렬 완료
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Bool
from collections import deque

# 라즈베리파이 GPIO 직접 읽기 (옵션)
try:
    import RPi.GPIO as GPIO
    import time
    GPIO_OK = True
except ImportError:
    GPIO_OK = False


class UltrasonicEdgeNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_edge_node')

        # ===== 파라미터 =====
        self.declare_parameter('use_gpio', False)
        self.declare_parameter('trig_pin', 23)
        self.declare_parameter('echo_pin', 24)
        self.declare_parameter('threshold_m', 0.15)   # 바퀴 감지 임계
        self.declare_parameter('window_size', 5)       # 이동평균 필터
        self.declare_parameter('edge_drop_m', 0.03)    # 엣지 판정 변화량

        self.use_gpio = self.get_parameter('use_gpio').value
        self.threshold = self.get_parameter('threshold_m').value
        self.window = self.get_parameter('window_size').value
        self.edge_drop = self.get_parameter('edge_drop_m').value

        # ===== 상태 =====
        self.distances = deque(maxlen=self.window)
        self.min_seen = float('inf')
        self.wheel_detected = False
        self.in_wheel_zone = False   # 바퀴 영역 진입 여부

        # ===== GPIO 설정 (라즈베리파이) =====
        if self.use_gpio and GPIO_OK:
            self.trig = self.get_parameter('trig_pin').value
            self.echo = self.get_parameter('echo_pin').value
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.trig, GPIO.OUT)
            GPIO.setup(self.echo, GPIO.IN)
            GPIO.output(self.trig, False)
            self.create_timer(0.05, self.read_gpio)   # 20Hz
            self.get_logger().info('GPIO 초음파 모드')
        else:
            # 토픽 구독 모드
            self.create_subscription(Range, '/robot/ultrasonic',
                                     self.range_cb, 10)
            self.get_logger().info('토픽 구독 모드')

        # ===== 발행 =====
        self.pub_aligned = self.create_publisher(
            Bool, '/robot/wheel_aligned', 10)

        self.get_logger().info('ultrasonic_edge_node 시작')

    # ================================================
    # GPIO 직접 읽기
    # ================================================
    def read_gpio(self):
        GPIO.output(self.trig, True)
        time.sleep(0.00001)
        GPIO.output(self.trig, False)

        start, stop = time.time(), time.time()
        timeout = time.time() + 0.04
        while GPIO.input(self.echo) == 0 and time.time() < timeout:
            start = time.time()
        while GPIO.input(self.echo) == 1 and time.time() < timeout:
            stop = time.time()

        elapsed = stop - start
        distance = (elapsed * 343.0) / 2.0   # m
        self.process_distance(distance)

    def range_cb(self, msg):
        self.process_distance(msg.range)

    # ================================================
    # 엣지 검출 로직
    # ================================================
    def process_distance(self, dist):
        if dist <= 0 or dist > 2.0:
            return  # 노이즈 무시

        # 이동평균 필터 (초음파 노이즈 제거)
        self.distances.append(dist)
        if len(self.distances) < self.window:
            return
        filtered = sum(self.distances) / len(self.distances)

        if self.wheel_detected:
            return

        # 1단계: 거리가 임계 이하로 줄면 바퀴 영역 진입
        if filtered < self.threshold:
            self.in_wheel_zone = True
            if filtered < self.min_seen:
                self.min_seen = filtered

        # 2단계: 영역 안에서 다시 거리가 늘면(변곡점) = 바퀴 중앙 통과
        if self.in_wheel_zone:
            if filtered > self.min_seen + self.edge_drop:
                # 줄었다 늘어나는 변곡점 = 바퀴 중심
                self.wheel_detected = True
                self.publish_aligned()
                self.get_logger().info(
                    f'바퀴 중앙 감지! (최소거리 {self.min_seen:.3f}m)')

    def publish_aligned(self):
        msg = Bool()
        msg.data = True
        self.pub_aligned.publish(msg)

    def reset(self):
        """다음 사이클을 위한 초기화"""
        self.distances.clear()
        self.min_seen = float('inf')
        self.wheel_detected = False
        self.in_wheel_zone = False

    def destroy_node(self):
        if self.use_gpio and GPIO_OK:
            GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicEdgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
