import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import math, random

class SimMecanumRobot(Node):
    def __init__(self):
        super().__init__('sim_robot')
        self.declare_parameter('name', 'front')
        self.declare_parameter('start_x', 0.7)
        self.declare_parameter('start_y', 0.6)
        self.declare_parameter('start_theta', 1.5708)
        self.name = self.get_parameter('name').value
        self.x = self.get_parameter('start_x').value
        self.y = self.get_parameter('start_y').value
        self.theta = self.get_parameter('start_theta').value
        self.vx = self.vy = self.omega = 0.0
        self.e_stopped = False

        self.sub_cmd = self.create_subscription(Twist, f'/{self.name}/cmd_vel', self.cmd_cb, 10)
        self.sub_estop = self.create_subscription(Bool, '/emergency_stop', self.estop_cb, 10)
        self.pub_odom = self.create_publisher(Odometry, f'/{self.name}/odom', 10)
        self.dt = 0.02
        self.timer = self.create_timer(self.dt, self.update)
        self.get_logger().info(f'[{self.name}] start ({self.x:.2f}, {self.y:.2f})')

    def cmd_cb(self, msg):
        if not self.e_stopped:
            self.vx = max(-0.1, min(0.1, msg.linear.x))
            self.vy = max(-0.1, min(0.1, msg.linear.y))
            self.omega = max(-1.0, min(1.0, msg.angular.z))

    def estop_cb(self, msg):
        if msg.data:
            print(f'[{self.name}] EMERGENCY STOP!')
            self.e_stopped = True
            self.vx = self.vy = self.omega = 0.0

    def update(self):
        if self.e_stopped:
            self.vx = self.vy = self.omega = 0.0
        ct, st = math.cos(self.theta), math.sin(self.theta)
        gvx = self.vx * ct - self.vy * st
        gvy = self.vx * st + self.vy * ct
        self.x += gvx * self.dt + random.gauss(0, 0.0008)
        self.y += gvy * self.dt + random.gauss(0, 0.0008)
        self.theta += self.omega * self.dt
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        self.x = max(0.05, min(1.15, self.x))
        self.y = max(0.05, min(0.75, self.y))

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'world'
        odom.child_frame_id = f'{self.name}_base'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2)
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.omega
        self.pub_odom.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = SimMecanumRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
