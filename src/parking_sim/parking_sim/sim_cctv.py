import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from std_msgs.msg import String
import json, math

SLOTS = [
    {"id": 1, "cx": 0.15, "cy": 0.10},
    {"id": 2, "cx": 0.40, "cy": 0.10},
    {"id": 3, "cx": 0.65, "cy": 0.10},
    {"id": 4, "cx": 0.90, "cy": 0.10},
]
OCCUPIED = [1, 3]

class SimCCTV(Node):
    def __init__(self):
        super().__init__('sim_cctv')
        self.pub_empty = self.create_publisher(PoseArray, '/parking/empty_slots', 10)
        self.pub_target = self.create_publisher(PoseStamped, '/parking/target_pose', 10)
        self.pub_info = self.create_publisher(String, '/parking/info', 10)
        self.timer = self.create_timer(1.0, self.publish_slots)
        empty_ids = [s["id"] for s in SLOTS if s["id"] not in OCCUPIED]
        self.get_logger().info(f'CCTV | occupied: {OCCUPIED} | empty: {empty_ids}')

    def publish_slots(self):
        now = self.get_clock().now().to_msg()
        empty = [s for s in SLOTS if s["id"] not in OCCUPIED]

        pa = PoseArray()
        pa.header.stamp = now
        pa.header.frame_id = 'world'
        for s in empty:
            p = Pose()
            p.position.x = s["cx"]
            p.position.y = s["cy"]
            pa.poses.append(p)
        self.pub_empty.publish(pa)

        if empty:
            ts = PoseStamped()
            ts.header.stamp = now
            ts.header.frame_id = 'world'
            ts.pose.position.x = empty[0]["cx"]
            ts.pose.position.y = empty[0]["cy"]
            ts.pose.orientation.z = math.sin(math.pi / 4)
            ts.pose.orientation.w = math.cos(math.pi / 4)
            self.pub_target.publish(ts)

        msg = String()
        msg.data = json.dumps({"empty": [s["id"] for s in empty], "occupied": OCCUPIED})
        self.pub_info.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimCCTV()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
