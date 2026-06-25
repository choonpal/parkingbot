import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
import math, json, time

class NavController(Node):
    def __init__(self):
        super().__init__('nav_controller')
        self.wheelbase = 0.25
        self.half_L = 0.125
        self.max_speed = 0.08
        self.max_omega = 0.3
        self.state = 'IDLE'
        self.target = None
        self.front = {'x':0.0,'y':0.0,'theta':0.0,'t':0.0}
        self.rear = {'x':0.0,'y':0.0,'theta':0.0,'t':0.0}
        self.error_level = 0
        self.speed_scale = 1.0
        self.front_ready = False
        self.rear_ready = False

        self.create_subscription(PoseStamped, '/parking/target_pose', self.target_cb, 10)
        self.create_subscription(Odometry, '/front/odom', self.front_cb, 10)
        self.create_subscription(Odometry, '/rear/odom', self.rear_cb, 10)
        self.pub_fc = self.create_publisher(Twist, '/front/cmd_vel', 10)
        self.pub_rc = self.create_publisher(Twist, '/rear/cmd_vel', 10)
        self.pub_estop = self.create_publisher(Bool, '/emergency_stop', 10)
        self.pub_status = self.create_publisher(String, '/nav/status', 10)
        self.create_timer(0.02, self.control_loop)
        self.create_timer(1.0, self.log_status)
        self.get_logger().info('nav_controller started - waiting for odom...')

    def target_cb(self, msg):
        x, y = msg.pose.position.x, msg.pose.position.y
        q = msg.pose.orientation
        t = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        self.target = (x, y, t)
        if self.front_ready and self.rear_ready:
            self.state = 'MOVING'
            self.get_logger().info(f'TARGET ({x:.3f}, {y:.3f})')

    def front_cb(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.front = {'x':p.x,'y':p.y,'theta':math.atan2(2*q.w*q.z,1-2*q.z*q.z),'t':time.time()}
        if not self.front_ready:
            self.front_ready = True
            self.get_logger().info(f'front odom OK ({p.x:.2f}, {p.y:.2f})')
            self.check_ready()

    def rear_cb(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.rear = {'x':p.x,'y':p.y,'theta':math.atan2(2*q.w*q.z,1-2*q.z*q.z),'t':time.time()}
        if not self.rear_ready:
            self.rear_ready = True
            self.get_logger().info(f'rear odom OK ({p.x:.2f}, {p.y:.2f})')
            self.check_ready()

    def check_ready(self):
        if self.front_ready and self.rear_ready and self.target:
            self.state = 'MOVING'
            self.get_logger().info('both robots ready - MOVING!')

    def control_loop(self):
        if not self.front_ready or not self.rear_ready:
            return
        now = time.time()
        if now - self.front['t'] > 0.5 or now - self.rear['t'] > 0.5:
            return
        if self.state != 'MOVING' or self.target is None:
            return

        cx = (self.front['x']+self.rear['x'])/2
        cy = (self.front['y']+self.rear['y'])/2
        dx = self.front['x']-self.rear['x']
        dy = self.front['y']-self.rear['y']
        ct = math.atan2(dy, dx)

        tx, ty, tt = self.target
        dist = math.sqrt((tx-cx)**2+(ty-cy)**2)
        if dist < 0.03:
            self.state = 'ARRIVED'
            self.send_stop()
            self.get_logger().info(f'ARRIVED! error: {dist*100:.1f}cm')
            return

        cc, ss = math.cos(ct), math.sin(ct)
        ldx = (tx-cx)*cc + (ty-cy)*ss
        ldy = -(tx-cx)*ss + (ty-cy)*cc
        dt = math.atan2(math.sin(tt-ct), math.cos(tt-ct))

        vx = max(-self.max_speed, min(self.max_speed, 0.8*ldx))
        vy = max(-self.max_speed, min(self.max_speed, 0.8*ldy))
        om = max(-self.max_omega, min(self.max_omega, 1.0*dt))
        if dist < 0.08:
            s = dist/0.08
            vx *= s; vy *= s

        fv = (vx, vy+om*self.half_L, om)
        rv = (vx, vy-om*self.half_L, om)

        ad = math.sqrt(dx*dx+dy*dy)
        de = abs(ad - self.wheelbase)
        if de < 0.005: self.error_level=0; self.speed_scale=1.0
        elif de < 0.015: self.error_level=1; self.speed_scale=0.7
        elif de < 0.030: self.error_level=2; self.speed_scale=0.3
        else: self.error_level=3; self.speed_scale=0.0

        if self.error_level >= 3:
            self.send_stop()
            self.get_logger().warn(f'E_STOP! dist_err={de*1000:.0f}mm')
            return

        ex = self.front['x']-self.wheelbase*math.cos(self.front['theta'])-self.rear['x']
        ey = self.front['y']-self.wheelbase*math.sin(self.front['theta'])-self.rear['y']
        cr, sr = math.cos(self.rear['theta']), math.sin(self.rear['theta'])
        lex, ley = ex*cr+ey*sr, -ex*sr+ey*cr
        Kp = [0,0.5,1.5,0][self.error_level]
        sc = self.speed_scale

        fc = Twist()
        fc.linear.x=fv[0]*sc; fc.linear.y=fv[1]*sc; fc.angular.z=fv[2]*sc
        self.pub_fc.publish(fc)

        rc = Twist()
        rc.linear.x=(rv[0]+Kp*lex)*sc; rc.linear.y=(rv[1]+Kp*ley)*sc; rc.angular.z=rv[2]*sc
        self.pub_rc.publish(rc)

    def send_stop(self):
        s = Twist()
        self.pub_fc.publish(s); self.pub_rc.publish(s)

    def log_status(self):
        cx=(self.front['x']+self.rear['x'])/2
        cy=(self.front['y']+self.rear['y'])/2
        d=0.0
        if self.target:
            d=math.sqrt((self.target[0]-cx)**2+(self.target[1]-cy)**2)
        ln=['OK','WARNING','DANGER','E_STOP']
        st={'state':self.state,'dist_cm':round(d*100,1),'sync':ln[self.error_level],'speed':self.speed_scale}
        m=String(); m.data=json.dumps(st); self.pub_status.publish(m)
        if self.state=='MOVING':
            self.get_logger().info(f'{self.state} | {d*100:.1f}cm | {ln[self.error_level]} | {self.speed_scale*100:.0f}%')

def main(args=None):
    rclpy.init(args=args)
    node = NavController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
