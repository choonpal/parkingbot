import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import cv2, numpy as np, math, json

SC=5; WW=120; WH=80; IW=WW*SC; IH=WH*SC
SLOTS=[{"id":1,"x":5,"y":3,"w":20,"h":14},{"id":2,"x":30,"y":3,"w":20,"h":14},
       {"id":3,"x":55,"y":3,"w":20,"h":14},{"id":4,"x":80,"y":3,"w":20,"h":14}]

def cm2px(x,y): return int(x*SC),int(y*SC)

class Visualizer(Node):
    def __init__(self):
        super().__init__('visualizer')
        self.front={'x':0,'y':0,'theta':0}
        self.rear={'x':0,'y':0,'theta':0}
        self.target=None; self.nav_status={}; self.occupied=[1,3]
        self.create_subscription(Odometry,'/front/odom',self.f_cb,10)
        self.create_subscription(Odometry,'/rear/odom',self.r_cb,10)
        self.create_subscription(PoseStamped,'/parking/target_pose',self.t_cb,10)
        self.create_subscription(String,'/nav/status',self.s_cb,10)
        self.create_subscription(String,'/parking/info',self.p_cb,10)
        self.create_timer(1/30, self.render)
        self.get_logger().info('visualizer started - press q to quit')

    def f_cb(self,m):
        p,q=m.pose.pose.position,m.pose.pose.orientation
        self.front={'x':p.x*100,'y':p.y*100,'theta':math.atan2(2*q.w*q.z,1-2*q.z*q.z)}
    def r_cb(self,m):
        p,q=m.pose.pose.position,m.pose.pose.orientation
        self.rear={'x':p.x*100,'y':p.y*100,'theta':math.atan2(2*q.w*q.z,1-2*q.z*q.z)}
    def t_cb(self,m): self.target=(m.pose.position.x*100,m.pose.position.y*100)
    def s_cb(self,m):
        try: self.nav_status=json.loads(m.data)
        except: pass
    def p_cb(self,m):
        try: self.occupied=json.loads(m.data).get('occupied',[])
        except: pass

    def draw_robot(self,img,r,color,label):
        cx,cy=cm2px(r['x'],r['y']); t=r['theta']; hw=10; hh=10
        corners=np.array([[-hw,-hh],[hw,-hh],[hw,hh],[-hw,hh]],dtype=np.float32)
        rot=np.array([[math.cos(t),-math.sin(t)],[math.sin(t),math.cos(t)]])
        pts=(rot@corners.T).T+np.array([cx,cy])
        cv2.fillPoly(img,[pts.astype(np.int32)],color)
        cv2.polylines(img,[pts.astype(np.int32)],True,(255,255,255),1)
        ax=int(cx+20*math.cos(t)); ay=int(cy+20*math.sin(t))
        cv2.arrowedLine(img,(cx,cy),(ax,ay),(255,255,255),2)
        cv2.putText(img,label,(cx-15,cy-18),cv2.FONT_HERSHEY_SIMPLEX,0.4,color,1)

    def render(self):
        img=np.full((IH,IW,3),(30,30,40),dtype=np.uint8)
        cv2.rectangle(img,(0,0),(IW-1,IH-1),(80,80,80),1)

        for s in SLOTS:
            x1,y1=cm2px(s['x'],s['y']); x2,y2=cm2px(s['x']+s['w'],s['y']+s['h'])
            if s['id'] in self.occupied:
                cv2.rectangle(img,(x1,y1),(x2,y2),(0,0,200),-1)
                mx,my=(x1+x2)//2,(y1+y2)//2
                cv2.rectangle(img,(mx-12,my-8),(mx+12,my+8),(50,50,150),-1)
                cv2.putText(img,'CAR',(mx-14,my+4),cv2.FONT_HERSHEY_SIMPLEX,0.3,(200,200,200),1)
            else:
                cv2.rectangle(img,(x1,y1),(x2,y2),(0,180,0),1)
            cv2.putText(img,f'#{s["id"]}',(x1+3,y2-3),cv2.FONT_HERSHEY_SIMPLEX,0.35,(200,200,200),1)

        if self.target:
            tx,ty=cm2px(*self.target)
            cv2.drawMarker(img,(tx,ty),(0,200,255),cv2.MARKER_CROSS,20,2)
            cv2.putText(img,'TARGET',(tx+12,ty-5),cv2.FONT_HERSHEY_SIMPLEX,0.35,(0,200,255),1)

        fp=cm2px(self.front['x'],self.front['y'])
        rp=cm2px(self.rear['x'],self.rear['y'])
        cv2.line(img,fp,rp,(100,100,200),1,cv2.LINE_AA)
        ccx=(self.front['x']+self.rear['x'])/2; ccy=(self.front['y']+self.rear['y'])/2
        cp=cm2px(ccx,ccy)
        cv2.circle(img,cp,6,(0,100,255),2); cv2.circle(img,cp,2,(0,100,255),-1)

        self.draw_robot(img,self.front,(240,160,50),'FRONT')
        self.draw_robot(img,self.rear,(50,200,50),'REAR')

        st=self.nav_status.get('state','IDLE')
        d=self.nav_status.get('dist_cm',0)
        lv=self.nav_status.get('sync','OK')
        sc=self.nav_status.get('speed',1.0)
        col={'OK':(0,255,0),'WARNING':(0,200,255),'DANGER':(0,100,255),'E_STOP':(0,0,255)}.get(lv,(200,200,200))
        cv2.putText(img,f'{st} | {d:.1f}cm | {lv} | {sc*100:.0f}%',(10,IH-15),cv2.FONT_HERSHEY_SIMPLEX,0.4,col,1)

        cv2.imshow('Parking Robot Sim', img)
        if cv2.waitKey(1)&0xFF==ord('q'):
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node=Visualizer()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()
