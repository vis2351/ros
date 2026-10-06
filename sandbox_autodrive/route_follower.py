"""Follow the legal closed route, with a stop before the north zebra crossing."""
import json
import math
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path as RosPath
from std_msgs.msg import String
from visualization_msgs.msg import Marker

def sample_route(spec):
    step=spec['sample_step_m']; points=[]
    for seg in spec['segments']:
        if seg['type']=='line':
            a=seg.get('from',points[-1] if points else [0,0]); b=seg['to']
            n=max(1,math.ceil(math.dist(a,b)/step))
            for i in range(1,n+1): points.append((a[0]+(b[0]-a[0])*i/n,a[1]+(b[1]-a[1])*i/n))
        elif seg['type']=='arc':
            c=seg['center']; r=seg['radius']; a=math.radians(seg['start_deg']); b=math.radians(seg['end_deg'])
            n=max(1,math.ceil(abs(b-a)*r/step))
            for i in range(1,n+1):
                t=a+(b-a)*i/n;points.append((c[0]+r*math.cos(t),c[1]+r*math.sin(t)))
        elif seg['type']=='bezier':
            a=seg.get('from',points[-1]); b=seg['control1']; c=seg['control2']; d=seg['to']
            estimate=math.dist(a,b)+math.dist(b,c)+math.dist(c,d)
            n=max(2,math.ceil(estimate/step))
            for i in range(1,n+1):
                t=i/n; u=1-t
                points.append((u**3*a[0]+3*u*u*t*b[0]+3*u*t*t*c[0]+t**3*d[0],
                               u**3*a[1]+3*u*u*t*b[1]+3*u*t*t*c[1]+t**3*d[1]))
        else:
            raise ValueError(f"Unknown route segment: {seg['type']}")
    return points

class Driver(Node):
    def __init__(self):
        super().__init__('route_follower')
        cfg=Path(get_package_share_directory('sandbox_autodrive'))/'config/route.json'
        self.spec=json.loads(cfg.read_text())
        self.points=sample_route(self.spec);self.idx=0;self.odom=None;self.signal='red';self.last_state='';self.hold_red=False;self.completed=False
        self.stop_idx=min(range(len(self.points)),key=lambda i:math.dist(self.points[i],self.spec['stop_center']))
        self.arc=[0.0]
        for a,b in zip(self.points,self.points[1:]):self.arc.append(self.arc[-1]+math.dist(a,b))
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.path_pub=self.create_publisher(RosPath,'/route',qos)
        self.marker_pub=self.create_publisher(Marker,'/vehicle_marker',10)
        self.cmd_pub=self.create_publisher(Twist,'/cmd_vel',10)
        self.state_pub=self.create_publisher(String,'/drive_state',10)
        self.create_subscription(Odometry,'/odom',self.on_odom,10)
        self.create_subscription(String,'/traffic_light_state',self.on_signal,10)
        self.create_timer(.05,self.tick);self.create_timer(1.,self.publish_route)
        self.get_logger().info(f'Loaded {len(self.points)} route samples; stop index {self.stop_idx}')
    def on_odom(self,msg):self.odom=msg
    def on_signal(self,msg):self.signal=msg.data.lower().strip()
    def publish_route(self):
        msg=RosPath();msg.header.frame_id='odom';msg.header.stamp=self.get_clock().now().to_msg()
        for x,y in self.points:
            p=PoseStamped();p.header=msg.header;p.pose.position.x=x;p.pose.position.y=y;p.pose.position.z=.025;p.pose.orientation.w=1.
            msg.poses.append(p)
        self.path_pub.publish(msg)
    def tick(self):
        if self.odom is None:return
        if self.completed:
            self.cmd_pub.publish(Twist())
            self.state_pub.publish(String(data='lap_complete'))
            return
        p=self.odom.pose.pose.position;q=self.odom.pose.pose.orientation
        x,y=p.x,p.y
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        if not all(math.isfinite(v) for v in (x,y,yaw)):
            self.cmd_pub.publish(Twist())
            self.state_pub.publish(String(data='invalid_odom'))
            if self.last_state!='invalid_odom':
                self.get_logger().error('Invalid odometry; command stopped')
                self.last_state='invalid_odom'
            return
        # Restrict the search to nearby forward samples so a full circle does not skip a lap.
        end=min(len(self.points),self.idx+32)
        candidate=min(range(max(0,self.idx-5),end),key=lambda i:math.hypot(self.points[i][0]-x,self.points[i][1]-y))
        self.idx=max(self.idx,candidate)
        if self.idx>=len(self.points)-3:
            self.completed=True
            self.cmd_pub.publish(Twist())
            self.state_pub.publish(String(data='lap_complete'))
            self.get_logger().info('Completed the closed demonstration route')
            return
        look=self.spec['lookahead_m'];target=self.idx
        while target<len(self.points)-1 and self.arc[target]-self.arc[self.idx]<look:target+=1
        tx,ty=self.points[target]
        dx,dy=tx-x,ty-y
        ld=max(.06,math.hypot(dx,dy))
        alpha=math.atan2(dy,dx)-yaw
        alpha=math.atan2(math.sin(alpha),math.cos(alpha))
        steer=math.atan2(2*self.spec['wheelbase_m']*math.sin(alpha),ld)
        steer=max(-self.spec['max_steer_rad'],min(self.spec['max_steer_rad'],steer))
        # Keep all four wheels aligned through the climb and its upper seam.
        if x>1.20 and .80<y<2.03:
            steer=0.
        speed=self.spec['cruise_speed_mps']
        if abs(steer)>.32:speed=min(speed,.075)
        state='driving'
        if self.signal=='green':
            self.hold_red=False
        if self.signal in ('red','yellow'):
            if self.hold_red:
                sx,sy=self.spec['stop_center']
                error=(sx-x)*math.cos(yaw)+(sy-y)*math.sin(yaw)
                speed=max(-.07,min(.07,.8*error))
                if abs(error)<.008:speed=0.
                state='stopped_for_signal'
            elif self.idx<=self.stop_idx:
                d=max(0.,self.arc[self.stop_idx]-self.arc[self.idx])
                if d<self.spec['stop_slowdown_m']:
                    speed=min(speed,max(0.,.35*(d-.015)))
                    if d<.045:
                        self.hold_red=True
                        speed=0.
                    state='stopped_for_signal' if self.hold_red else 'approaching_red'
        remaining=self.arc[-1]-self.arc[self.idx]
        if remaining<.30:
            speed=min(speed,max(0.,.45*remaining))
        cmd=Twist();cmd.linear.x=speed;cmd.angular.z=steer if speed>.001 else 0.
        self.cmd_pub.publish(cmd)
        if state!=self.last_state:
            self.get_logger().info(f'{state}; signal={self.signal}; route_index={self.idx}')
            self.last_state=state
        self.state_pub.publish(String(data=state))
        m=Marker();m.header.frame_id='odom';m.header.stamp=self.get_clock().now().to_msg();m.ns='car';m.id=1;m.type=Marker.ARROW;m.action=Marker.ADD
        m.pose.position.x=x;m.pose.position.y=y;m.pose.position.z=.09;m.pose.orientation=q
        m.scale.x=.22;m.scale.y=.053;m.scale.z=.053;m.color.r=.10;m.color.g=.30;m.color.b=1.;m.color.a=.85
        self.marker_pub.publish(m)

def main():
    rclpy.init();node=Driver()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.cmd_pub.publish(Twist());node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
