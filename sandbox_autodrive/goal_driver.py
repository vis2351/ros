"""RViz click-to-go navigation on the directed sand-table road graph."""
import json
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker
import rclpy

from .road_graph import RoadGraph, length
from .route_follower import sample_route


def yaw_of(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),
                      1-2*(q.y*q.y+q.z*q.z))


class GoalDriver(Node):
    def __init__(self):
        super().__init__('goal_driver')
        self.graph = RoadGraph()
        config = Path(get_package_share_directory('sandbox_autodrive'))/'config/route.json'
        self.spec = json.loads(config.read_text())
        self.declare_parameter('auto_survey', True)
        self.auto_survey = bool(self.get_parameter('auto_survey').value)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.odom = None
        self.signal = 'red'
        self.scan = None
        self.map_msg = None
        self.pending_goal = None
        self.waiting_for_tf_reported = False
        self.stages = []
        self.stage_idx = 0
        self.idx = 0
        self.arc_lengths = []
        self.hold_stop = None
        self.last_state = ''
        self.status = 'waiting_for_odom'
        self.target = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.path_pub = self.create_publisher(RosPath, '/route', qos)
        self.vehicle_pub = self.create_publisher(Marker, '/vehicle_marker', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/drive_state', 10)
        self.result_pub = self.create_publisher(String, '/nav_result', qos)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(String, '/traffic_light_state', self.on_signal, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 10)
        self.create_timer(.05, self.tick)
        self.create_timer(1., self.publish_route)
        if self.auto_survey:
            points = sample_route(self.spec)
            stop = self.spec['stop_center']
            stop_idx = min(range(len(points)),
                           key=lambda i:math.dist(points[i],stop))
            self.set_stages([{'gear':1,'points':points,
                              'stops':[(stop_idx,stop)],
                              'edges':['initial_slam_survey']}], 'mapping_drive')
        self.get_logger().info('Click 2D Goal Pose in RViz or publish /goal_pose; '
                               'targets snap to drivable directed lanes and bays')

    def on_odom(self, msg):
        self.odom = msg
        if self.status == 'waiting_for_odom':
            self.status = 'idle_ready'

    def on_signal(self, msg):
        self.signal = msg.data.strip().lower()

    def on_scan(self, msg):
        self.scan = msg

    def on_map(self, msg):
        if self.map_msg is None:
            self.get_logger().info(
                f'SLAM map available: {msg.info.width} x {msg.info.height} '
                f'at {msg.info.resolution:.3f} m/cell')
        self.map_msg = msg

    def on_goal(self, msg):
        self.pending_goal = msg
        self.waiting_for_tf_reported = False
        self.get_logger().info(
            f'Goal requested in {msg.header.frame_id or "map"}: '
            f'({msg.pose.position.x:.3f}, {msg.pose.position.y:.3f})')

    def transform_xy(self, point, source, target):
        if source == target:
            return point
        transform = self.tf_buffer.lookup_transform(target, source, Time())
        translation = transform.transform.translation
        yaw = yaw_of(transform.transform.rotation)
        return (translation.x+math.cos(yaw)*point[0]-math.sin(yaw)*point[1],
                translation.y+math.sin(yaw)*point[0]+math.cos(yaw)*point[1])

    def map_occupied(self, odom_xy):
        if self.map_msg is None:
            return False
        try:
            x,y = self.transform_xy(odom_xy, 'odom', 'map')
        except TransformException:
            return False
        info=self.map_msg.info
        ox,oy=info.origin.position.x,info.origin.position.y
        gx=int(math.floor((x-ox)/info.resolution))
        gy=int(math.floor((y-oy)/info.resolution))
        if 0<=gx<info.width and 0<=gy<info.height:
            return self.map_msg.data[gy*info.width+gx]>=80
        return False

    def maybe_plan_goal(self):
        if self.pending_goal is None or self.odom is None:
            return
        msg=self.pending_goal
        frame=msg.header.frame_id or 'map'
        try:
            goal=self.transform_xy((msg.pose.position.x,msg.pose.position.y),
                                   frame,'odom')
        except TransformException:
            if not self.waiting_for_tf_reported:
                self.report('waiting_for_slam_transform')
                self.waiting_for_tf_reported = True
            return
        self.pending_goal=None
        pose=self.odom.pose.pose
        x,y=pose.position.x,pose.position.y
        yaw=yaw_of(pose.orientation)
        if not all(math.isfinite(v) for v in (x,y,yaw,*goal)):
            self.report('goal_rejected: invalid coordinate')
            return
        if math.hypot(goal[0]-x,goal[1]-y)<.035:
            self.cmd_pub.publish(Twist())
            self.stages=[]
            self.status='arrived'
            self.report('arrived: already at target')
            return
        if self.map_occupied(goal):
            self.report('goal_rejected: SLAM map marks the target occupied')
            return
        try:
            stages,snapped=self.graph.plan((x,y),yaw,goal)
        except ValueError as exc:
            self.report(f'goal_rejected: {exc}')
            return
        if not stages:
            self.report('goal_rejected: route is too short')
            return
        self.target=snapped
        self.set_stages(stages,'navigating')
        distance=sum(length(stage['points']) for stage in stages)
        self.report(f'goal_accepted: target ({snapped[0]:.3f},'
                    f'{snapped[1]:.3f}), route {distance:.2f} m, '
                    f'{len(stages)} gear stage(s)')

    def report(self, message):
        self.result_pub.publish(String(data=message))
        self.get_logger().info(message)

    def set_stages(self, stages, status):
        self.stages=stages
        self.stage_idx=0
        self.status=status
        self.set_stage(0)
        if hasattr(self,'path_pub'):
            self.publish_route()

    def set_stage(self, index):
        self.stage_idx=index
        self.idx=0
        self.hold_stop=None
        self.last_state=''
        points=self.stages[index]['points']
        self.arc_lengths=[0.]
        for a,b in zip(points,points[1:]):
            self.arc_lengths.append(self.arc_lengths[-1]+math.dist(a,b))

    def publish_route(self):
        msg=RosPath()
        msg.header.frame_id='odom'
        msg.header.stamp=self.get_clock().now().to_msg()
        for stage in self.stages[self.stage_idx:]:
            for x,y in stage['points']:
                pose=PoseStamped()
                pose.header=msg.header
                pose.pose.position.x=x
                pose.pose.position.y=y
                pose.pose.position.z=.025
                pose.pose.orientation.w=1.
                msg.poses.append(pose)
        self.path_pub.publish(msg)

    def publish_vehicle(self):
        pose=self.odom.pose.pose
        marker=Marker()
        marker.header.frame_id='odom'
        # RViz should use the newest SLAM transform despite transport jitter.
        marker.ns='car'
        marker.id=1
        marker.type=Marker.LINE_STRIP
        marker.action=Marker.ADD
        marker.pose.orientation.w=1.
        marker.scale.x=.025
        yaw=yaw_of(pose.orientation)
        c,s=math.cos(yaw),math.sin(yaw)
        # A closed outline plus a short centre stroke makes the front obvious.
        for local_x,local_y in [(-.11,-.053),(.11,-.053),(.11,.053),
                                (-.11,.053),(-.11,-.053),(.11,0)]:
            point=Point()
            point.x=pose.position.x+c*local_x-s*local_y
            point.y=pose.position.y+s*local_x+c*local_y
            point.z=.06
            marker.points.append(point)
        marker.color.r=.10
        marker.color.g=.30
        marker.color.b=1.
        marker.color.a=1.
        self.vehicle_pub.publish(marker)

    def obstacle_distance(self, reverse):
        scan=self.scan
        if scan is None:
            return float('inf')
        closest=float('inf')
        for i,value in enumerate(scan.ranges):
            if not math.isfinite(value) or value<scan.range_min:
                continue
            angle=scan.angle_min+i*scan.angle_increment
            relevant=(abs(abs(angle)-math.pi)<.38 if reverse else
                      abs(angle)<.38)
            if relevant and value<closest:
                closest=value
        return closest

    def tick(self):
        self.maybe_plan_goal()
        if self.odom is None:
            return
        self.publish_vehicle()
        if not self.stages or self.status in ('arrived','idle_ready'):
            self.cmd_pub.publish(Twist())
            self.state_pub.publish(String(data=self.status))
            return
        stage=self.stages[self.stage_idx]
        points=stage['points']
        pose=self.odom.pose.pose
        x,y=pose.position.x,pose.position.y
        yaw=yaw_of(pose.orientation)
        if not all(math.isfinite(v) for v in (x,y,yaw)):
            self.cmd_pub.publish(Twist())
            self.state_pub.publish(String(data='invalid_odom'))
            return
        end=min(len(points),self.idx+32)
        candidate=min(range(max(0,self.idx-5),end),
                      key=lambda i:math.hypot(points[i][0]-x,points[i][1]-y))
        self.idx=max(self.idx,candidate)
        goal_x,goal_y=points[-1]
        if self.idx>=len(points)-3 and math.hypot(goal_x-x,goal_y-y)<.055:
            self.cmd_pub.publish(Twist())
            if self.stage_idx+1<len(self.stages):
                self.set_stage(self.stage_idx+1)
                self.get_logger().info('Changed driving direction for next route stage')
            else:
                self.stages=[]
                self.status='idle_ready' if self.status=='mapping_drive' else 'arrived'
                self.state_pub.publish(String(data=self.status))
                self.report('SLAM survey complete; waiting for a goal' if
                            self.status=='idle_ready' else 'arrived at selected target')
            return
        reverse=stage['gear']<0
        in_parking=abs(x)<1.06 and 1.10<y<2.18
        lookahead=.09 if in_parking or reverse else self.spec['lookahead_m']
        target=self.idx
        while target<len(points)-1 and \
                self.arc_lengths[target]-self.arc_lengths[self.idx]<lookahead:
            target+=1
        dx,dy=points[target][0]-x,points[target][1]-y
        distance=max(.04,math.hypot(dx,dy))
        travel_yaw=yaw+(math.pi if reverse else 0.)
        alpha=math.atan2(math.sin(math.atan2(dy,dx)-travel_yaw),
                         math.cos(math.atan2(dy,dx)-travel_yaw))
        steer=math.atan2(2*self.spec['wheelbase_m']*math.sin(alpha),distance)
        if reverse:
            steer=-steer
        limit=self.spec['max_steer_rad']
        steer=max(-limit,min(limit,steer))
        if not reverse and x>1.20 and .80<y<2.03:
            steer=0.
        speed=(-.055 if reverse else
               (.065 if in_parking else self.spec['cruise_speed_mps']))
        if not reverse and abs(steer)>.32:
            speed=min(speed,.075)
        state=('mapping_drive' if self.status=='mapping_drive' else
               'reversing' if reverse else
               'parking' if in_parking else 'driving_to_goal')

        if self.signal=='green':
            self.hold_stop=None
        if self.signal in ('red','yellow') and not reverse:
            if self.hold_stop is not None:
                events=[(i,p) for i,p in stage['stops'] if i==self.hold_stop]
            else:
                events=[(i,p) for i,p in stage['stops'] if i>=self.idx-2]
            if events:
                stop_idx,stop=events[0]
                if self.hold_stop==stop_idx:
                    error=(stop[0]-x)*math.cos(yaw)+(stop[1]-y)*math.sin(yaw)
                    speed=max(-.07,min(.07,.8*error))
                    if abs(error)<.008:
                        speed=0.
                    state='stopped_for_signal'
                else:
                    to_stop=max(0.,self.arc_lengths[stop_idx]-
                                self.arc_lengths[self.idx])
                    if to_stop<self.spec['stop_slowdown_m']:
                        speed=min(speed,max(0.,.35*(to_stop-.015)))
                        if to_stop<.045:
                            self.hold_stop=stop_idx
                            speed=0.
                        state=('stopped_for_signal' if self.hold_stop==stop_idx
                               else 'approaching_red')
        remaining=self.arc_lengths[-1]-self.arc_lengths[self.idx]
        if remaining<.20:
            speed=math.copysign(min(abs(speed),max(.025,.45*remaining)),speed)
        if self.obstacle_distance(reverse)<.15:
            speed=0.
            state='blocked_by_lidar'
        cmd=Twist()
        cmd.linear.x=speed
        cmd.angular.z=steer if abs(speed)>.001 else 0.
        self.cmd_pub.publish(cmd)
        self.state_pub.publish(String(data=state))
        if state!=self.last_state:
            self.get_logger().info(f'{state}; signal={self.signal}; '
                                   f'route_index={self.idx}; stage={self.stage_idx}')
            self.last_state=state


def main():
    rclpy.init()
    node=GoalDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()
