"""Legal point-to-point drive, forward bay parking, reverse exit and final goal."""
import json
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path as RosPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty, String
import rclpy

from .route_follower import sample_route


def line(start, end):
    return {'type': 'line', 'from': list(start), 'to': list(end)}


def arc(center, radius, start_deg, end_deg):
    return {'type': 'arc', 'center': list(center), 'radius': radius,
            'start_deg': start_deg, 'end_deg': end_deg}


def bezier(control1, control2, end):
    return {'type': 'bezier', 'control1': list(control1),
            'control2': list(control2), 'to': list(end)}


def make_mission(spec, bay_index):
    """Draw centerlines inside the existing asphalt, observing A/B one-way use."""
    bay_x = -.82 + (bay_index + .5) * .205
    approach_segments = list(spec['segments'][:6])
    approach_segments += [
        line((-1.15, .675), (-.70, .675)),
        bezier((-.50, .675), (-.46, 1.125), (0, 1.125)),
        bezier((.46, 1.125), (.50, .675), (.70, .675)),
        line((.70, .675), (.75, .675)),
        arc((.75, .845), .17, -90, 0),
        line((.92, .845), (.92, 1.44)),
        arc((.75, 1.44), .17, 0, 90),
        line((.75, 1.61), (bay_x + .17, 1.61)),
        arc((bay_x + .17, 1.78), .17, -90, -180),
        line((bay_x, 1.78), (bay_x, 1.93)),
    ]
    approach = sample_route({'sample_step_m': spec['sample_step_m'],
                             'segments': approach_segments})
    # Back along the actual parking turn, without reversing all the way to B.
    reverse_start = min(range(len(approach)),
                        key=lambda i: math.dist(approach[i],
                                                (bay_x + .17, 1.61)))
    reverse = list(reversed(approach[reverse_start:]))
    departure_segments = [
        line((bay_x + .17, 1.61), (-.75, 1.61)),
        arc((-.75, 1.44), .17, 90, 180),
        line((-.92, 1.44), (-.92, .845)),
        arc((-.75, .845), .17, 180, 270),
        line((-.75, .675), (-.55, .675)),
    ] + list(spec['segments'][7:])
    departure = sample_route({'sample_step_m': spec['sample_step_m'],
                              'segments': departure_segments})
    return {'approach': approach, 'reverse': reverse,
            'departure': departure}, bay_x


class ParkingDriver(Node):
    def __init__(self):
        super().__init__('parking_driver')
        config = Path(get_package_share_directory('sandbox_autodrive')) / 'config/route.json'
        self.spec = json.loads(config.read_text())
        self.declare_parameter('bay_index', 5)
        self.declare_parameter('park_seconds', -1.0)
        bay_index = int(self.get_parameter('bay_index').value)
        if not 0 <= bay_index <= 5:
            raise ValueError('bay_index must be 0 through 5 for this maneuver')
        self.park_seconds = float(self.get_parameter('park_seconds').value)
        self.routes, self.bay_x = make_mission(self.spec, bay_index)
        self.parking_start_idx = min(range(len(self.routes['approach'])),
                                     key=lambda i: math.dist(self.routes['approach'][i],
                                                             (.70, .675)))
        self.parking_exit_idx = min(range(len(self.routes['departure'])),
                                    key=lambda i: math.dist(self.routes['departure'][i],
                                                            (-.55, .675)))
        self.stage = 'approach'
        self.idx = 0
        self.odom = None
        self.signal = 'red'
        self.hold_red = False
        self.last_state = ''
        self.parked_at = None
        self.depart_requested = False
        self.arc_lengths = []
        self.stop_idx = None
        self.set_stage('approach', announce=False)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.path_pub = self.create_publisher(RosPath, '/route', qos)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/drive_state', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(String, '/traffic_light_state', self.on_signal, 10)
        self.create_subscription(Empty, '/parking_depart', self.on_depart, 10)
        self.create_timer(.05, self.tick)
        self.create_timer(1., self.publish_route)
        self.get_logger().info(
            f'Point-to-point mission: start (1.35,-1.45), park in bay {bay_index + 1} '
            f'at ({self.bay_x:.3f},1.93), exit A, finish (1.35,-1.45)')

    def on_odom(self, msg):
        self.odom = msg

    def on_signal(self, msg):
        self.signal = msg.data.strip().lower()

    def on_depart(self, _msg):
        self.depart_requested = True

    def set_stage(self, stage, announce=True):
        self.stage = stage
        self.idx = 0
        self.hold_red = False
        self.last_state = ''
        if stage in self.routes:
            points = self.routes[stage]
            self.arc_lengths = [0.]
            for a, b in zip(points, points[1:]):
                self.arc_lengths.append(self.arc_lengths[-1] + math.dist(a, b))
            if stage == 'departure':
                self.stop_idx = min(range(len(points)), key=lambda i:
                                    math.dist(points[i], self.spec['stop_center']))
        if announce:
            self.get_logger().info(f'Mission stage: {stage}')
        if hasattr(self, 'path_pub'):
            self.publish_route()

    def publish_route(self):
        if self.stage not in self.routes:
            return
        msg = RosPath()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in self.routes[self.stage]:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = .025
            pose.pose.orientation.w = 1.
            msg.poses.append(pose)
        self.path_pub.publish(msg)

    def tick(self):
        if self.odom is None:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if self.stage == 'parked':
            self.cmd_pub.publish(Twist())
            self.state_pub.publish(String(data='parked'))
            if self.depart_requested or (self.park_seconds >= 0. and
                                         now - self.parked_at >= self.park_seconds):
                self.set_stage('reverse')
            return
        if self.stage == 'arrived':
            self.cmd_pub.publish(Twist())
            self.state_pub.publish(String(data='arrived'))
            return

        pose = self.odom.pose.pose
        x, y = pose.position.x, pose.position.y
        q = pose.orientation
        yaw = math.atan2(2 * (q.w*q.z + q.x*q.y),
                         1 - 2 * (q.y*q.y + q.z*q.z))
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            self.cmd_pub.publish(Twist())
            self.state_pub.publish(String(data='invalid_odom'))
            return

        points = self.routes[self.stage]
        end = min(len(points), self.idx + 32)
        candidate = min(range(max(0, self.idx - 5), end),
                        key=lambda i: math.hypot(points[i][0]-x, points[i][1]-y))
        self.idx = max(self.idx, candidate)
        goal_x, goal_y = points[-1]
        if self.idx >= len(points)-3 and math.hypot(goal_x-x, goal_y-y) < .055:
            self.cmd_pub.publish(Twist())
            if self.stage == 'approach':
                self.parked_at = now
                self.set_stage('parked')
            elif self.stage == 'reverse':
                self.set_stage('departure')
            else:
                self.set_stage('arrived')
            return

        reverse = self.stage == 'reverse'
        in_parking = reverse or \
            (self.stage == 'approach' and self.idx >= self.parking_start_idx) or \
            (self.stage == 'departure' and self.idx <= self.parking_exit_idx)
        lookahead = .09 if in_parking else self.spec['lookahead_m']
        target = self.idx
        while target < len(points)-1 and \
                self.arc_lengths[target]-self.arc_lengths[self.idx] < lookahead:
            target += 1
        dx, dy = points[target][0]-x, points[target][1]-y
        distance = max(.04, math.hypot(dx, dy))
        travel_yaw = yaw + (math.pi if reverse else 0.)
        alpha = math.atan2(math.sin(math.atan2(dy, dx)-travel_yaw),
                           math.cos(math.atan2(dy, dx)-travel_yaw))
        steer = math.atan2(2*self.spec['wheelbase_m']*math.sin(alpha), distance)
        if reverse:
            steer = -steer
        limit = self.spec['max_steer_rad']
        steer = max(-limit, min(limit, steer))
        if not reverse and x > 1.20 and .80 < y < 2.03:
            steer = 0.

        speed = -.055 if reverse else (.065 if in_parking else self.spec['cruise_speed_mps'])
        if abs(steer) > .32 and not reverse:
            speed = min(speed, .075)
        state = {'approach': 'parking' if in_parking else 'driving_to_parking',
                 'reverse': 'reversing_out',
                 'departure': 'driving_to_goal'}[self.stage]
        if self.stage == 'departure':
            if self.signal == 'green':
                self.hold_red = False
            if self.signal in ('red', 'yellow'):
                if self.hold_red:
                    sx, sy = self.spec['stop_center']
                    error = (sx-x)*math.cos(yaw) + (sy-y)*math.sin(yaw)
                    speed = max(-.07, min(.07, .8*error))
                    if abs(error) < .008:
                        speed = 0.
                    state = 'stopped_for_signal'
                elif self.idx <= self.stop_idx:
                    distance_to_stop = max(0., self.arc_lengths[self.stop_idx] -
                                           self.arc_lengths[self.idx])
                    if distance_to_stop < self.spec['stop_slowdown_m']:
                        speed = min(speed, max(0., .35*(distance_to_stop-.015)))
                        if distance_to_stop < .045:
                            self.hold_red = True
                            speed = 0.
                        state = ('stopped_for_signal' if self.hold_red
                                 else 'approaching_red')
        remaining = self.arc_lengths[-1] - self.arc_lengths[self.idx]
        if remaining < .20:
            speed = math.copysign(min(abs(speed), max(.025, .45*remaining)), speed)
        cmd = Twist()
        cmd.linear.x = speed
        cmd.angular.z = steer if abs(speed) > .001 else 0.
        self.cmd_pub.publish(cmd)
        self.state_pub.publish(String(data=state))
        if state != self.last_state:
            self.get_logger().info(f'{state}; signal={self.signal}; route_index={self.idx}')
            self.last_state = state


def main():
    rclpy.init()
    node = ParkingDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
