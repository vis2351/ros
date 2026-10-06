"""Keep the tiny Ackermann model on the commanded kinematic path in Gazebo.

Gazebo Classic's contact solver can tip a 10 cm wide car at the 15 cm bridge.
The Ackermann plugin still accepts /cmd_vel and publishes /odom; this node
sets the model pose from the same commanded speed and steering angle.
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState


class PoseStabilizer(Node):
    def __init__(self):
        super().__init__('pose_stabilizer')
        self.x = None
        self.y = None
        self.yaw = None
        self.last_time = None
        self.cmd = Twist()
        self.pending = None
        self.client = self.create_client(SetEntityState, '/set_entity_state')
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.create_timer(.02, self.tick)

    def on_odom(self, msg):
        if self.x is not None:
            return
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y),
                         1-2*(q.y*q.y+q.z*q.z))
        if all(math.isfinite(v) for v in (p.x,p.y,yaw)):
            self.x, self.y, self.yaw = p.x, p.y, yaw

    def on_cmd(self, msg):
        self.cmd = msg

    def road_height_and_pitch(self):
        if abs(self.x)>1.15 and .70<self.y<2.025:
            u = (self.y-.70)/(2.025-.70)
            height = -.004+.154*u
            # Keep the 2D laser level while its height follows the ramp.
            return max(-.002,height-.002),0.
        if self.y>=2.025:
            return .148,0.
        return -.002,0.

    def tick(self):
        if self.x is None or not self.client.service_is_ready():
            return
        if self.pending is not None and not self.pending.done():
            return
        now = self.get_clock().now().nanoseconds/1e9
        if self.last_time is None:
            self.last_time = now
            return
        dt = max(0., min(.06, now-self.last_time))
        self.last_time = now
        speed = max(-.22,min(.22,self.cmd.linear.x))
        steer = max(-.68,min(.68,self.cmd.angular.z))
        yaw_rate = speed*math.tan(steer)/.13
        mid_yaw = self.yaw+yaw_rate*dt/2
        self.x += speed*math.cos(mid_yaw)*dt
        self.y += speed*math.sin(mid_yaw)*dt
        self.yaw = math.atan2(math.sin(self.yaw+yaw_rate*dt),
                              math.cos(self.yaw+yaw_rate*dt))
        z,pitch = self.road_height_and_pitch()
        half_yaw = self.yaw/2
        half_pitch = pitch/2
        state = EntityState()
        state.name = 'sandbox_car'
        state.reference_frame = 'world'
        state.pose.position.x = self.x
        state.pose.position.y = self.y
        state.pose.position.z = z
        state.pose.orientation.x = -math.sin(half_pitch)*math.sin(half_yaw)
        state.pose.orientation.y = math.sin(half_pitch)*math.cos(half_yaw)
        state.pose.orientation.z = math.cos(half_pitch)*math.sin(half_yaw)
        state.pose.orientation.w = math.cos(half_pitch)*math.cos(half_yaw)
        state.twist.linear.x = speed*math.cos(self.yaw)
        state.twist.linear.y = speed*math.sin(self.yaw)
        state.twist.angular.z = yaw_rate
        request = SetEntityState.Request()
        request.state = state
        self.pending = self.client.call_async(request)


def main():
    rclpy.init()
    node = PoseStabilizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
