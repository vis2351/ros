"""Cycle the intersection signal in ROS, RViz and the Gazebo scene."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState


class Signal(Node):
    def __init__(self):
        super().__init__('traffic_signal')
        self.pub = self.create_publisher(String, '/traffic_light_state', 10)
        self.marker = self.create_publisher(Marker, '/traffic_light_marker', 10)
        self.client = self.create_client(SetEntityState, '/set_entity_state')
        self.start = None
        self.last = ''
        self.visual_state = ''
        self.create_timer(.1, self.tick)

    def set_bulb(self, name, y, z):
        state = EntityState()
        state.name = name
        state.reference_frame = 'world'
        state.pose.position.x = -.37
        state.pose.position.y = y
        state.pose.position.z = z
        state.pose.orientation.w = 1.
        request = SetEntityState.Request()
        request.state = state
        self.client.call_async(request)

    def tick(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if self.start is None:
            self.start = now
        phase = (now - self.start) % 60.
        state = 'red' if phase < 40 else 'green' if phase < 57 else 'yellow'
        self.pub.publish(String(data=state))
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.ns = 'signal'
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = -.37
        marker.pose.position.y = -.455
        marker.pose.position.z = .52
        marker.pose.orientation.w = 1.
        marker.scale.x = marker.scale.y = marker.scale.z = .055
        marker.color.a = 1.
        if state == 'red':
            marker.color.r = 1.
        elif state == 'yellow':
            marker.color.r = 1.
            marker.color.g = .78
        else:
            marker.color.g = 1.
        self.marker.publish(marker)
        if self.client.service_is_ready() and self.visual_state != state:
            for i, color in enumerate(('red', 'yellow', 'green')):
                active = color == state
                self.set_bulb(f'signal_lens_{color}', -.405 if active else -.455,
                              .488-i*.05)
                self.set_bulb(f'signal_top_{color}', -.455,
                              .522 if active else .43)
            self.visual_state = state
        if state != self.last:
            self.get_logger().info(f'traffic light: {state}')
            self.last = state


def main():
    rclpy.init()
    node = Signal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
