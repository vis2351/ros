"""Publish an RViz-compatible destination from a terminal."""
import argparse
import math
import time

from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
import rclpy


def main():
    parser=argparse.ArgumentParser(description='Send a sand-table drive goal')
    parser.add_argument('x',type=float)
    parser.add_argument('y',type=float)
    parser.add_argument('--frame',choices=['odom','map'],default='odom')
    parser.add_argument('--yaw',type=float,default=0.,
                        help='Goal heading in degrees (parking orientation is planned)')
    args=parser.parse_args()
    rclpy.init()
    node=Node('sand_table_goal_cli')
    publisher=node.create_publisher(PoseStamped,'/goal_pose',10)
    message=PoseStamped()
    message.header.frame_id=args.frame
    message.pose.position.x=args.x
    message.pose.position.y=args.y
    yaw=math.radians(args.yaw)/2
    message.pose.orientation.z=math.sin(yaw)
    message.pose.orientation.w=math.cos(yaw)
    deadline=time.monotonic()+3.
    while publisher.get_subscription_count()==0 and time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.1)
    message.header.stamp=node.get_clock().now().to_msg()
    publisher.publish(message)
    rclpy.spin_once(node,timeout_sec=.3)
    node.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':
    main()
