#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__("trajectory_publisher")
        self.path_pub = self.create_publisher(
            Path, "/volcanibot_controller/trajectory", 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, "/volcanibot_controller/odom", self.odom_callback, 10
        )
        self.path = Path()
        self.path.header.frame_id = "odom"

    def odom_callback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.path.header.stamp = msg.header.stamp
        self.path.poses.append(pose)
        self.path_pub.publish(self.path)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
