#!/usr/bin/env python3
"""Publish the robot's travelled path as nav_msgs/Path for RViz. Read-only.

diff_drive_controller publishes /odom (Odometry), which RViz can only show as a
single moving pose. This node accumulates those poses into a Path so you can see
the trail base_footprint made through the odom frame.

Run alongside the robot:
    python3 odom_path.py
    python3 odom_path.py --ros-args -p odom_topic:=/volcanibot_controller/odom

Then in RViz: Add -> Path, set Topic to /odom_path (Fixed Frame = odom).
"""

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class OdomPath(Node):
    def __init__(self):
        super().__init__("odom_path")
        self.declare_parameter("odom_topic", "/volcanibot_controller/odom")
        self.declare_parameter("min_dist", 0.05)   # m between stored points
        topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        self._min_dist = self.get_parameter("min_dist").get_parameter_value().double_value

        self._path = Path()
        self._last = None
        self.create_subscription(Odometry, topic, self._cb, 10)
        self._pub = self.create_publisher(Path, "/odom_path", 10)
        self.get_logger().info(f"Publishing /odom_path from {topic}")

    def _cb(self, msg):
        p = msg.pose.pose.position
        if self._last is not None and \
           math.hypot(p.x - self._last[0], p.y - self._last[1]) < self._min_dist:
            return
        self._last = (p.x, p.y)
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = msg.pose.pose
        self._path.header = msg.header
        self._path.poses.append(ps)
        self._pub.publish(self._path)


def main():
    rclpy.init()
    rclpy.spin(OdomPath())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
