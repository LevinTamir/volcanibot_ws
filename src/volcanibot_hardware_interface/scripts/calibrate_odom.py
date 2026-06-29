#!/usr/bin/env python3
"""Odometry calibration helper (read-only).

Subscribes to /joint_states and the controller odometry and, between two Enter
presses, reports how far each wheel turned, how far the robot moved, and how
much it rotated. It then prints the multiplier to apply to the relevant
calibration parameter, so you don't have to eyeball tf2_echo.

This node NEVER publishes - you drive the robot yourself (joystick, deadman
held). Run it alongside real_bringup:

    python3 calibrate_odom.py
    python3 calibrate_odom.py --odom /volcanibot_controller/odom

Workflow per measurement:
  1. Press Enter to mark START.
  2. Drive the robot (straight line, or a known in-place rotation, or roll one
     wheel a known number of turns with wheels off the ground).
  3. Press Enter to mark STOP, then type the ground-truth value when asked.

Suggested multipliers (apply to the value currently in your config):
  - counts_per_rev   *= reported_revs / actual_revs        (one-wheel roll)
  - wheel_radius     *= measured_distance / odom_distance  (straight line)
  - wheel_separation *= odom_yaw / actual_yaw              (in-place rotation)
"""

import argparse
import math
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class OdomCalib(Node):
    def __init__(self, joint_topic, odom_topic):
        super().__init__("odom_calib")
        self._joints = None   # dict name -> position
        self._odom = None     # (x, y, yaw)
        self.create_subscription(JointState, joint_topic, self._joint_cb, 10)
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
        self.get_logger().info(
            f"Listening on {joint_topic} and {odom_topic}")

    def _joint_cb(self, msg):
        self._joints = dict(zip(msg.name, msg.position))

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        self._odom = (p.x, p.y, yaw_from_quat(msg.pose.pose.orientation))

    def snapshot(self):
        return (dict(self._joints) if self._joints else None, self._odom)


def fmt_revs(d_rad):
    return f"{d_rad:+.3f} rad ({d_rad / (2.0 * math.pi):+.4f} rev)"


def report(start, stop):
    sj, so = start
    ej, eo = stop

    print("\n--- wheels ---")
    if sj and ej:
        for name in sorted(set(sj) & set(ej)):
            print(f"  {name}: {fmt_revs(ej[name] - sj[name])}")
    else:
        print("  (no /joint_states received)")

    print("--- odometry ---")
    if so and eo:
        dx, dy = eo[0] - so[0], eo[1] - so[1]
        dist = math.hypot(dx, dy)
        dyaw = math.degrees((eo[2] - so[2] + math.pi) % (2.0 * math.pi) - math.pi)
        print(f"  distance: {dist:.4f} m   (dx={dx:+.4f}, dy={dy:+.4f})")
        print(f"  yaw:      {dyaw:+.3f} deg")
    else:
        print("  (no /odom received)")
        return

    print("--- suggested multipliers (apply to your current config value) ---")
    g = input("  actual straight distance in m (blank to skip): ").strip()
    if g:
        try:
            print(f"    wheel_radius *= {float(g) / dist:.5f}")
        except (ValueError, ZeroDivisionError):
            print("    (skip)")
    g = input("  actual rotation in deg (blank to skip): ").strip()
    if g:
        try:
            print(f"    wheel_separation *= {dyaw / float(g):.5f}")
        except (ValueError, ZeroDivisionError):
            print("    (skip)")
    g = input("  wheel turns rolled, for counts_per_rev (blank to skip): ").strip()
    if g and sj and ej:
        try:
            turns = float(g)
            # Average the rear wheels' reported revolutions.
            revs = [(ej[n] - sj[n]) / (2.0 * math.pi)
                    for n in set(sj) & set(ej) if "rear" in n]
            if revs and turns:
                avg = sum(revs) / len(revs)
                print(f"    counts_per_rev *= {avg / turns:.5f}")
        except (ValueError, ZeroDivisionError):
            print("    (skip)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--joints", default="/joint_states")
    ap.add_argument("--odom", default="/volcanibot_controller/odom")
    args = ap.parse_args()

    rclpy.init()
    node = OdomCalib(args.joints, args.odom)
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    print("Waiting for first messages...")
    try:
        while rclpy.ok() and node.snapshot() == (None, None):
            pass
        print("Ready.")
        while rclpy.ok():
            input("\nPress Enter to mark START (Ctrl-C to quit)...")
            start = node.snapshot()
            input("Drive, then press Enter to mark STOP...")
            report(start, node.snapshot())
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
