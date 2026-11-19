#!/usr/bin/env python3
"""Keyboard teleop for the Volcanibot: arrow keys or WASD, hold to drive.

Publishes geometry_msgs/TwistStamped on /joy_vel (the twist_mux joystick
input, priority 100), so it overrides autonomy exactly like the gamepad.

Hold-to-drive: a terminal only sees key presses and auto-repeats, never key
releases. The first press is held for `first_press_hold` seconds (to bridge
the OS auto-repeat delay, 500 ms by default on GNOME); once repeats arrive,
the command expires `repeat_hold` seconds after the last one. On release the
node sends zeros briefly and then goes silent, so twist_mux can hand control
back to lower-priority sources (/nav_vel).

    ros2 run volcanibot_controller keyboard_teleop.py
    # in simulation add:  --ros-args -p use_sim_time:=true
"""

import os
import select
import sys
import termios
import threading
import time
import tty

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node

HELP = """
Volcanibot keyboard teleop  (publishing on /{topic})
-----------------------------------------------------
  W / Up      forward          Q   forward + left
  S / Down    backward         E   forward + right
  A / Left    turn left
  D / Right   turn right       Space / X   stop now

  + / -   speed up / slow down (both)
  ] / [   linear speed only        } / {   angular speed only
  Ctrl-C  quit

Hold a key to drive; release it and the robot stops.
"""

# key -> (linear sign, angular sign)
MOVES = {
    "w": (1, 0), "UP": (1, 0),
    "s": (-1, 0), "DOWN": (-1, 0),
    "a": (0, 1), "LEFT": (0, 1),
    "d": (0, -1), "RIGHT": (0, -1),
    "q": (1, 1),
    "e": (1, -1),
}
ARROWS = {"A": "UP", "B": "DOWN", "C": "RIGHT", "D": "LEFT"}


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__("keyboard_teleop")
        p = self.declare_parameter
        self.topic = p("topic", "joy_vel").value
        self.frame_id = p("frame_id", "base_footprint").value
        self.linear = p("linear_speed", 0.5).value
        self.angular = p("angular_speed", 0.8).value
        self.max_linear = p("max_linear_speed", 2.0).value
        self.max_angular = p("max_angular_speed", 1.7).value
        self.step = p("speed_step", 0.1).value
        self.first_hold = p("first_press_hold", 0.6).value
        self.repeat_hold = p("repeat_hold", 0.15).value
        self.rate = p("publish_rate", 20.0).value
        self.zero_tail = p("stop_publish_time", 0.3).value

        self.pub = self.create_publisher(TwistStamped, self.topic, 10)
        self.lock = threading.Lock()
        self.move = (0, 0)
        self.last_key = None
        self.last_key_time = 0.0
        self.deadline = 0.0
        self.stopped_at = None  # when the command expired; None = silent
        self.create_timer(1.0 / self.rate, self.tick)

    # ---- key handling (called from the input thread) ----
    def on_key(self, key):
        now = time.monotonic()
        with self.lock:
            if key in MOVES:
                repeating = (key == self.last_key and now < self.deadline)
                self.move = MOVES[key]
                self.deadline = now + (self.repeat_hold if repeating else self.first_hold)
                self.last_key, self.last_key_time = key, now
                self.stopped_at = None
            elif key in (" ", "x"):
                self.stop_now(now)
            elif key in ("+", "="):
                self.linear = self.clamp(self.linear + self.step, self.max_linear)
                self.angular = self.clamp(self.angular + self.step, self.max_angular)
            elif key in ("-", "_"):
                self.linear = self.clamp(self.linear - self.step, self.max_linear)
                self.angular = self.clamp(self.angular - self.step, self.max_angular)
            elif key == "]":
                self.linear = self.clamp(self.linear + self.step, self.max_linear)
            elif key == "[":
                self.linear = self.clamp(self.linear - self.step, self.max_linear)
            elif key == "}":
                self.angular = self.clamp(self.angular + self.step, self.max_angular)
            elif key == "{":
                self.angular = self.clamp(self.angular - self.step, self.max_angular)
        self.status()

    def stop_now(self, now):
        self.move = (0, 0)
        self.deadline = 0.0
        self.last_key = None
        self.stopped_at = now

    @staticmethod
    def clamp(v, vmax):
        return round(min(max(v, 0.0), vmax), 3)

    # ---- publishing (timer) ----
    def tick(self):
        now = time.monotonic()
        with self.lock:
            if self.move != (0, 0) and now > self.deadline:
                self.stop_now(now)
                expired = True
            else:
                expired = False
            if self.move != (0, 0):
                lin, ang = self.move[0] * self.linear, self.move[1] * self.angular
            elif self.stopped_at is not None and now - self.stopped_at < self.zero_tail:
                lin, ang = 0.0, 0.0
            else:
                self.stopped_at = None
                return  # silent: let twist_mux fall back to other sources
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.twist.linear.x = float(lin)
        msg.twist.angular.z = float(ang)
        self.pub.publish(msg)
        if expired:
            self.status()

    def status(self):
        lin, ang = self.move
        state = {(0, 0): "stopped"}.get((lin, ang), f"lin {lin:+d} ang {ang:+d}")
        sys.stdout.write(
            f"\r  speed: linear {self.linear:.2f} m/s  angular {self.angular:.2f} rad/s"
            f"   [{state:^16}]   ")
        sys.stdout.flush()


def read_key(fd):
    """Return one key name, or None on timeout. Decodes arrow escape codes."""
    r, _, _ = select.select([fd], [], [], 0.1)
    if not r:
        return None
    ch = os.read(fd, 1).decode(errors="ignore")
    if ch == "\x1b":
        seq = ""
        while len(seq) < 2 and select.select([fd], [], [], 0.01)[0]:
            seq += os.read(fd, 1).decode(errors="ignore")
        return ARROWS.get(seq[1:2]) if seq[:1] in ("[", "O") else None
    if ch == "\x03":
        raise KeyboardInterrupt
    return ch.lower() if ch.isalpha() else ch


def main(args=None):
    if not sys.stdin.isatty():
        print("keyboard_teleop needs an interactive terminal.", file=sys.stderr)
        return 1
    rclpy.init(args=args)
    node = KeyboardTeleop()
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    print(HELP.replace("{topic}", node.topic))
    node.status()
    try:
        tty.setcbreak(fd)
        while rclpy.ok():
            key = read_key(fd)
            if key is not None:
                node.on_key(key)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        # Explicit stop before leaving.
        with node.lock:
            node.stop_now(time.monotonic())
        msg = TwistStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = node.frame_id
        for _ in range(3):
            node.pub.publish(msg)
            time.sleep(0.05)
        print("\nkeyboard_teleop: stopped.")
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
