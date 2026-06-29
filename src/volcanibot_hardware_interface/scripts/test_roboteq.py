#!/usr/bin/env python3
"""Roboteq quick test - sends a short motor command sequence over serial.

⚠️  BENCH-ONLY. This SENDS MOTOR COMMANDS directly to the Roboteq, bypassing
ros2_control and all safety arbitration (twist_mux / deadman). Only run it on a
bench with the wheels off the ground (or motor power disabled), never on the
deployed robot without explicit sign-off.

Usage:
    python3 test_roboteq.py                 # canned forward/stop/read sequence
    python3 test_roboteq.py "!S 1 50"       # send one raw command
    ROBOTEQ_PORT=/dev/ttyACM0 python3 test_roboteq.py   # override port

Talks to the stable /dev/roboteq symlink created by the udev rule in
volcanibot_hardware_interface/udev/.
"""

import os
import sys
import time

import serial

PORT = os.environ.get("ROBOTEQ_PORT", "/dev/roboteq")
BAUD = int(os.environ.get("ROBOTEQ_BAUD", "115200"))
TIMEOUT = 1.0


def send_command(ser, command):
    """Send a command and read the response."""
    ser.write((command + "\r").encode())
    time.sleep(0.1)
    response = ser.read(100).decode("utf-8", errors="ignore")
    return response.strip()


def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
        print(f"Connected to {PORT} at {BAUD} baud")
        time.sleep(2)  # let the link stabilize

        print("Turning echo off...")
        send_command(ser, "^ECHOF 1")

        print("\nForward (motor 1 + 2 at 100 RPM)...")
        print(f"Motor 1 response: {send_command(ser, '!S 1 100')}")
        print(f"Motor 2 response: {send_command(ser, '!S 2 100')}")
        time.sleep(2)

        print("\nStopping motors...")
        send_command(ser, "!S 1 0")
        send_command(ser, "!S 2 0")

        print("\nReading current RPM...")
        print(f"RPM response: {send_command(ser, '?S')}")

        ser.close()
        print("\nDisconnected")

    except serial.SerialException as e:
        print(f"Serial error: {e}")
        print(f"Make sure {PORT} exists and you have permissions.")
    except KeyboardInterrupt:
        print("\nStopping motors...")
        try:
            send_command(ser, "!S 1 0")
            send_command(ser, "!S 2 0")
            ser.close()
        except Exception:
            pass
        print("Exited")


if __name__ == "__main__":
    if len(sys.argv) > 1:
        ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
        time.sleep(2)
        command = sys.argv[1]
        print(f"Command: {command}")
        print(f"Response: {send_command(ser, command)}")
        ser.close()
    else:
        main()
