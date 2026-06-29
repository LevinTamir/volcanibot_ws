#!/usr/bin/env python3
"""Interactive Roboteq console - type raw commands, read responses.

⚠️  BENCH-ONLY. This SENDS MOTOR COMMANDS directly to the Roboteq, bypassing
ros2_control and all safety arbitration (twist_mux / deadman). Only run it on a
bench with the wheels off the ground (or motor power disabled), never on the
deployed robot without explicit sign-off.

Usage:
    python3 interactive_roboteq.py
    ROBOTEQ_PORT=/dev/ttyACM0 python3 interactive_roboteq.py   # override port

Talks to the stable /dev/roboteq symlink created by the udev rule in
volcanibot_hardware_interface/udev/. Type 'quit' to exit (motors are stopped
on exit).
"""

import os
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
        print(f"Connecting to {PORT} at {BAUD} baud...")
        ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
        time.sleep(2)  # let the link stabilize
        print("Connected! Type commands (or 'quit' to exit)")
        print("Examples: !S 1 100, !S 2 100, ?S, !S 1 0")
        print("-" * 50)

        send_command(ser, "^ECHOF 1")  # echo off

        while True:
            try:
                command = input("Roboteq> ").strip()
                if command.lower() in ("quit", "exit", "q"):
                    break
                if not command:
                    continue
                response = send_command(ser, command)
                print(f"Response: {response}" if response else "(No response)")
            except KeyboardInterrupt:
                break

        print("\nStopping motors...")
        send_command(ser, "!S 1 0")
        send_command(ser, "!S 2 0")
        ser.close()
        print("Disconnected")

    except serial.SerialException as e:
        print(f"Error: {e}")
        print(f"Make sure {PORT} exists and you have permissions.")
    except Exception as e:
        print(f"Error: {e}")


if __name__ == "__main__":
    main()
