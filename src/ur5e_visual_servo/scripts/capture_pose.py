#!/usr/bin/env python3
"""Capture the arm's CURRENT joint configuration into servo_start_positions.yaml.

Use this instead of computing a pose offline: jog the arm in the running sim until
the wrist camera sees the marker nicely (centered, status 0), then run this to
freeze that exact configuration as the start pose. It reads the live /joint_states,
so the pose is guaranteed consistent with the real robot model.

Usage:
  python3 capture_pose.py                       # print only
  python3 capture_pose.py --write               # also write config/servo_start_positions.yaml
"""
import os
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

ARM_JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
              "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class Capture(Node):
    def __init__(self):
        super().__init__("capture_pose")
        self.sub = self.create_subscription(JointState, "/joint_states", self.cb, 10)
        self.done = False

    def cb(self, msg):
        pos = dict(zip(msg.name, msg.position))
        if not all(j in pos for j in ARM_JOINTS):
            return  # wait for a message containing all arm joints
        self.values = {j: pos[j] for j in ARM_JOINTS}
        self.done = True


def main():
    rclpy.init()
    node = Capture()
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=1.0)
    v = node.values
    print("captured current pose:")
    for j in ARM_JOINTS:
        print(f"  {j:<20}{v[j]: .4f}")
    if "--write" in sys.argv:
        path = os.path.join(PKG, "config", "servo_start_positions.yaml")
        with open(path, "w") as f:
            f.write("# Captured from the live sim by scripts/capture_pose.py\n")
            f.write("# (arm jogged until the wrist camera framed the marker).\n")
            for j in ARM_JOINTS:
                f.write(f"{j}: {v[j]:.4f}\n")
        print(f"\nwrote {path}  (rebuild ur5e_visual_servo to install)")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
