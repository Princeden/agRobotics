#!/usr/bin/env python3
"""Publish a properly time-stamped TwistStamped jog to MoveIt Servo.

`ros2 topic pub` leaves header.stamp=0, which Servo (with use_sim_time) treats as
a command older than incoming_command_timeout and silently ignores. This node
stamps every message with the (sim) clock, so Servo actually acts on it.

Examples:
  python3 jog.py                 # camera +z (optical axis), 0.05 m/s
  python3 jog.py --lx 0.05       # camera +x (right)
  python3 jog.py --az 0.2        # rotate about camera +z
  python3 jog.py --lz -0.05      # back off along optical axis
Ctrl-C to stop (Servo then halts on command timeout).
"""
import argparse

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import TwistStamped


class Jog(Node):
    def __init__(self, a):
        super().__init__("servo_jog")
        # Match Servo's clock so the stamps are comparable.
        self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.a = a
        self.pub = self.create_publisher(TwistStamped, a.topic, 10)
        self.create_timer(1.0 / a.rate, self.tick)
        self.get_logger().info(
            f"jogging {a.topic} in '{a.frame}' "
            f"lin=[{a.lx},{a.ly},{a.lz}] ang=[{a.ax},{a.ay},{a.az}] @ {a.rate} Hz"
        )

    def tick(self):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.a.frame
        m.twist.linear.x, m.twist.linear.y, m.twist.linear.z = self.a.lx, self.a.ly, self.a.lz
        m.twist.angular.x, m.twist.angular.y, m.twist.angular.z = self.a.ax, self.a.ay, self.a.az
        self.pub.publish(m)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--topic", default="/servo_node/delta_twist_cmds")
    p.add_argument("--frame", default="camera_color_optical_frame")
    p.add_argument("--rate", type=float, default=50.0)
    for ax in ("lx", "ly", "lz", "ax", "ay", "az"):
        p.add_argument(f"--{ax}", type=float, default=0.0)
    a = p.parse_args()
    if a.lx == a.ly == a.lz == a.ax == a.ay == a.az == 0.0:
        a.lz = 0.05  # sensible default jog
    rclpy.init()
    node = Jog(a)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
