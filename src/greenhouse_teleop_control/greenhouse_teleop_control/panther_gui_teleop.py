import tkinter as tk
from tkinter import ttk

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class PantherGuiTeleop(Node):
    def __init__(self):
        super().__init__("panther_gui_teleop")

        self.declare_parameter("cmd_vel_topic", "/panther/cmd_vel")
        self.declare_parameter("linear_speed", 0.5)
        self.declare_parameter("angular_speed", 0.8)
        self.declare_parameter("publish_rate", 10.0)

        self.cmd_vel_topic = (
            self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        )
        self.linear_speed = (
            self.get_parameter("linear_speed").get_parameter_value().double_value
        )
        self.angular_speed = (
            self.get_parameter("angular_speed").get_parameter_value().double_value
        )
        publish_rate = self.get_parameter("publish_rate").get_parameter_value().double_value
        self.publish_period_ms = max(20, int(1000.0 / publish_rate))

        self.publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.linear_x = 0.0
        self.angular_z = 0.0
        self.active_keys = set()

        self.root = tk.Tk()
        self.root.title("Panther Teleop")
        self.root.geometry("430x360")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.linear_var = tk.DoubleVar(value=self.linear_speed)
        self.angular_var = tk.DoubleVar(value=self.angular_speed)
        self.status_var = tk.StringVar(value=f"Publishing to {self.cmd_vel_topic}")

        self._build_ui()
        self._bind_keys()
        self.root.after(self.publish_period_ms, self.publish_loop)

    def _build_ui(self):
        outer = ttk.Frame(self.root, padding=12)
        outer.grid(row=0, column=0, sticky="nsew")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        title = ttk.Label(outer, text="Panther Teleop", font=("TkDefaultFont", 14, "bold"))
        title.grid(row=0, column=0, columnspan=3, pady=(0, 10))

        ttk.Label(outer, textvariable=self.status_var).grid(
            row=1, column=0, columnspan=3, sticky="w", pady=(0, 10)
        )

        forward = ttk.Button(outer, text="Forward (W)")
        left = ttk.Button(outer, text="Left (A)")
        stop = ttk.Button(outer, text="Stop (Space)")
        right = ttk.Button(outer, text="Right (D)")
        reverse = ttk.Button(outer, text="Reverse (S)")

        forward.grid(row=2, column=1, padx=5, pady=5, sticky="ew")
        left.grid(row=3, column=0, padx=5, pady=5, sticky="ew")
        stop.grid(row=3, column=1, padx=5, pady=5, sticky="ew")
        right.grid(row=3, column=2, padx=5, pady=5, sticky="ew")
        reverse.grid(row=4, column=1, padx=5, pady=5, sticky="ew")

        self._bind_button(forward, "w")
        self._bind_button(left, "a")
        self._bind_button(right, "d")
        self._bind_button(reverse, "s")
        stop.configure(command=self.stop_motion)

        ttk.Label(outer, text="Linear speed").grid(row=5, column=0, sticky="w", pady=(18, 0))
        linear_scale = ttk.Scale(
            outer,
            from_=0.05,
            to=2.0,
            orient="horizontal",
            variable=self.linear_var,
        )
        linear_scale.grid(row=5, column=1, columnspan=2, sticky="ew", pady=(18, 0))

        ttk.Label(outer, text="Angular speed").grid(row=6, column=0, sticky="w", pady=(10, 0))
        angular_scale = ttk.Scale(
            outer,
            from_=0.05,
            to=4.0,
            orient="horizontal",
            variable=self.angular_var,
        )
        angular_scale.grid(row=6, column=1, columnspan=2, sticky="ew", pady=(10, 0))

        hint = ttk.Label(
            outer,
            text="Click this window, then use W/A/S/D. Hold buttons or keys to move.",
            wraplength=380,
        )
        hint.grid(row=7, column=0, columnspan=3, pady=(18, 0), sticky="w")

        for col in range(3):
            outer.columnconfigure(col, weight=1)

    def _bind_button(self, button, key):
        button.bind("<ButtonPress-1>", lambda _event: self.press_key(key))
        button.bind("<ButtonRelease-1>", lambda _event: self.release_key(key))

    def _bind_keys(self):
        self.root.bind("<KeyPress>", self.on_key_press)
        self.root.bind("<KeyRelease>", self.on_key_release)
        self.root.focus_force()

    def on_key_press(self, event):
        key = event.keysym.lower()
        if key in ("w", "a", "s", "d"):
            self.press_key(key)
        elif key in ("space", "x"):
            self.stop_motion()

    def on_key_release(self, event):
        key = event.keysym.lower()
        if key in ("w", "a", "s", "d"):
            self.release_key(key)

    def press_key(self, key):
        self.active_keys.add(key)
        self.update_motion()

    def release_key(self, key):
        self.active_keys.discard(key)
        self.update_motion()

    def update_motion(self):
        linear_speed = float(self.linear_var.get())
        angular_speed = float(self.angular_var.get())

        linear = 0.0
        angular = 0.0

        if "w" in self.active_keys:
            linear += linear_speed
        if "s" in self.active_keys:
            linear -= linear_speed
        if "a" in self.active_keys:
            angular += angular_speed
        if "d" in self.active_keys:
            angular -= angular_speed

        self.linear_x = linear
        self.angular_z = angular
        self.status_var.set(
            f"cmd_vel: linear.x={self.linear_x:.2f}, angular.z={self.angular_z:.2f}"
        )

    def publish_loop(self):
        self.publish_twist()
        self.root.after(self.publish_period_ms, self.publish_loop)

    def publish_twist(self):
        msg = Twist()
        msg.linear.x = self.linear_x
        msg.angular.z = self.angular_z
        self.publisher.publish(msg)

    def stop_motion(self):
        self.active_keys.clear()
        self.linear_x = 0.0
        self.angular_z = 0.0
        self.publish_twist()
        self.status_var.set("Stopped")

    def on_close(self):
        self.stop_motion()
        self.root.after(100, self.root.destroy)

    def run(self):
        self.get_logger().info(f"Publishing Twist commands to {self.cmd_vel_topic}")
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = PantherGuiTeleop()
    try:
        node.run()
    finally:
        node.stop_motion()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
