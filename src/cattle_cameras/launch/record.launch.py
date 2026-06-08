import re
import subprocess
from datetime import datetime

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


# Topic suffixes published by each realsense camera, relative to its namespace.
CAMERA_TOPIC_SUFFIXES = [
    "camera/color/image_raw/compressed",
    "camera/depth/image_rect_raw/compressedDepth",
    "camera/color/camera_info",
    "camera/depth/camera_info",
]


def get_serials():
    """Return the serial numbers of all connected RealSense cameras."""
    try:
        out = subprocess.run(
            ["rs-enumerate-devices", "-s"],
            capture_output=True,
            text=True,
            check=True,
        ).stdout
    except (FileNotFoundError, subprocess.CalledProcessError):
        return []

    return re.findall(r"\d{8,}", out)


def generate_launch_description():
    serials = get_serials()

    nodes = []
    camera_names = []
    for serial in serials:
        camera_name = f"camera_{serial}"
        camera_names.append(camera_name)
        node = Node(
            package="realsense2_camera",
            executable="realsense2_camera_node",
            name="camera",
            namespace=camera_name,
            parameters=[
                {
                    "serial_no": serial,
                    # Disable to prevent permission errors
                    "enable_gyro": False,
                    "enable_accel": False,
                    # Might need to turn off depending on usb bandwith
                    "enable_infra1": True,
                    "enable_infra2": True,
                }
            ],
        )
        nodes.append(node)

    if camera_names:
        topics = [
            f"/{name}/{suffix}"
            for name in camera_names
            for suffix in CAMERA_TOPIC_SUFFIXES
        ]
        bag_name = f"realsense_data_{datetime.now():%Y%m%d_%H%M%S}"
        nodes.append(
            ExecuteProcess(
                cmd=["ros2", "bag", "record", "-o", bag_name, *topics],
                output="screen",
            )
        )

    return LaunchDescription(nodes)
