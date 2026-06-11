import os
import re
import subprocess
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Camera topics recorded to the bag, relative to each camera's namespace.
RECORDED_TOPIC_SUFFIXES = (
    "camera/color/image_raw/compressed",
    "camera/depth/image_rect_raw/compressedDepth",
    "camera/color/camera_info",
    "camera/depth/camera_info",
)


def declare_args():
    """User-facing launch arguments and their defaults."""
    return [
        # `yolo`/`view` are evaluated by IfCondition -> lowercase true/false.
        DeclareLaunchArgument(
            "yolo",
            default_value="true",
            description="Run a per-camera YOLO detector (on by default).",
        ),
        DeclareLaunchArgument(
            "view",
            default_value="false",
            description="Open an OpenCV window per camera showing detections "
            "(needs yolo:=true and a display).",
        ),
        # `use_tracking`/`use_3d` are eval()-ed inside yolo_ros -> capitalized True/False.
        DeclareLaunchArgument(
            "use_tracking",
            default_value="True",
            description="Track detections across frames with persistent IDs "
            "(on by default). Set False for plain per-frame detection.",
        ),
        DeclareLaunchArgument(
            "use_3d",
            default_value="False",
            description="Publish 3D detections from the depth stream.",
        ),
        DeclareLaunchArgument(
            "tracker",
            default_value="botsort.yaml",
            description="Tracker config. Defaults to the built-in BoT-SORT settings. "
            "Pass a built-in name ('bytetrack.yaml'), a bare filename from this "
            "package's config/ (e.g. 'cattle_botsort.yaml'), or an absolute path.",
        ),
        DeclareLaunchArgument(
            "model",
            default_value="yolo11m.pt",
            description="YOLO model name or path. Pass a built-in name "
            "('yolo11m.pt', auto-downloaded), a bare filename from this package's "
            "weights/ dir (e.g. 'cow_status.pt'), or an absolute path to a "
            "trained .pt file.",
        ),
        DeclareLaunchArgument(
            "device",
            default_value="cuda:0",
            description="Inference device, e.g. cuda:0 or cpu.",
        ),
        DeclareLaunchArgument(
            "threshold",
            default_value="0.5",
            description="Detection confidence threshold.",
        ),
    ]


def resolve_tracker(value):
    """Resolve the `tracker` arg to something yolo_ros/Ultralytics can load.

    A bare filename that exists in this package's installed config/ dir is
    expanded to its absolute path, so you can pass `tracker:=cattle_botsort.yaml`
    instead of the full install path. An absolute path, or a built-in Ultralytics
    name like `botsort.yaml`, is passed through unchanged.
    """
    if os.path.isabs(value) or os.sep in value:
        return value
    local = os.path.join(
        get_package_share_directory("cattle_cameras"), "config", value
    )
    return local if os.path.exists(local) else value


def resolve_model(value):
    """Resolve the `model` arg to a path or name YOLO can load.

    A bare filename that exists in this package's installed weights/ dir is
    expanded to its absolute path, so you can pass `model:=cow_status.pt`
    instead of the full install path. An absolute path, or a built-in
    Ultralytics name like `yolo11m.pt` (auto-downloaded), is passed through
    unchanged.
    """
    if os.path.isabs(value) or os.sep in value:
        return value
    local = os.path.join(
        get_package_share_directory("cattle_cameras"), "weights", value
    )
    return local if os.path.exists(local) else value


def get_serials():
    """Serial numbers of all connected RealSense cameras."""
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


def realsense_node(serial, camera_name):
    """RealSense driver for a single camera."""
    return Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        name="camera",
        namespace=camera_name,
        parameters=[
            {
                "serial_no": serial,
                # Disabled to avoid permission errors.
                "enable_gyro": False,
                "enable_accel": False,
                # May need to disable on limited USB bandwidth.
                "enable_infra1": True,
                "enable_infra2": True,
            }
        ],
    )


def yolo_stack(camera_name, tracker, model):
    """Per-camera YOLO detector/tracker, gated on the `yolo` arg."""
    yolo_launch = os.path.join(
        get_package_share_directory("yolo_bringup"), "launch", "yolo.launch.py"
    )
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(yolo_launch),
        condition=IfCondition(LaunchConfiguration("yolo")),
        launch_arguments={
            "namespace": f"{camera_name}/yolo",
            "input_image_topic": f"/{camera_name}/camera/color/image_raw",
            "input_depth_topic": f"/{camera_name}/camera/depth/image_rect_raw",
            "input_depth_info_topic": f"/{camera_name}/camera/depth/camera_info",
            "model": model,
            "device": LaunchConfiguration("device"),
            "use_tracking": LaunchConfiguration("use_tracking"),
            "tracker": tracker,
            "threshold": LaunchConfiguration("threshold"),
            "use_3d": LaunchConfiguration("use_3d"),
        }.items(),
    )


def viewer_node(camera_name):
    """OpenCV window of this camera's annotated detections, gated on `view`."""
    return Node(
        package="cattle_cameras",
        executable="detection_viewer",
        name="detection_viewer",
        namespace=camera_name,
        condition=IfCondition(LaunchConfiguration("view")),
        parameters=[{"image_topic": f"/{camera_name}/yolo/dbg_image"}],
        output="screen",
    )


def bag_recorder(camera_names):
    """Record every camera's color+depth streams; returns (bag_name, action)."""
    topics = [
        f"/{name}/{suffix}"
        for name in camera_names
        for suffix in RECORDED_TOPIC_SUFFIXES
    ]
    bag_name = f"realsense_data_{datetime.now():%Y%m%d_%H%M%S}"
    action = ExecuteProcess(
        cmd=["ros2", "bag", "record", "-o", bag_name, *topics],
        output="screen",
    )
    return bag_name, action


def build_nodes(context, *_args, **_kwargs):
    """Assemble the per-camera nodes once launch arguments are resolvable."""
    tracker = resolve_tracker(LaunchConfiguration("tracker").perform(context))
    model = resolve_model(LaunchConfiguration("model").perform(context))

    serials = get_serials()
    camera_names = [f"camera_{serial}" for serial in serials]

    nodes = []
    for serial, camera_name in zip(serials, camera_names):
        nodes += [
            realsense_node(serial, camera_name),
            yolo_stack(camera_name, tracker, model),
            viewer_node(camera_name),
        ]

    if camera_names:
        bag_name, recorder = bag_recorder(camera_names)
        nodes += [
            recorder,
            LogInfo(
                msg=f"Recording {len(camera_names)} camera(s) to '{bag_name}': "
                f"{', '.join(camera_names)}"
            ),
        ]
    else:
        nodes.append(
            LogInfo(
                msg="No RealSense cameras detected (rs-enumerate-devices found none). "
                "Nothing to record or detect, so the launch will exit. Check the USB3 "
                "connection and that 'rs-enumerate-devices -s' lists a serial."
            )
        )
    return nodes


def generate_launch_description():
    return LaunchDescription([*declare_args(), OpaqueFunction(function=build_nodes)])
