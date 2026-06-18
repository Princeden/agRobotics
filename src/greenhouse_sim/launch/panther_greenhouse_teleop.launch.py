import os
import tempfile
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import SetEnvironmentVariable
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString


LIDAR_TYPES_BY_CHANNELS = {
    16: "LDR20",
    32: "LDR10",
    64: "LDR11",
    128: "LDR12",
}


def _sensor_component(sensor_config, component_type):
    component = {
        "type": component_type,
        "parent_link": sensor_config["parent_link"],
        "xyz": sensor_config.get("xyz", "0.0 0.0 0.0"),
        "rpy": sensor_config.get("rpy", "0.0 0.0 0.0"),
    }
    device_namespace = sensor_config.get("device_namespace", sensor_config.get("name", ""))
    if device_namespace:
        component["device_namespace"] = device_namespace
    return component


def _make_components_config(sensor_config_path):
    with open(sensor_config_path, "r", encoding="utf-8") as config_file:
        sensor_config = yaml.safe_load(config_file) or {}

    components = []

    lidar = sensor_config.get("lidar", {})
    if lidar.get("enabled", True):
        channels = int(lidar.get("channels", 32))
        try:
            lidar_type = LIDAR_TYPES_BY_CHANNELS[channels]
        except KeyError as exc:
            supported = ", ".join(str(value) for value in sorted(LIDAR_TYPES_BY_CHANNELS))
            raise RuntimeError(
                f"Unsupported lidar.channels={channels}. Supported values: {supported}."
            ) from exc
        components.append(_sensor_component(lidar, lidar_type))

    camera = sensor_config.get("camera", {})
    if camera.get("enabled", True):
        components.append(_sensor_component(camera, camera.get("type", "CAM01")))

    output_path = Path(tempfile.gettempdir()) / "greenhouse_panther_components.yaml"
    with open(output_path, "w", encoding="utf-8") as generated_file:
        yaml.safe_dump({"components": components}, generated_file, sort_keys=False)

    return str(output_path)


def _make_resolved_world(world_path, model_path):
    with open(world_path, "r", encoding="utf-8") as world_file:
        world = world_file.read()

    for model_name in ("greenhouse_shelf", "greenhouse_shelf_metal"):
        world = world.replace(
            f"model://{model_name}",
            f"file://{Path(model_path, model_name).as_posix()}",
        )

    output_path = Path(tempfile.gettempdir()) / "greenhouse_resolved.world"
    with open(output_path, "w", encoding="utf-8") as resolved_file:
        resolved_file.write(world)

    return str(output_path)


def _make_namespaced_gui_config(gz_gui_path, namespace):
    if not gz_gui_path:
        return ""

    with open(gz_gui_path, "r", encoding="utf-8") as gui_file:
        gui_config = gui_file.read()

    gui_config = gui_config.replace("{namespace}", namespace.strip("/"))

    output_path = Path(tempfile.gettempdir()) / "greenhouse_panther_gui.config"
    with open(output_path, "w", encoding="utf-8") as generated_file:
        generated_file.write(gui_config)

    return str(output_path)


def _launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("greenhouse_sim")
    sensor_config_path = LaunchConfiguration("sensor_config").perform(context)
    components_config_path = _make_components_config(sensor_config_path)

    namespace = LaunchConfiguration("namespace")
    namespace_value = LaunchConfiguration("namespace").perform(context)
    log_level = LaunchConfiguration("log_level")
    gz_gui = LaunchConfiguration("gz_gui").perform(context)
    gz_log_level = LaunchConfiguration("gz_log_level").perform(context)

    model_path = os.path.join(pkg_share, "models")
    world_path = LaunchConfiguration("world").perform(context)
    resolved_world_path = _make_resolved_world(world_path, model_path)

    sim_resource_path = os.pathsep.join(
        path for path in [model_path, os.environ.get("GZ_SIM_RESOURCE_PATH", "")] if path
    )
    ign_resource_path = os.pathsep.join(
        path for path in [model_path, os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")] if path
    )
    gazebo_model_path = os.pathsep.join(
        path for path in [model_path, os.environ.get("GAZEBO_MODEL_PATH", "")] if path
    )

    gz_args = f"-r -v {gz_log_level} {resolved_world_path}"
    namespaced_gz_gui = _make_namespaced_gui_config(gz_gui, namespace_value)
    if namespaced_gz_gui:
        gz_args = f"--gui-config {namespaced_gz_gui} {gz_args}"

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={"gz_args": gz_args}.items(),
    )

    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("husarion_ugv_description"),
                "/launch/load_urdf.launch.py",
            ]
        ),
        launch_arguments={
            "namespace": namespace,
            "robot_model": "panther",
            "components_config_path": components_config_path,
            "use_sim": "True",
            "log_level": log_level,
        }.items(),
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        namespace=namespace,
        arguments=[
            "-name",
            namespace,
            "-topic",
            "robot_description",
            "-x",
            LaunchConfiguration("x"),
            "-y",
            LaunchConfiguration("y"),
            "-z",
            LaunchConfiguration("z"),
            "-R",
            LaunchConfiguration("roll"),
            "-P",
            LaunchConfiguration("pitch"),
            "-Y",
            LaunchConfiguration("yaw"),
        ],
        output="screen",
        emulate_tty=True,
    )

    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("husarion_ugv_controller"),
                "/launch/controller.launch.py",
            ]
        ),
        launch_arguments={
            "namespace": namespace,
            "robot_model": "panther",
            "components_config_path": components_config_path,
            "publish_robot_state": "False",
            "use_sim": "True",
            "log_level": log_level,
        }.items(),
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("husarion_ugv_localization"),
                "/launch/localization.launch.py",
            ]
        ),
        launch_arguments={
            "namespace": namespace,
            "use_sim": "True",
            "log_level": log_level,
        }.items(),
    )

    components = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("husarion_components_description"),
                "/launch/gz_components.launch.py",
            ]
        ),
        launch_arguments={
            "namespace": namespace,
            "components_config_path": components_config_path,
            "use_sim": "True",
        }.items(),
    )

    model_name = PythonExpression(["'", namespace, "' if '", namespace, "' else 'panther'"])
    namespaced_bridge_config = ReplaceString(
        source_file=PathJoinSubstitution(
            [FindPackageShare("husarion_ugv_gazebo"), "config", "gz_bridge.yaml"]
        ),
        replacements={"<model_name>": model_name, "<namespace>": namespace, "//": "/"},
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        namespace=namespace,
        parameters=[{"config_file": namespaced_bridge_config}],
        arguments=["--ros-args", "--log-level", log_level],
        output="screen",
        emulate_tty=True,
    )

    world_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_odom_tf",
        arguments=[
            "--x",
            LaunchConfiguration("world_x"),
            "--y",
            LaunchConfiguration("world_y"),
            "--z",
            LaunchConfiguration("world_z"),
            "--roll",
            LaunchConfiguration("world_roll"),
            "--pitch",
            LaunchConfiguration("world_pitch"),
            "--yaw",
            LaunchConfiguration("world_yaw"),
            "--frame-id",
            "world",
            "--child-frame-id",
            PythonExpression(["'", namespace, "/odom' if '", namespace, "' else 'odom'"]),
        ],
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("publish_world_tf")),
    )

    teleop = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="keyboard_teleop",
        namespace=namespace,
        prefix=LaunchConfiguration("teleop_prefix"),
        parameters=[
            {
                "stamped": False,
                "frame_id": "base_link",
            }
        ],
        remappings=[("cmd_vel", "cmd_vel")],
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("use_teleop")),
    )

    return [
        SetEnvironmentVariable(name="GZ_SIM_RESOURCE_PATH", value=sim_resource_path),
        SetEnvironmentVariable(name="IGN_GAZEBO_RESOURCE_PATH", value=ign_resource_path),
        SetEnvironmentVariable(name="GAZEBO_MODEL_PATH", value=gazebo_model_path),
        gz_sim,
        robot_description,
        spawn_robot,
        TimerAction(period=4.0, actions=[controller]),
        localization,
        components,
        bridge,
        world_transform,
        TimerAction(period=6.0, actions=[teleop]),
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory("greenhouse_sim")
    default_world = os.path.join(pkg_share, "worlds", "greenhouse.world")
    default_sensor_config = os.path.join(pkg_share, "config", "panther_sensors.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=default_world,
                description="Gazebo world file to load.",
            ),
            DeclareLaunchArgument(
                "sensor_config",
                default_value=default_sensor_config,
                description="YAML file controlling Panther LiDAR/camera transforms and LiDAR channels.",
            ),
            DeclareLaunchArgument(
                "gz_gui",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("husarion_ugv_gazebo"), "config", "teleop_with_estop.config"]
                ),
                description="Gazebo GUI config.",
            ),
            DeclareLaunchArgument(
                "gz_log_level",
                default_value="2",
                choices=["0", "1", "2", "3", "4"],
                description="Gazebo console verbosity.",
            ),
            DeclareLaunchArgument(
                "namespace",
                default_value="panther",
                description="Robot namespace and Gazebo model name.",
            ),
            DeclareLaunchArgument(
                "teleop_prefix",
                default_value="gnome-terminal --",
                description=(
                    "Terminal command prefix used to run keyboard teleop. "
                    "Set to an empty string only if launching from an interactive TTY."
                ),
            ),
            DeclareLaunchArgument(
                "use_teleop",
                default_value="True",
                choices=["True", "true", "False", "false"],
                description="Start teleop_twist_keyboard with the robot.",
            ),
            DeclareLaunchArgument("x", default_value="0.0"),
            DeclareLaunchArgument("y", default_value="-1.25"),
            DeclareLaunchArgument("z", default_value="0.35"),
            DeclareLaunchArgument("roll", default_value="0.0"),
            DeclareLaunchArgument("pitch", default_value="0.0"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            DeclareLaunchArgument("world_x", default_value="0.0"),
            DeclareLaunchArgument("world_y", default_value="0.0"),
            DeclareLaunchArgument("world_z", default_value="0.0"),
            DeclareLaunchArgument("world_roll", default_value="0.0"),
            DeclareLaunchArgument("world_pitch", default_value="0.0"),
            DeclareLaunchArgument("world_yaw", default_value="0.0"),
            DeclareLaunchArgument(
                "publish_world_tf",
                default_value="True",
                choices=["True", "true", "False", "false"],
                description=(
                    "Publish a static world->odom transform. Disable this when running SLAM, "
                    "because slam_toolbox publishes map->odom."
                ),
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="INFO",
                choices=["DEBUG", "INFO", "WARNING", "ERROR", "FATAL"],
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
