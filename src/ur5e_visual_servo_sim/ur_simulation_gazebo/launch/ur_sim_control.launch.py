# Copyright (c) 2021 Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Denis Stogl
#
# Brings up the UR5e + wrist camera in Gazebo Classic with ros2_control, MoveIt
# (move_group), and — for eye-in-hand visual servoing — MoveIt Servo. The body is
# split into one builder function per subsystem (description, controllers, gazebo,
# moveit, servo) so the wiring in launch_setup() reads top-to-bottom.

import os

import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def load_yaml(package_name, file_path):
    """Load a YAML file from a package's share directory into a dict (or None)."""
    path = os.path.join(get_package_share_directory(package_name), file_path)
    try:
        with open(path) as f:
            return yaml.safe_load(f)
    except OSError:
        return None


def _xacro(file_path, mappings):
    """Build a `xacro <file> k:=v ...` Command substitution from a mappings dict."""
    cmd = [FindExecutable(name="xacro"), " ", file_path]
    for key, value in mappings.items():
        cmd += [" ", f"{key}:=", value]
    return Command(cmd)


def robot_description():
    """The URDF (robot_description) from the wrist-camera xacro."""
    pkg = LaunchConfiguration("runtime_config_package")
    content = _xacro(
        PathJoinSubstitution(
            [
                FindPackageShare(LaunchConfiguration("description_package")),
                "urdf",
                LaunchConfiguration("description_file"),
            ]
        ),
        {
            "safety_limits": LaunchConfiguration("safety_limits"),
            "safety_pos_margin": LaunchConfiguration("safety_pos_margin"),
            "safety_k_position": LaunchConfiguration("safety_k_position"),
            "name": "ur",
            "ur_type": LaunchConfiguration("ur_type"),
            "prefix": LaunchConfiguration("prefix"),
            "sim_gazebo": "true",
            "simulation_controllers": PathJoinSubstitution(
                [FindPackageShare(pkg), "config", LaunchConfiguration("controllers_file")]
            ),
            "initial_positions_file": PathJoinSubstitution(
                [FindPackageShare(pkg), "config", LaunchConfiguration("initial_positions_file")]
            ),
            "camera_sensor_type": LaunchConfiguration("camera_sensor_type"),
        },
    )
    return {"robot_description": content}


def robot_description_semantic():
    """The SRDF (robot_description_semantic) for MoveIt / Servo."""
    content = _xacro(
        PathJoinSubstitution(
            [FindPackageShare("ur_moveit_config"), "srdf", "ur.srdf.xacro"]
        ),
        {"name": "ur", "prefix": LaunchConfiguration("prefix")},
    )
    return {"robot_description_semantic": content}


def _spawner(controller, *extra, condition=None):
    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=[controller, "-c", "/controller_manager", *extra],
        condition=condition,
    )


def controller_spawners():
    """joint_state_broadcaster + the initial joint controller + (for hybrid
    servoing) the Servo target controller loaded INACTIVE.

    Returns (joint_state_broadcaster, [other spawners]); the broadcaster is split
    out so RViz can be delayed until it is up.
    """
    initial = LaunchConfiguration("initial_joint_controller")
    start = LaunchConfiguration("start_joint_controller")
    jsb = _spawner("joint_state_broadcaster")
    others = [
        _spawner(initial, condition=IfCondition(start)),
        _spawner(initial, "--stopped", condition=UnlessCondition(start)),
        # Hybrid: forward_position_controller is what Servo commands during TRACK.
        # Loaded inactive; the orchestrator activates it (and deactivates the
        # joint_trajectory_controller) for servoing, and reverses it for APPROACH.
        _spawner(
            LaunchConfiguration("servo_controller"),
            "--stopped",
            condition=IfCondition(LaunchConfiguration("use_servo")),
        ),
    ]
    return jsb, others


def gazebo_nodes():
    """Gazebo server/client, model path, and the robot spawn."""
    # Make this package's models/ discoverable so model:// URIs in the world
    # (e.g. the aruco_marker target) resolve. gzserver.launch.py appends to the
    # existing GAZEBO_MODEL_PATH, so setting it here doesn't depend on the
    # gazebo_ros package.xml export scanner.
    model_path = AppendEnvironmentVariable(
        "GAZEBO_MODEL_PATH",
        PathJoinSubstitution([FindPackageShare("ur_simulation_gazebo"), "models"]),
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]
        ),
        launch_arguments={
            "gui": LaunchConfiguration("gazebo_gui"),
            "world": LaunchConfiguration("world"),
        }.items(),
    )
    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_ur",
        arguments=["-entity", "ur", "-topic", "robot_description"],
        output="screen",
    )
    return [model_path, gazebo, spawn]


def moveit_node():
    """move_group from ur_moveit_config. Its stock servo_node is suppressed
    (launch_servo:=false); we run our own tuned one below."""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ur_moveit_config"), "/launch/ur_moveit.launch.py"]
        ),
        launch_arguments={
            "ur_type": LaunchConfiguration("ur_type"),
            "launch_rviz": "true",
            "use_sim_time": "true",
            "launch_servo": "false",
        }.items(),
    )


def servo_node(robot_desc):
    """MoveIt Servo (eye-in-hand). Reuses the URDF, builds its own SRDF +
    kinematics, and consumes the planning scene move_group owns. Delayed so
    Gazebo, the controllers, and move_group are up first."""
    kinematics = PathJoinSubstitution(
        [FindPackageShare("ur_moveit_config"), "config", "kinematics.yaml"]
    )
    node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        output="screen",
        parameters=[
            {"moveit_servo": load_yaml("ur5e_visual_servo", "config/ur_servo.yaml")},
            robot_desc,
            robot_description_semantic(),
            kinematics,
            {"use_sim_time": True},
        ],
        condition=IfCondition(LaunchConfiguration("use_servo")),
    )
    return TimerAction(period=12.0, actions=[node])


def launch_setup(context, *args, **kwargs):
    robot_desc = robot_description()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_desc],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare("ur_simulation_gazebo"), "rviz", "view_robot_camera.rviz"]
            ),
        ],
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )

    joint_state_broadcaster, other_controllers = controller_spawners()
    # Hold RViz until joint_state_broadcaster is up (so TF/robot model are ready).
    delay_rviz = RegisterEventHandler(
        OnProcessExit(target_action=joint_state_broadcaster, on_exit=[rviz]),
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )

    return [
        robot_state_publisher,
        joint_state_broadcaster,
        delay_rviz,
        *other_controllers,
        *gazebo_nodes(),
        moveit_node(),
        servo_node(robot_desc),
    ]


def _declare_arguments():
    return [
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur5e",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur7e", "ur10", "ur12e",
                     "ur10e", "ur16e", "ur20", "ur30"],
            description="Type/series of used UR robot.",
        ),
        DeclareLaunchArgument(
            "safety_limits", default_value="true",
            description="Enables the safety limits controller if true.",
        ),
        DeclareLaunchArgument(
            "safety_pos_margin", default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        ),
        DeclareLaunchArgument(
            "safety_k_position", default_value="20",
            description="k-position factor in the safety controller.",
        ),
        DeclareLaunchArgument(
            "runtime_config_package", default_value="ur_simulation_gazebo",
            description="Package with the controllers' config in its 'config' folder.",
        ),
        DeclareLaunchArgument(
            "controllers_file", default_value="ur_controllers.yaml",
            description="YAML file with the controllers configuration.",
        ),
        DeclareLaunchArgument(
            "initial_positions_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("ur_description"), "config", "initial_positions.yaml"]
            ),
            description="YAML file (absolute path) with the robot's initial joint positions.",
        ),
        DeclareLaunchArgument(
            "description_package", default_value="ur_simulation_gazebo",
            description="Description package with the robot URDF/XACRO files.",
        ),
        DeclareLaunchArgument(
            "description_file", default_value="ur_with_camera.urdf.xacro",
            description="URDF/XACRO file. Defaults to the UR arm with a wrist camera.",
        ),
        DeclareLaunchArgument(
            "prefix", default_value='""',
            description="Joint-name prefix for multi-robot setups (must match the "
                        "controllers' config).",
        ),
        DeclareLaunchArgument(
            "start_joint_controller", default_value="true",
            description="Start the initial joint controller active (else loaded stopped).",
        ),
        DeclareLaunchArgument(
            "initial_joint_controller", default_value="joint_trajectory_controller",
            description="Initially-active joint controller. For hybrid servoing keep "
                        "this as joint_trajectory_controller (used for planned APPROACH).",
        ),
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?"),
        DeclareLaunchArgument("gazebo_gui", default_value="true", description="Start gazebo with GUI?"),
        DeclareLaunchArgument(
            "world", default_value="",
            description="Absolute path to a Gazebo .world file (empty = default empty world). "
                        "e.g. world:=$(ros2 pkg prefix ur_simulation_gazebo)/share/"
                        "ur_simulation_gazebo/worlds/servo_demo.world",
        ),
        DeclareLaunchArgument(
            "camera_sensor_type", default_value="depth", choices=["camera", "depth"],
            description="Wrist RealSense sensor type: 'camera' (RGB only) or 'depth' "
                        "(adds depth image + point cloud; heavier, needs offscreen depth render).",
        ),
        DeclareLaunchArgument("use_servo", default_value="true", description="Launch MoveIt Servo?"),
        DeclareLaunchArgument(
            "servo_controller", default_value="forward_position_controller",
            description="Controller MoveIt Servo commands during TRACK. Loaded inactive; "
                        "the orchestrator activates it during servoing.",
        ),
    ]


def generate_launch_description():
    return LaunchDescription(_declare_arguments() + [OpaqueFunction(function=launch_setup)])
