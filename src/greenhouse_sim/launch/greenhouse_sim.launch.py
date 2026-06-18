import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory("greenhouse_sim")

    default_world = os.path.join(pkg_share, "worlds", "greenhouse.world")
    model_path = os.path.join(pkg_share, "models")

    gazebo_model_path = os.pathsep.join(
        path
        for path in [model_path, os.environ.get("GAZEBO_MODEL_PATH", "")]
        if path
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=default_world,
                description="Path to the Gazebo world file to load.",
            ),
            SetEnvironmentVariable(
                name="GAZEBO_MODEL_PATH",
                value=gazebo_model_path,
            ),
            ExecuteProcess(
                cmd=["gazebo", "--verbose", LaunchConfiguration("world")],
                output="screen",
            ),
        ]
    )
