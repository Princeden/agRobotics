from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    pkg_share = get_package_share_directory("greenhouse_sim")

    world_file = os.path.join(pkg_share, "worlds", "greenhouse.world")

    model_path = os.path.join(pkg_share, "models")

    gazebo_model_path = model_path + ":" + os.environ.get("GAZEBO_MODEL_PATH", "")

    return LaunchDescription(
        [
            SetEnvironmentVariable(name="GAZEBO_MODEL_PATH", value=gazebo_model_path),
            ExecuteProcess(cmd=["gazebo", "--verbose", world_file], output="screen"),
        ]
    )
