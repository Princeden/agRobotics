from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "cmd_vel_topic",
                default_value="/panther/cmd_vel",
                description="Twist command topic for Panther.",
            ),
            DeclareLaunchArgument(
                "linear_speed",
                default_value="0.5",
                description="Initial linear speed in m/s.",
            ),
            DeclareLaunchArgument(
                "angular_speed",
                default_value="0.8",
                description="Initial angular speed in rad/s.",
            ),
            Node(
                package="greenhouse_teleop_control",
                executable="panther_gui_teleop",
                name="panther_gui_teleop",
                output="screen",
                parameters=[
                    {
                        "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                        "linear_speed": LaunchConfiguration("linear_speed"),
                        "angular_speed": LaunchConfiguration("angular_speed"),
                    }
                ],
            ),
        ]
    )
