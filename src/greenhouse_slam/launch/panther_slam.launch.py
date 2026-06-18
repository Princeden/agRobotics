from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    use_rviz = LaunchConfiguration("use_rviz")
    pointcloud_config = LaunchConfiguration("pointcloud_config")
    slam_config = LaunchConfiguration("slam_config")
    rviz_config = LaunchConfiguration("rviz_config")
    cloud_topic = LaunchConfiguration("cloud_topic")
    scan_topic = LaunchConfiguration("scan_topic")
    publish_world_tf = LaunchConfiguration("publish_world_tf")

    world_to_map = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_map_tf",
        arguments=[
            "--x",
            "0.0",
            "--y",
            "0.0",
            "--z",
            "0.0",
            "--roll",
            "0.0",
            "--pitch",
            "0.0",
            "--yaw",
            "0.0",
            "--frame-id",
            "world",
            "--child-frame-id",
            "map",
        ],
        output="screen",
        emulate_tty=True,
        condition=IfCondition(publish_world_tf),
    )

    pointcloud_to_laserscan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        namespace=namespace,
        parameters=[pointcloud_config],
        remappings=[
            ("cloud_in", cloud_topic),
            ("scan", scan_topic),
        ],
        output="screen",
        emulate_tty=True,
    )

    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        parameters=[slam_config],
        remappings=[("scan", scan_topic)],
        output="screen",
        emulate_tty=True,
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="greenhouse_slam_rviz",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        output="screen",
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="panther",
                description="Robot namespace used by the running simulation.",
            ),
            DeclareLaunchArgument(
                "cloud_topic",
                default_value="front_lidar/velodyne_points",
                description="Namespaced PointCloud2 topic from the simulated Velodyne.",
            ),
            DeclareLaunchArgument(
                "scan_topic",
                default_value="/panther/scan",
                description="Namespaced LaserScan topic generated for SLAM.",
            ),
            DeclareLaunchArgument(
                "pointcloud_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("greenhouse_slam"),
                        "config",
                        "pointcloud_to_laserscan.yaml",
                    ]
                ),
                description="pointcloud_to_laserscan parameter file.",
            ),
            DeclareLaunchArgument(
                "slam_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("greenhouse_slam"), "config", "slam_toolbox.yaml"]
                ),
                description="slam_toolbox parameter file.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("greenhouse_slam"), "rviz", "panther_slam.rviz"]
                ),
                description="RViz configuration for greenhouse SLAM.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="True",
                choices=["True", "true", "False", "false"],
                description="Open RViz with SLAM displays.",
            ),
            DeclareLaunchArgument(
                "publish_world_tf",
                default_value="True",
                choices=["True", "true", "False", "false"],
                description="Publish static world->map so RViz can use world as fixed frame.",
            ),
            world_to_map,
            pointcloud_to_laserscan,
            slam_toolbox,
            rviz,
        ]
    )
