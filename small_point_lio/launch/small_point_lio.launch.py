from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("small_point_lio")

    small_point_lio_node = Node(
        package="small_point_lio",
        executable="small_point_lio_node",
        name="small_point_lio",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [package_share, "config", "mid360.yaml"]
            ),
            LaunchConfiguration("body_lidar_config"),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "body_lidar_config",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "body_lidar.yaml"]
                ),
                description="Shared body-LiDAR mounting parameter file",
            ),
            small_point_lio_node,
        ]
    )
