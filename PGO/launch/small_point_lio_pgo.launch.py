from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("small_point_lio_pgo")
    bridge_config = PathJoinSubstitution(
        [package_share, "config", "bridge.yaml"]
    )
    backend_config = PathJoinSubstitution(
        [package_share, "config", "backend.yaml"]
    )
    map_config = PathJoinSubstitution(
        [package_share, "config", "map.yaml"]
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    clock_parameter = {
        "use_sim_time": ParameterValue(use_sim_time, value_type=bool)
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use /clock, for example during rosbag playback",
            ),
            Node(
                package="small_point_lio_pgo",
                executable="keyframe_bridge_node",
                name="small_point_lio_keyframe_bridge",
                output="screen",
                parameters=[bridge_config, clock_parameter],
            ),
            Node(
                package="small_point_lio_pgo",
                executable="loop_detector_node",
                name="small_point_lio_pgo_loop_detector",
                output="screen",
                parameters=[backend_config, clock_parameter],
            ),
            Node(
                package="small_point_lio_pgo",
                executable="map_node",
                name="map_node",
                output="screen",
                parameters=[map_config, clock_parameter],
            ),
        ]
    )
