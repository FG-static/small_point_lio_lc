from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
    local_feedback_config = PathJoinSubstitution(
        [package_share, "config", "local_map_feedback.yaml"]
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_local_map_feedback = LaunchConfiguration(
        "enable_local_map_feedback"
    )
    enable_terrain_mapping = LaunchConfiguration("enable_terrain_mapping")
    clock_parameter = {
        "use_sim_time": ParameterValue(use_sim_time, value_type=bool)
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use /clock, for example during rosbag playback",
            ),
            DeclareLaunchArgument(
                "enable_local_map_feedback",
                default_value="false",
                description=(
                    "Run the fixed-lag local-map feedback node. The Point-LIO "
                    "frontend must also set local_map_feedback_enable:=true."
                ),
            ),
            DeclareLaunchArgument(
                "enable_terrain_mapping",
                default_value="false",
                description="Run the local rolling terrain mapping node.",
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
            Node(
                package="small_point_lio_pgo",
                executable="local_map_feedback_node",
                name="small_point_lio_local_map_feedback",
                output="screen",
                condition=IfCondition(enable_local_map_feedback),
                parameters=[local_feedback_config, clock_parameter],
            ),
            Node(
                package="small_point_lio_map_tools",
                executable="terrain_mapping_node",
                name="terrain_mapping_node",
                output="screen",
                condition=IfCondition(enable_terrain_mapping),
                parameters=[clock_parameter],
            ),
        ]
    )
