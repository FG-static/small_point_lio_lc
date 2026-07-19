from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("small_point_lio")
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_local_map_feedback = LaunchConfiguration(
        "enable_local_map_feedback"
    )
    clock_parameter = {
        "use_sim_time": ParameterValue(use_sim_time, value_type=bool)
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use /clock; keep true for rosbag playback",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Launch RViz2 with the project display config",
            ),
            DeclareLaunchArgument(
                "save_pcd",
                default_value="false",
                description="Enable the frontend's independent PCD accumulator",
            ),
            DeclareLaunchArgument(
                "enable_local_map_feedback",
                default_value="false",
                description=(
                    "Publish packet correction evidence and accept versioned "
                    "local tracking maps."
                ),
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "mid360.yaml"]
                ),
                description="Small Point-LIO parameter file",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "small_point_lio_lc.rviz"]
                ),
                description="RViz2 configuration file",
            ),
            Node(
                package="small_point_lio",
                executable="small_point_lio_node",
                name="small_point_lio",
                output="screen",
                parameters=[
                    LaunchConfiguration("params_file"),
                    clock_parameter,
                    {
                        "save_pcd": ParameterValue(
                            LaunchConfiguration("save_pcd"), value_type=bool
                        ),
                        "local_map_feedback_enable": ParameterValue(
                            enable_local_map_feedback, value_type=bool
                        ),
                    },
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_base_link_to_livox_frame",
                output="screen",
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
                    "base_link",
                    "--child-frame-id",
                    "livox_frame",
                ],
                parameters=[clock_parameter],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", LaunchConfiguration("rviz_config")],
                parameters=[clock_parameter],
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ]
    )
