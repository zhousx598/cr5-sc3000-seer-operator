from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    config_file = PathJoinSubstitution(
        [FindPackageShare("seer_agv_driver"), "config", "seer_agv.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("host", default_value="192.168.192.5"),
            DeclareLaunchArgument("enable_cmd_vel", default_value="false"),
            DeclareLaunchArgument("debug_motion", default_value="false"),
            DeclareLaunchArgument("motion_duration_ms", default_value="300"),
            Node(
                package="seer_agv_driver",
                executable="seer_agv_node",
                name="seer_agv_node",
                output="screen",
                parameters=[
                    config_file,
                    {
                        "host": LaunchConfiguration("host"),
                        "enable_cmd_vel": LaunchConfiguration("enable_cmd_vel"),
                        "debug_motion": LaunchConfiguration("debug_motion"),
                        "motion_duration_ms": LaunchConfiguration("motion_duration_ms"),
                    },
                ],
            ),
        ]
    )
