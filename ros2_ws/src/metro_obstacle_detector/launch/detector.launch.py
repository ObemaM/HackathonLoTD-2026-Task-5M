from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("metro_obstacle_detector")
    default_config = PathJoinSubstitution([package_share, "config", "default.yaml"])
    default_rviz = PathJoinSubstitution([package_share, "rviz", "detector.rviz"])

    config = LaunchConfiguration("config")
    input_topic = LaunchConfiguration("input_topic")
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_rviz = LaunchConfiguration("rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            DeclareLaunchArgument("input_topic", default_value="/lidar_points"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="false"),
            Node(
                package="metro_obstacle_detector",
                executable="detector_node",
                name="obstacle_detector",
                output="screen",
                parameters=[
                    config,
                    {
                        "input_topic": input_topic,
                        "use_sim_time": use_sim_time,
                    },
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="detector_rviz",
                arguments=["-d", default_rviz],
                condition=IfCondition(start_rviz),
                output="screen",
            ),
        ]
    )

