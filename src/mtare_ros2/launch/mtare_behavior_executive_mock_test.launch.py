from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    namespaces = ["robot_a", "robot_b"]

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            Node(
                package="mtare_ros2",
                executable="mtare_behavior_executive_cpp",
                name="mtare_behavior_executive_cpp",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"namespaces": namespaces},
                    {"enable_hysteresis_guard": False},
                    {"enable_switch_lock_guard": False},
                    {"enable_stale_guard": True},
                    {"source_timeout_sec": 2.0},
                    {"output_rate_hz": 8.0},
                ],
                output="screen",
            ),
            Node(
                package="mtare_ros2",
                executable="exact_backend_mock_sources.py",
                name="exact_backend_mock_sources",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"namespaces": namespaces},
                    {"publish_rate_hz": 8.0},
                    {"phase_period_sec": 12.0},
                    {"map_frame": "world"},
                ],
                output="screen",
            ),
        ]
    )
