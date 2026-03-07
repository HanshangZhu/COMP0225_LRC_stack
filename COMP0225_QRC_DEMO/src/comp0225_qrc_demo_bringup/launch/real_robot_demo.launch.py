import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource


def generate_launch_description():
    demo_pkg = get_package_share_directory("comp0225_qrc_demo_bringup")
    return LaunchDescription([
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(
                    demo_pkg,
                    "launch",
                    "real_robot",
                    "system_real_robot_with_route_planner.launch",
                )
            )
        )
    ])
