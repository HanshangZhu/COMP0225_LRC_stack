#!/usr/bin/env python3
"""
Simple Autonomy Launch - Gazebo + Waypoint Follower
Follows a predefined trajectory through the L-corridor (no planning).
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package paths
    go2_gazebo_pkg = get_package_share_directory('go2_gazebo_sim')
    
    # Launch arguments
    gui = LaunchConfiguration('gui')
    
    declare_gui = DeclareLaunchArgument('gui', default_value='true')
    
    # Launch Gazebo with Go2 in L-corridor
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(go2_gazebo_pkg, 'launch', 'go2_l_corridor.launch.py')
        ),
        launch_arguments={
            'gui': gui,
            'rviz': 'false',
        }.items()
    )
    
    # Waypoint Follower - follows predefined path through L-corridor
    waypoint_follower = Node(
        package='go2_gazebo_sim',
        executable='waypoint_follower.py',
        name='waypoint_follower',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )
    
    # Delay autonomy to let Gazebo + controllers fully start
    delayed_autonomy = TimerAction(
        period=15.0,
        actions=[waypoint_follower]
    )
    
    return LaunchDescription([
        declare_gui,
        gazebo_launch,
        delayed_autonomy,
    ])
