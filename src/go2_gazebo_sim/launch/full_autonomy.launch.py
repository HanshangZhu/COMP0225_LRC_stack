"""
Full Autonomy Launch: Gazebo + CMU Autonomy Stack + Exploration
Simplified version that works reliably.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    """Launch complete autonomy system."""

    # Get package paths
    go2_gazebo_pkg = get_package_share_directory('go2_gazebo_sim')
    
    # Launch Arguments
    gui = LaunchConfiguration('gui')
    
    # === 1. Gazebo with Robot ===
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(go2_gazebo_pkg, 'launch', 'go2_l_corridor.launch.py')
        ),
        launch_arguments={
            'gui': gui,
            'rviz': 'true',  # Use go2_config's RViz
        }.items(),
    )
    
    # === 2. Bridges (after robot is up) ===
    
    # Convert 2D LaserScan -> 3D PointCloud2
    laserscan_to_pointcloud = Node(
        package='pointcloud_to_laserscan',
        executable='laserscan_to_pointcloud_node',
        name='laserscan_to_pointcloud',
        parameters=[{
            'use_sim_time': True,
            'target_frame': 'odom',
            'transform_tolerance': 1.0,
            'min_height': 0.15,
            'max_height': 1.0,
        }],
        remappings=[
            ('scan_in', '/scan'),
            ('cloud', '/registered_scan'),
        ],
        output='screen',
    )

    # QoS Bridge for registered_scan
    qos_bridge_node = Node(
        package='go2_gazebo_sim',
        executable='qos_bridge.py',
        name='qos_bridge',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )
    
    # Twist Bridge (TwistStamped -> Twist)
    twist_bridge_node = Node(
        package='go2_gazebo_sim',
        executable='twist_bridge.py',
        name='twist_bridge',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )
    
    # Stand Up Script (Run after Gazebo starts)
    stand_up_node = Node(
        package='go2_gazebo_sim',
        executable='stand_up_slowly.py',
        name='stand_up_slowly',
        output='screen',
    )
    
    delayed_stand_up = TimerAction(
        period=8.0,
        actions=[stand_up_node]
    )

    # === 3. CMU Autonomy Stack ===
    cmu_autonomy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(go2_gazebo_pkg, 'launch', 'cmu_autonomy.launch.py')
        ),
    )
    
    # Delayed bridges (wait for robot to spawn)
    delayed_bridges = TimerAction(
        period=10.0,
        actions=[laserscan_to_pointcloud, qos_bridge_node, twist_bridge_node]
    )
    
    # Delayed autonomy (wait for bridges)
    delayed_autonomy = TimerAction(
        period=15.0,
        actions=[cmu_autonomy_launch]
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Run GUI'),
        
        # 1. Start Gazebo first
        gazebo_launch,
        
        # 2. Stand up (delayed 8s)
        delayed_stand_up,
        
        # 3. Bridges (delayed 10s)
        delayed_bridges,
        
        # 3. CMU autonomy (delayed 15s)
        delayed_autonomy,
    ])
