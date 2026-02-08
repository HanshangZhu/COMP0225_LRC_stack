#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    # Paths
    go2_gazebo_pkg = get_package_share_directory('go2_gazebo_sim')
    champ_gazebo_pkg = get_package_share_directory('champ_gazebo')
    go2_desc_pkg = get_package_share_directory('go2_description')
    
    empty_world = os.path.join(go2_gazebo_pkg, 'worlds', 'empty_test.world')
    urdf_path = os.path.join(go2_desc_pkg, 'xacro', 'robot.xacro')
    ros_control_path = os.path.join(go2_desc_pkg, 'config', 'ros_control', 'ros_control.yaml')
    
    # Launch arguments
    gui = LaunchConfiguration('gui')
    
    declare_gui = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Show Gazebo GUI'
    )

    # Robot State Publisher - Essential for publishing /robot_description topic
    # spawn_entity.py waits for this topic!
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': Command(['xacro ', urdf_path])
        }],
    )

    # Slow Stand-Up Script - Moves joints gradually to prevent "explode"
    stand_up_node = Node(
        package='go2_gazebo_sim',
        executable='stand_up_slowly.py',
        name='stand_up_slowly',
        output='screen'
    )
    
    # Include base gazebo launch (spawns robot)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(champ_gazebo_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': empty_world,
            'robot_name': 'go2',
            'rviz': 'false',
            'lite': 'false',
            # description_path is used by champ_gazebo but we also run RSP ourselves
            'description_path': urdf_path,
            'ros_control_file': ros_control_path,
            # Spawn position
            'world_init_x': '0.0',
            'world_init_y': '0.0',
            'world_init_z': '0.25',
            'world_init_heading': '0.0',
            'gui': gui,
        }.items()
    )
    
    return LaunchDescription([
        declare_gui,
        robot_state_publisher_node,
        stand_up_node,
        gazebo_launch,
    ])
