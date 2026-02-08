import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch Go2 robot in L-shaped corridor world."""
    
    # Get package paths
    go2_gazebo_pkg = get_package_share_directory('go2_gazebo_sim')
    go2_config_pkg = get_package_share_directory('go2_config')
    
    # L-corridor world file
    l_corridor_world = os.path.join(go2_gazebo_pkg, 'worlds', 'l_corridor.world')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    
    # Include Go2 Gazebo launch with our custom world
    go2_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(go2_config_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'world': l_corridor_world,
            'robot_name': 'go2',
            'rviz': rviz,
            # Spawn position in the corridor (x=2.0 is safely inside, NOT at origin wall)
            # z=0.275 is the original Unitree default spawn height
            'world_init_x': '2.5',
            'world_init_y': '0.0',
            'world_init_z': '0.32',
            'world_init_heading': '0.0',
            'gui': gui,
        }.items()
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument('gui', default_value='true', description='Run Gazebo GUI'),
        DeclareLaunchArgument('rviz', default_value='true', description='Run RViz'),
        go2_gazebo_launch,
    ])
