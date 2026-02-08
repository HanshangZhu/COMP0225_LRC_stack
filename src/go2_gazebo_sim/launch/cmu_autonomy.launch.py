"""
Launch file to integrate CMU autonomy stack with Gazebo Go2 simulation.
This bridges the topic differences between CHAMP/Gazebo and CMU stack.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    """Launch CMU autonomy stack adapted for Gazebo."""

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    enable_far_planner = LaunchConfiguration('enable_far_planner', default='false')
    enable_simple_frontier = LaunchConfiguration('enable_simple_frontier', default='false')
    enable_geometric_frontier = LaunchConfiguration('enable_geometric_frontier', default='true')
    enable_gazebo_frontier_visual = LaunchConfiguration('enable_gazebo_frontier_visual', default='true')
    
    # Get package paths
    try:
        local_planner_pkg = get_package_share_directory('local_planner')
        terrain_analysis_pkg = get_package_share_directory('terrain_analysis')
        far_planner_pkg = get_package_share_directory('far_planner')
    except Exception as e:
        print(f"Warning: Some CMU packages not found: {e}")
        local_planner_pkg = ""
        terrain_analysis_pkg = ""
        far_planner_pkg = ""

    # === CMU Autonomy Nodes ===
    # Note: remapped to use reliable topics provided by qos_bridge
    
    # 1. Terrain Analysis - processes point clouds into traversability
    terrain_analysis_node = Node(
        package='terrain_analysis',
        executable='terrainAnalysis',
        name='terrain_analysis',
        parameters=[{
            'use_sim_time': True,
            'scanVoxelSize': 0.1,
            'decayTime': 2.0,
            'noDecayDis': 4.0,
            'clearingDis': 8.0,
            'useSorting': True,
            'quantileZ': 0.25,
            'vehicleHeight': 0.4,
            'terrainVoxelSize': 0.05,
            'terrainVoxelHalfWidth': 100, # Increased from 12 (0.6m) to 100 (5m) to see walls!
            'noDataObstacle': False,
            'clearDyObs': True,
            'terrainFrame': 'odom',
            # Accept the scan heights from laserscan_to_pointcloud (0.15-1.0m)
            'minRelZ': -0.3,
            'maxRelZ': 1.0,
        }],
        remappings=[
            ('/state_estimation', '/odom/ground_truth'),
            ('/registered_scan', '/registered_scan_reliable'),
        ],
        output='screen',
    )
    
    # 2. Local Planner - path planning with obstacle avoidance
    local_planner_node = Node(
        package='local_planner',
        executable='localPlanner',
        name='local_planner',
        parameters=[{
            'use_sim_time': True,
            'pathFolder': os.path.join(local_planner_pkg, 'paths') if local_planner_pkg else '',
            'vehicleLength': 0.6,
            'vehicleWidth': 0.35, # Reduced from 0.5 to prevent being stuck
            'sensorOffsetX': 0.0,
            'sensorOffsetY': 0.0,
            'twoWayDrive': True,
            'laserVoxelSize': 0.05,
            'terrainVoxelSize': 0.05,
            'useTerrainAnalysis': False,  # 2D lidar has no height info; use raw scan
            'checkObstacle': True,
            'adjacentRange': 4.0,
            'obstacleHeightThre': 0.15,
            'groundHeightThre': 0.15,
            'maxSpeed': 1.0, # Increase speed slightly
            'autonomyMode': True,
            'autonomySpeed': 1.0,
            'goalClearRange': 0.5,
        }],
        remappings=[
            ('/state_estimation', '/odom/ground_truth'),
            ('/registered_scan', '/registered_scan_reliable'),
        ],
        output='screen',
    )
    
    # 3. Path Follower - converts paths to cmd_vel
    path_follower_node = Node(
        package='local_planner',
        executable='pathFollower',
        name='path_follower',
        parameters=[{
            'use_sim_time': True,
            'sensorOffsetX': 0.0,
            'sensorOffsetY': 0.0,
            'twoWayDrive': True,
            'lookAheadDis': 0.8,
            'yawRateGain': 4.0,
            'maxYawRate': 1.0,
            'maxSpeed': 0.5,
            'maxAccel': 0.5,
            'autonomyMode': True,
            'autonomySpeed': 0.4,
        }],
        remappings=[
            ('/state_estimation', '/odom/ground_truth'),
        ],
        output='screen',
    )
    
    # 4. FAR Planner - exploration/frontier planning (publishes /way_point)
    far_planner_node = Node(
        package='far_planner',
        executable='far_planner',
        name='far_planner',
        parameters=[{
            'use_sim_time': True,
            'world_frame': 'odom',
            'robot_frame': 'base_link',
            'sensor_range': 3.5,  # Match our 2D lidar range
            'is_attempt_autoswitch': True,
            'is_viewpoint_extend': True,
            'is_terrain_planner': False,  # Disable for 2D
            # Treat 2D scan points as obstacles (disZ ~ 0.0)
            'util/terrain_free_Z': -0.01,
        }],
        remappings=[
            ('/odom_world', '/odom'),
            ('/scan_cloud', '/registered_scan_reliable'),
            ('/terrain_cloud', '/terrain_map'),
            # In Gazebo we don't publish terrain_local_cloud separately; reuse terrain_map.
            ('/terrain_local_cloud', '/terrain_map'),
        ],
        output='screen',
        condition=IfCondition(enable_far_planner),
    )

    # Frontier explorer: publish frontiers directly as waypoints
    frontier_explorer_node = Node(
        package='go2_gazebo_sim',
        executable='frontier_explorer.py',
        name='frontier_explorer',
        parameters=[{
            'use_sim_time': True,
            'exploration_frame': 'odom',
            'goal_topic': '/way_point',
            'min_frontier_distance': 1.0,
            'goal_reached_threshold': 2.0,
            'goal_publish_rate': 2.0,
        }],
        output='screen',
        condition=IfCondition(enable_far_planner),
    )

    # Simple frontier navigation using local terrain map (no FAR dependency)
    simple_frontier_node = Node(
        package='go2_gazebo_sim',
        executable='simple_frontier_nav.py',
        name='simple_frontier_nav',
        parameters=[{
            'use_sim_time': True,
            'terrain_topic': '/terrain_map',
            'odom_topic': '/odom/ground_truth',
            'waypoint_topic': '/way_point',
            'free_intensity_max': 0.15,
            'min_distance': 1.0,
            'max_distance': 4.0,
            'publish_rate': 1.0,
            'reselect_interval': 3.0,
            'max_points_sample': 2000,
        }],
        output='screen',
        condition=IfCondition(enable_simple_frontier),
    )

    geometric_frontier_node = Node(
        package='go2_gazebo_sim',
        executable='geometric_frontier.py',
        name='geometric_frontier',
        parameters=[{
            'use_sim_time': True,
            'scan_topic': '/scan',
            'odom_topic': '/odom/ground_truth',
            'map_frame': 'odom',
            'map_topic': '/map',
            'resolution': 0.1,
            'width': 400,
            'height': 400,
            'origin_x': -20.0,
            'origin_y': -20.0,
            'max_range': 6.0,
            'update_rate': 0.5,
            'frontier_min_size': 10,
            # Smoother heading behavior: prefer sizeable but not-too-far goals,
            # and avoid retargeting too frequently.
            'selection_mode': 'score',
            'goal_hysteresis_distance': 0.35,
            'goal_hold_sec': 3.0,
            'obstacle_clearance_cells': 2,
            'startup_delay': 12.0,          # let robot stand up + autonomy enable first
            'frontier_goal_topic': '/way_point',
            'frontier_marker_topic': '/frontier_goal_marker',
            'frontier_regions_topic': '/frontier_markers',
        }],
        output='screen',
        condition=IfCondition(enable_geometric_frontier),
    )

    gazebo_frontier_visual_node = Node(
        package='go2_gazebo_sim',
        executable='gazebo_frontier_visual.py',
        name='gazebo_frontier_visual',
        parameters=[{
            'use_sim_time': True,
            'goal_topic': '/frontier_goal',
            'entity_name': 'frontier_goal_marker',
            'min_update_distance': 0.2,
            'min_update_sec': 1.0,
        }],
        output='screen',
        condition=IfCondition(enable_gazebo_frontier_visual),
    )

    # Motion monitor: reports if robot moves and wall stop state
    motion_monitor_node = Node(
        package='go2_gazebo_sim',
        executable='motion_monitor.py',
        name='motion_monitor',
        parameters=[{
            'use_sim_time': True,
            'odom_topic': '/odom/ground_truth',
            'stop_topic': '/stop',
            'report_rate': 1.0,
            'move_threshold': 0.05,
        }],
        output='screen',
    )
    
    # 5. Visualization tools (Disabled due to crash)
    # visualization_node = Node(
    #     package='visualization_tools',
    #     executable='visualizationTools',
    #     name='visualization_tools',
    #     parameters=[{'use_sim_time': True}],
    #     output='screen',
    # )

    # Safety: Wall Collision Checker
    wall_checker_node = Node(
        package='go2_gazebo_sim',
        executable='wall_collision_checker.py',
        name='wall_collision_checker',
        parameters=[{
            'use_sim_time': True,
            # Avoid dead-stop in tight corridor; keep a smaller buffer.
            'safety_dist': 0.2,
            'check_angle_deg': 20.0,
        }],
        output='screen',
    )

    # Recovery: select a frontier when wall stop persists
    frontier_recovery_node = Node(
        package='go2_gazebo_sim',
        executable='frontier_recovery.py',
        name='frontier_recovery',
        parameters=[{
            'use_sim_time': True,
            'min_frontier_distance': 1.0,
            'trigger_stop_duration': 0.8,
            'cooldown_sec': 3.0,
            'publish_rate': 2.0,
        }],
        output='screen',
    )

    # Bridge Goalpoint tool to waypoint for local planner
    goalpoint_bridge_node = Node(
        package='go2_gazebo_sim',
        executable='goalpoint_to_waypoint.py',
        name='goalpoint_to_waypoint',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # Reactive navigation controller — replaces CMU local_planner + pathFollower
    # Drives directly toward /way_point using laser scan for obstacle avoidance
    reactive_nav_node = Node(
        package='go2_gazebo_sim',
        executable='reactive_nav.py',
        name='reactive_nav',
        parameters=[{
            'use_sim_time': True,
            'max_linear_speed': 0.35,       # m/s — gentle corridor speed
            'max_angular_speed': 0.8,       # rad/s — responsive turning
            # Keep tight so we don't early-stop short of each frontier centroid.
            'goal_tolerance': 0.1,          # m — close enough to goal
            'obstacle_slow_dist': 0.6,      # m — start slowing near walls
            'obstacle_stop_dist': 0.25,     # m — hard stop distance
            'front_half_angle_deg': 35.0,   # degrees — front cone for speed limit
            'side_check_angle_deg': 50.0,   # degrees — side zone for push
            'avoidance_gain': 0.9,          # lateral repulsion strength
            'control_rate': 15.0,           # Hz
            'startup_delay': 12.0,          # wait for stand-up + frontier goal
        }],
        output='screen',
    )

    # Publish synthetic /joy so autonomy mode is enabled without a joystick.
    autonomy_enabler_node = Node(
        package='go2_gazebo_sim',
        executable='autonomy_enabler.py',
        name='autonomy_enabler',
        parameters=[{
            'use_sim_time': True,
            'startup_delay': 15.0,
            'rate': 10.0,
        }],
        output='screen',
    )

    # Delay autonomy nodes to let bridges start first
    delayed_autonomy = TimerAction(
        period=3.0,
        actions=[
            wall_checker_node,
            frontier_recovery_node,
            goalpoint_bridge_node,
            simple_frontier_node,
            geometric_frontier_node,
            gazebo_frontier_visual_node,
            motion_monitor_node,
            autonomy_enabler_node,
            reactive_nav_node,
        ]
    )
    
    # Delay FAR planner even more
    delayed_far_planner = TimerAction(
        period=5.0,
        actions=[
            far_planner_node,
            # visualization_node,
            frontier_explorer_node,
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('enable_far_planner', default_value='false'),
        DeclareLaunchArgument('enable_simple_frontier', default_value='false'),
        DeclareLaunchArgument('enable_geometric_frontier', default_value='true'),
        DeclareLaunchArgument('enable_gazebo_frontier_visual', default_value='true'),
        
        # CMU autonomy nodes (delayed start)
        delayed_autonomy,
        delayed_far_planner,
    ])
