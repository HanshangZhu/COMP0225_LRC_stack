cmake_minimum_required(VERSION 3.8)
project(go2_gazebo_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(xacro REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Install launch files
install(DIRECTORY
  launch
  worlds
  urdf
  rviz
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/qos_bridge.py
  scripts/geometric_frontier.py
  scripts/twist_bridge.py
  scripts/wall_collision_checker.py
  scripts/stand_up_slowly.py
  scripts/autonomy_enabler.py
  scripts/reactive_nav.py
  scripts/dual_map_coverage_visualizer.py
  scripts/robot_status_monitor.py
  scripts/odom_comparison.py
  scripts/slam_odom_relay.py
  scripts/pointcloud_adapter.py
  scripts/initial_pose_guard.py
  scripts/multi_robot_goal_assigner.py
  scripts/goal_assigner_passthrough.py
  scripts/waypoint_mux.py
  scripts/spawn_drift_monitor.py
  scripts/spawn_entity_direct.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  scripts/assets
  scripts/perception
  scripts/slam
  scripts/control
  scripts/observability
  DESTINATION lib/${PROJECT_NAME}/scripts
)

ament_package()
