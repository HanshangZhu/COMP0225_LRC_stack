cmake_minimum_required(VERSION 3.8)
project(go2_issac_stack)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(simple_scan_mapper_cpp src/simple_scan_mapper_cpp.cpp)
ament_target_dependencies(simple_scan_mapper_cpp
  rclcpp
  nav_msgs
  sensor_msgs
)

install(DIRECTORY
  launch
  config
  docs
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  simple_scan_mapper_cpp
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/isaac_t_world_bringup.py
  scripts/isaac_t_world_dual_bringup.py
  scripts/isaac_topic_router.py
  scripts/odom_tf_broadcaster.py
  scripts/readiness_gate.py
  scripts/simple_scan_mapper.py
  scripts/simple_frontier_explorer.py
  scripts/joint_trajectory_to_joint_state.py
  scripts/isaac_robot_stack.py
  scripts/pointcloud_contract_probe.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
