cmake_minimum_required(VERSION 3.8)
project(comp0225_qrc_demo_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  meshes
  worlds
  urdf
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/stand_up_slowly.py
  scripts/qos_bridge.py
  scripts/pointcloud_adapter.py
  scripts/slam_odom_relay.py
  scripts/autonomy_enabler.py
  scripts/twist_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  scripts/assets
  scripts/perception
  scripts/slam
  scripts/control
  scripts/reactive_nav_core
  scripts/observability
  DESTINATION lib/${PROJECT_NAME}/scripts
)

ament_package()
