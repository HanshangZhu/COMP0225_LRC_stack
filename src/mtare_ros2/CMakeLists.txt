cmake_minimum_required(VERSION 3.8)
project(mtare_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GridWorldStatus.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

add_executable(mtare_behavior_executive_cpp src/mtare_behavior_executive_cpp.cpp)
target_compile_features(mtare_behavior_executive_cpp PUBLIC cxx_std_17)
ament_target_dependencies(
  mtare_behavior_executive_cpp
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
)

install(DIRECTORY
  launch
  config
  docs
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/mdvrp_solver.py
  scripts/mtare_coordinator.py
  scripts/mtare_topic_bridge.py
  scripts/exact_backend_mock_sources.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  scripts/map_merge_utils.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  mtare_behavior_executive_cpp
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_mdvrp_solver
    test/test_mdvrp_solver.py
    PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}"
  )
  ament_add_pytest_test(
    test_vrp_coordinator
    test/test_vrp_coordinator.py
    PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}"
  )
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()
