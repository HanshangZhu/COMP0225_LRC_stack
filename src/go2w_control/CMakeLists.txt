cmake_minimum_required(VERSION 3.8)
project(go2w_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/reactive_nav.py
  scripts/wall_collision_checker.py
  scripts/go2w_hybrid_cmd_router.py
  scripts/go2w_startup_mode.py
  scripts/autonomy_enabler.py
  scripts/cmd_vel_activity_mux.py
  scripts/cmd_vel_to_sport_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install reactive_nav_core module alongside the scripts
install(DIRECTORY
  scripts/reactive_nav_core
  DESTINATION lib/${PROJECT_NAME}/scripts
)

ament_package()
