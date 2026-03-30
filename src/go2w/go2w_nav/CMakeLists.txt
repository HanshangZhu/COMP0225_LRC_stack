cmake_minimum_required(VERSION 3.8)
project(go2w_nav)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/reactive_nav.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  scripts/reactive_nav_core
  DESTINATION lib/${PROJECT_NAME}/scripts
)

ament_package()
