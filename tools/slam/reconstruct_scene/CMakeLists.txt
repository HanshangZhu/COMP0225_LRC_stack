cmake_minimum_required(VERSION 3.14)
project(reconstruct_scene CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io registration filters kdtree)
find_package(Ceres REQUIRED)
find_package(yaml-cpp REQUIRED)

# rosbag2 from ROS2
find_package(ament_cmake QUIET)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(reconstruct_scene reconstruct_scene.cpp)

target_include_directories(reconstruct_scene PRIVATE
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
)

target_link_libraries(reconstruct_scene
    ${PCL_LIBRARIES}
    ${CERES_LIBRARIES}
    yaml-cpp
    Eigen3::Eigen
)

ament_target_dependencies(reconstruct_scene
    rosbag2_cpp
    rosbag2_storage
    sensor_msgs
    rclcpp
)

# BALM2 standalone runner
add_executable(run_balm2 run_balm2.cpp)
target_include_directories(run_balm2 PRIVATE
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ../../src/BALM/include
    ../../src/BALM/src/benchmark
)
target_link_libraries(run_balm2
    ${PCL_LIBRARIES}
    Eigen3::Eigen
)
target_compile_definitions(run_balm2 PRIVATE -DBALM_STANDALONE)
