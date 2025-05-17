
# find dependencies
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(cloudini_ros SHARED
  src/conversion_utils.cpp
)

target_include_directories(cloudini_ros
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(cloudini_ros cloudini_lib)

ament_target_dependencies(cloudini_ros
  rosbag2_cpp
  sensor_msgs
  rclcpp
)

add_executable(rosbag_benchmark test/rosbag_benchmark.cpp)

target_link_libraries(rosbag_benchmark
  cloudini_ros
)