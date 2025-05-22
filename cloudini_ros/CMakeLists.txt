cmake_minimum_required(VERSION 3.16)
project(cloudini_ros)

# find dependencies
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(cloudini_lib REQUIRED)
find_package(pluginlib REQUIRED)
find_package(point_cloud_interfaces REQUIRED)
find_package(point_cloud_transport REQUIRED)

set(PLUGIN_DEPS
  pluginlib::pluginlib
  rosbag2_cpp::rosbag2_cpp
  point_cloud_transport::point_cloud_transport
  rclcpp::rclcpp
  cloudini_lib::cloudini_lib
  ${sensor_msgs_TARGETS}
  ${point_cloud_interfaces_TARGETS}
)

######### library #########

add_library(cloudini_ros SHARED
  src/conversion_utils.cpp
)

target_include_directories(cloudini_ros
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(cloudini_ros
  PUBLIC
    ${PLUGIN_DEPS}
)

install(TARGETS cloudini_ros
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/cloudini_ros
)

install(
  DIRECTORY include/cloudini_ros/
  DESTINATION include/cloudini_ros
)


######### rosbag benchmark #########
add_executable(rosbag_benchmark test/rosbag_benchmark.cpp)

target_link_libraries(rosbag_benchmark
  cloudini_ros
)

######### plugins #########
set(PLUGIN_NAME cloudini_point_cloud_transport)

add_library(${PLUGIN_NAME}
  SHARED
  src/cloudini_publisher.cpp
  src/cloudini_subscriber.cpp
  src/plugin_manifest.cpp
)

target_link_libraries(${PLUGIN_NAME} PRIVATE
  ${PLUGIN_DEPS}
)


target_include_directories(${PLUGIN_NAME} PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PLUGIN_NAME}>")


install(TARGETS ${PLUGIN_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PLUGIN_NAME}
)

install(
  DIRECTORY include/cloudini_plugin
  DESTINATION include/${PLUGIN_NAME}
)

pluginlib_export_plugin_description_file(point_cloud_transport cloudini_plugins.xml)

ament_export_include_directories(include)
ament_export_libraries(cloudini_ros ${PLUGIN_NAME})
ament_export_dependencies(${PLUGIN_DEPS})
ament_package()