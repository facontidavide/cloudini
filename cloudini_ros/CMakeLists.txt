cmake_minimum_required(VERSION 3.16)
project(cloudini_ros)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(cloudini_lib REQUIRED)
find_package(pluginlib REQUIRED)
find_package(point_cloud_interfaces REQUIRED)
find_package(point_cloud_transport REQUIRED)
find_package(Draco QUIET)
find_package(PCL COMPONENTS common REQUIRED)
find_package(pcl_conversions REQUIRED)

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
  src/cloudini_subscriber_pcl.cpp
)

target_include_directories(cloudini_ros
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(cloudini_ros
  PUBLIC
    ${PLUGIN_DEPS}
    ${PCL_COMMON_LIBRARIES}
)

install(TARGETS cloudini_ros
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/cloudini_ros
)

install(
  DIRECTORY include/cloudini_ros/
  DESTINATION include/cloudini_ros
)

######### topic converter #########

add_executable(cloudini_topic_converter
  src/topic_converter.cpp
)

target_include_directories(cloudini_topic_converter
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(cloudini_topic_converter
  cloudini_ros
  cloudini_lib::cloudini_lib
)

install(TARGETS cloudini_topic_converter
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/cloudini_ros
)

######### rosbag benchmark #########
add_executable(rosbag_benchmark
  test/rosbag_benchmark.cpp
  test/draco_helper.cpp
)

target_link_libraries(rosbag_benchmark
  cloudini_ros
)

if(Draco_FOUND)
  target_compile_definitions(rosbag_benchmark PRIVATE DRACO_FOUND)
  target_link_libraries(rosbag_benchmark draco::draco)
endif()

install(TARGETS rosbag_benchmark
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/cloudini_ros
)

######### test cloudini subscriber #########
add_executable(test_cloudini_subscriber test/test_cloudini_subscriber.cpp)

target_link_libraries(test_cloudini_subscriber
  cloudini_ros
  ${PCL_COMMON_LIBRARIES}
)

install(TARGETS test_cloudini_subscriber
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/cloudini_ros
)

######### test cloudini plugin #########
add_executable(test_plugin_subscriber test/test_plugin_subscriber.cpp)

target_link_libraries(test_plugin_subscriber
  cloudini_ros
  ${PCL_COMMON_LIBRARIES}
  ${point_cloud_interfaces_TARGETS}
  point_cloud_transport::point_cloud_transport
)

target_include_directories(test_plugin_subscriber
  PRIVATE
    ${pcl_conversions_INCLUDE_DIRS}
)

install(TARGETS test_plugin_subscriber
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/cloudini_ros
)

######### test plugin publisher #########
add_executable(test_plugin_publisher test/test_plugin_publisher.cpp)

target_link_libraries(test_plugin_publisher
  cloudini_ros
  ${sensor_msgs_TARGETS}
  point_cloud_transport::point_cloud_transport
  rclcpp::rclcpp
)

install(TARGETS test_plugin_publisher
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/cloudini_ros
)

######### test_direct_publisher #########
add_executable(test_direct_publisher test/test_direct_publisher.cpp)

target_link_libraries(test_direct_publisher
  cloudini_ros
)

install(TARGETS test_direct_publisher
  RUNTIME DESTINATION lib/cloudini_ros
)

######### plugins #########
set(PLUGIN_NAME cloudini_point_cloud_transport)

add_library(${PLUGIN_NAME}
  SHARED
  src/conversion_utils.cpp
  src/cloudini_publisher_plugin.cpp
  src/cloudini_subscriber_plugin.cpp
  src/plugin_manifest.cpp
)

target_link_libraries(${PLUGIN_NAME} PRIVATE
  ${PLUGIN_DEPS}
)

target_include_directories(${PLUGIN_NAME} PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PLUGIN_NAME}>"
)

install(TARGETS ${PLUGIN_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PLUGIN_NAME}
)

install(
  DIRECTORY include/cloudini_plugin
  DESTINATION include/${PLUGIN_NAME}
)

pluginlib_export_plugin_description_file(point_cloud_transport cloudini_plugins.xml)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  pluginlib
  rosbag2_cpp
  point_cloud_transport
  rclcpp
  cloudini_lib
  sensor_msgs
  point_cloud_interfaces
  PCL
)
ament_package()
