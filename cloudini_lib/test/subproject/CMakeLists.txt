# Builds cloudini_lib the way a parent project embeds it (add_subdirectory or
# FetchContent) and fails at configure time if Cloudini leaks into the parent.
#   cmake -S cloudini_lib/test/subproject -B build_subproject && cmake --build build_subproject
cmake_minimum_required(VERSION 3.16)
project(cloudini_subproject_consumer LANGUAGES C CXX)

# A parent that already provides zstd under the conventional target names, as a
# Find module does. Cloudini must reuse it instead of looking zstd up again.
find_library(CONSUMER_ZSTD_LIBRARY NAMES zstd REQUIRED)
find_path(CONSUMER_ZSTD_INCLUDE_DIR NAMES zstd.h REQUIRED)
add_library(zstd::libzstd_shared SHARED IMPORTED GLOBAL)
set_target_properties(zstd::libzstd_shared PROPERTIES
  IMPORTED_LOCATION "${CONSUMER_ZSTD_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES "${CONSUMER_ZSTD_INCLUDE_DIR}")

# A ROS parent has already found ament_cmake; that must not turn the embedded
# Cloudini into a second ament package.
find_package(ament_cmake QUIET)

set(CLOUDINI_FORCE_VENDORED_DEPS OFF CACHE BOOL "" FORCE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../.. ${CMAKE_CURRENT_BINARY_DIR}/cloudini_lib)

if(NOT TARGET cloudini::cloudini_lib)
  message(FATAL_ERROR "embedded Cloudini does not define cloudini::cloudini_lib")
endif()
get_target_property(_type cloudini_lib TYPE)
if(NOT _type STREQUAL "STATIC_LIBRARY")
  message(FATAL_ERROR "embedded Cloudini is ${_type}, expected STATIC_LIBRARY")
endif()
if(CMAKE_BUILD_TYPE)
  message(FATAL_ERROR "embedded Cloudini forced CMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} on the parent")
endif()
foreach(_leaked test_cloudini cloudini_rosbag_converter)
  if(TARGET ${_leaked})
    message(FATAL_ERROR "embedded Cloudini built '${_leaked}' inside the parent")
  endif()
endforeach()
get_target_property(_links cloudini_lib INTERFACE_LINK_LIBRARIES)
if(_links MATCHES "pcl|PCL")
  message(FATAL_ERROR "embedded Cloudini made PCL a link dependency of the parent: ${_links}")
endif()
if(EXISTS "${CMAKE_BINARY_DIR}/include/data_path.hpp")
  message(FATAL_ERROR "embedded Cloudini wrote data_path.hpp into the parent's include directory")
endif()

add_executable(consumer main.cpp)
target_link_libraries(consumer PRIVATE cloudini::cloudini_lib)
install(TARGETS consumer DESTINATION bin)
