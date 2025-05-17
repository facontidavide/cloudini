cmake_minimum_required(VERSION 3.22)
project(cloudini)

# General settings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(EMSCRIPTEN)
  # Tested only with emscripten 3.1.70 and Qt 6.9.0
  set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};$ENV{EMSDK}/upstream/emscripten/cmake/Modules")
endif()

option(CLOUDINI_BUILD_EXAMPLES "Build examples" ON)
option(CLOUDINI_BUILD_BENCHMARKS "Build benchmarks" ON)

# If not specified, set the default build type to Release
if (NOT CMAKE_BUILD_TYPE)
  message(STATUS "No build type selected, default to Release")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "RelWithDebInfo" "MinSizeRel")
endif()
message(STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")


# ----  Trick to pass an absolute path to C++ code  ----
get_filename_component(DATA_PATH "${CMAKE_CURRENT_LIST_DIR}/samples" ABSOLUTE)
configure_file (data_path.hpp.in "${CMAKE_BINARY_DIR}/include/data_path.hpp" @ONLY)
add_library(data_path INTERFACE IMPORTED)
target_include_directories(data_path INTERFACE ${CMAKE_BINARY_DIR}/include)
# ------------------------------------------------------

# main library
add_subdirectory(cloudini_lib)

# if ament detected, compile cloudini_ros
find_package(ament_cmake QUIET)

if (ament_cmake_FOUND AND NOT EMSCRIPTEN)
  add_subdirectory(cloudini_ros)
  ament_package()
endif()

# add_subdirectory(cloudini_tools)
