cmake_minimum_required(VERSION 3.22)
project(cloudini)

# General settings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# main library
add_subdirectory(cloudini_lib)

# if ament detected, compile cloudini_ros
find_package(ament_cmake QUIET)

if (ament_cmake_FOUND)
  add_subdirectory(cloudini_ros)
endif()
