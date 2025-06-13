cmake_minimum_required(VERSION 3.16)
project(cloudini_lib)

# If not specified, set the default build type to Release
if (NOT CMAKE_BUILD_TYPE)
  message(STATUS "No build type selected, default to Release")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "RelWithDebInfo" "MinSizeRel")
endif()
message(STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")

option(CLOUDINI_BUILD_TOOLS "Build Tools" ON)

# create compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

##################################################################################
## Third party dependeciens

# add cmake folder to path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(FetchContent)
include(cmake/CPM.cmake)
include(cmake/download_deps.cmake)

find_package(ament_cmake QUIET)

find_or_download_zstd()
message(STATUS "ZSTD_INCLUDE_DIR: ${ZSTD_INCLUDE_DIR}")
message(STATUS "ZSTD_LIBRARY: ${ZSTD_LIBRARY}")

find_or_download_lz4()
message(STATUS "LZ4_INCLUDE_DIR: ${LZ4_INCLUDE_DIR}")
message(STATUS "LZ4_LIBRARY: ${LZ4_LIBRARY}")


##################################################################################

# ----  Trick to pass an absolute path to C++ code  ----
get_filename_component(DATA_PATH "${CMAKE_CURRENT_LIST_DIR}/samples" ABSOLUTE)
configure_file (data_path.hpp.in "${CMAKE_BINARY_DIR}/include/data_path.hpp" @ONLY)
add_library(data_path INTERFACE IMPORTED)
target_include_directories(data_path INTERFACE ${CMAKE_BINARY_DIR}/include)

##################################################################################

if(ament_cmake_FOUND)
  message(STATUS "PCL found using ament_cmake")
  find_package(pcl_ros REQUIRED)
  set(PCL_SRC src/pcl_conversion.cpp)

else()
  find_package(PCL QUIET)
  if(PCL_FOUND)
    message(STATUS "PCL found in system")
    add_definitions(${PCL_DEFINITIONS})
    link_directories(${PCL_LIBRARY_DIRS})
    set(PCL_SRC src/pcl_conversion.cpp)
  else()
    message(STATUS "PCL NOT found")
  endif()
endif()

##################################################################################

# add address sanitizer in debug mode
if(CMAKE_BUILD_TYPE MATCHES "Debug")
  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    message(STATUS "Adding address sanitizer")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -fno-omit-frame-pointer")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=address")
  endif()
endif()

option(CLOUDINI_BUILD_BENCHMARKS "Build benchmarks" ON)

add_library(cloudini_lib
  src/cloudini.cpp
  src/field_encoder.cpp
  src/field_decoder.cpp
  src/ros_msg_utils.cpp
  ${PCL_SRC}
)

target_compile_features(cloudini_lib PUBLIC cxx_std_20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(cloudini_lib PRIVATE -Wall -Wextra)
endif()

target_include_directories(cloudini_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
  PRIVATE
    $<BUILD_INTERFACE:${LZ4_INCLUDE_DIR}>
    $<BUILD_INTERFACE:${ZSTD_INCLUDE_DIR}>
    $<BUILD_INTERFACE:${DRACO_INCLUDE_DIR}>
)

target_link_libraries(cloudini_lib
  PRIVATE
    ${LZ4_LIBRARY}
    ${ZSTD_LIBRARY}
    ${DRACO_LIBRARY}
  PUBLIC
    ${PCL_LIBRARIES}
)

if(NOT EMSCRIPTEN)
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|i686")
    if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
      target_compile_options(cloudini_lib PRIVATE -msse4.1)
    endif()
  endif()

  if(CLOUDINI_BUILD_BENCHMARKS)
    add_subdirectory(benchmarks)
  endif()

  include(CTest)
  message( STATUS "BUILD_TESTING:   ${BUILD_TESTING} " )
  if (BUILD_TESTING)
    add_subdirectory(test)
  endif()

else()
  # WASM module target
  add_executable(cloudini_wasm
    src/wasm_functions.cpp
  )

  target_compile_features(cloudini_wasm PUBLIC cxx_std_20)

  target_include_directories(cloudini_wasm
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
      ${LZ4_INCLUDE_DIR}
      ${ZSTD_INCLUDE_DIR}
  )

  target_link_libraries(cloudini_wasm
    PRIVATE
      cloudini_lib
  )

  set_target_properties(cloudini_wasm PROPERTIES
    LINK_FLAGS "-s EXPORTED_FUNCTIONS=['_ComputeCompressedSize','_malloc','_free'] -s EXPORTED_RUNTIME_METHODS=['ccall','cwrap','HEAPU8'] -s MODULARIZE=1 -s EXPORT_NAME='CloudiniModule'"
  )

endif(NOT EMSCRIPTEN)

if(CLOUDINI_BUILD_TOOLS)
  add_subdirectory(tools)
endif()

INSTALL(TARGETS cloudini_lib
        EXPORT cloudini_libTargets
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
  )

INSTALL( DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
    DESTINATION include
    FILES_MATCHING PATTERN "*.h*")

if(ament_cmake_FOUND)
  ament_export_include_directories(include)
  ament_export_dependencies(pcl_ros)
  ament_export_libraries(cloudini_lib)
  ament_export_targets(cloudini_libTargets)
  ament_package()
endif()
