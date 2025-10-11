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
option(CLOUDINI_FORCE_VENDORED_DEPS "Force vendored dependencies" ON)


# create compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

##################################################################################
## Third party dependeciens

# add cmake folder to path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
  message(STATUS "Found ament_cmake")
  set(CLOUDINI_FORCE_VENDORED_DEPS ON FORCE)
endif()

include(FetchContent)
include(cmake/CPM.cmake)

include(cmake/find_or_download_zstd.cmake)
find_or_download_zstd(CLOUDINI_FORCE_VENDORED_DEPS)

include(cmake/find_or_download_lz4.cmake)
find_or_download_lz4(CLOUDINI_FORCE_VENDORED_DEPS)

include(cmake/find_or_download_mcap.cmake)
find_or_download_mcap()

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

# Use SHARED library when building with ament, STATIC otherwise
SET(CLOUDINI_SRC
  src/cloudini.cpp
  src/field_encoder.cpp
  src/field_decoder.cpp
  src/ros_msg_utils.cpp
)

if(ament_cmake_FOUND)
  add_library(cloudini_lib SHARED
    ${CLOUDINI_SRC}
    ${PCL_SRC}
  )
else()
  add_library(cloudini_lib STATIC
    ${CLOUDINI_SRC}
    ${PCL_SRC}
  )
endif()

target_compile_features(cloudini_lib PUBLIC cxx_std_20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(cloudini_lib PRIVATE -Wall -Wextra)
endif()

target_include_directories(cloudini_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${PCL_INCLUDE_DIRS}>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(cloudini_lib
  PRIVATE
    $<BUILD_INTERFACE:LZ4::lz4_static>
    $<BUILD_INTERFACE:zstd::libzstd_static>
  PUBLIC
    ${PCL_LIBRARIES}
)

if(NOT EMSCRIPTEN)
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|i686")
    if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
      target_compile_options(cloudini_lib PRIVATE -msse4.1)
    endif()
  endif()

  find_package(benchmarks QUIET)
  if(CLOUDINI_BUILD_BENCHMARKS AND benchmarks_FOUND AND PCL_FOUND)
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
  )

  target_link_libraries(cloudini_wasm
    PRIVATE
      cloudini_lib
  )

  set(CLD_EXPORTED_FUNCTIONS
      "_malloc"
      "_free"
      "_cldn_GetHeaderAsYAML"
      "_cldn_ComputeCompressedSize"
      "_cldn_GetDecompressedSize"
      "_cldn_DecodeCompressedMessage"
      "_cldn_DecodeCompressedData"
      "_cldn_ConvertCompressedMsgToPointCloud2Msg"
      "_cldn_EncodePointcloudData"
    )
  set(QUOTED_FUNCTIONS "")
  FOREACH(Item IN LISTS CLD_EXPORTED_FUNCTIONS)
    LIST(APPEND QUOTED_FUNCTIONS "'${Item}'")
  ENDFOREACH()

  STRING(JOIN "," CLD_EXPORTED_FUNCTIONS_STR ${QUOTED_FUNCTIONS})

  # joint these in a single string, adding commas between elements
  message(STATUS "Exporting the following WASM functions: [${CLD_EXPORTED_FUNCTIONS_STR}]")

  set_target_properties(cloudini_wasm PROPERTIES
    LINK_FLAGS "-s SINGLE_FILE=1 -s NO_DISABLE_EXCEPTION_CATCHING -s ASSERTIONS -s EXPORTED_FUNCTIONS=[${CLD_EXPORTED_FUNCTIONS_STR}] -s EXPORTED_RUNTIME_METHODS=['ccall','cwrap','HEAPU8','HEAP8','UTF8ToString'] -s MODULARIZE=1 -s EXPORT_NAME='CloudiniModule' -s INITIAL_MEMORY=67108864 -s MAXIMUM_MEMORY=134217728 -s ALLOW_MEMORY_GROWTH=1 -s TOTAL_STACK=1048576"
  )
endif(NOT EMSCRIPTEN)

if(CLOUDINI_BUILD_TOOLS)
  if(EMSCRIPTEN)
    message(STATUS "Disabling tools build for Emscripten")
  else()
    add_subdirectory(tools)
  endif()
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
