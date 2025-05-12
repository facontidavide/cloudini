cmake_minimum_required(VERSION 3.22)
project(cloudini_lib)

##################################################################################
## Third party dependeciens

# add cmake folder to path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(FetchContent)
include(cmake/CPM.cmake)


option(PREFER_SYSTEM_INCLUDE "Use system include" OFF)

if (PREFER_SYSTEM_INCLUDE)
  find_package(LZ4 QUIET)
  find_package(Zstd QUIET)
endif()

if (LZ4_FOUND)
  message(STATUS "LZ4 found, using system include")
else()
  message(STATUS "LZ4 not found, downloading with CPM")
    CPMAddPackage(
    NAME lz4
    GITHUB_REPOSITORY lz4/lz4
    VERSION 1.10.0
    DOWNLOAD_ONLY YES
  )

  if(lz4_ADDED)
    option(LZ4_BUILD_CLI "Build the LZ4 command line interface" OFF)
    option(LZ4_BUILD_LEGACY "Build the legacy LZ4 library" OFF)
    add_subdirectory(${lz4_SOURCE_DIR}/build/cmake lz4_build)
    set(LZ4_INCLUDE_DIRS ${lz4_SOURCE_DIR}/lib)
    set(LZ4_LIBRARY lz4)
    endif()
endif()

if(ZSTD_FOUND)
  message(STATUS "Zstd found, using system include")
else()
  message(STATUS "Zstd not found, downloading with CPM")
    CPMAddPackage(
    NAME zstd
    GITHUB_REPOSITORY facebook/zstd
    VERSION 1.5.7
    DOWNLOAD_ONLY YES
  )

  if(zstd_ADDED)
    option(ZSTD_BUILD_PROGRAMS "BUILD PROGRAMS" OFF)
    option(ZSTD_LEGACY_SUPPORT "LEGACY SUPPORT" OFF)
    option(ZSTD_BUILD_SHARED "BUILD SHARED LIBS" OFF)
    option(ZSTD_BUILD_STATIC "BUILD STATIC LIBS" ON)
    add_subdirectory(${zstd_SOURCE_DIR}/build/cmake zstd_build)
    set(ZSTD_INCLUDE_DIR ${zstd_SOURCE_DIR}/lib)
    set(ZSTD_LIBRARY zstd)
  endif()
endif()

##################################################################################

find_package(PCL QUIET)

if(PCL_FOUND)
  message(STATUS "PCL found")
  add_definitions(${PCL_DEFINITIONS})
  link_directories(${PCL_LIBRARY_DIRS})
  set(PCL_SRC src/pcl_conversion.cpp)
else()
  message(STATUS "PCL NOT found")
endif()

add_library(cloudini_lib STATIC
  src/cloudini.cpp
  src/field_encoder.cpp
  src/field_decoder.cpp
  ${PCL_SRC}
)

target_compile_features(cloudini_lib
  PUBLIC
    cxx_std_20
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(cloudini_lib PRIVATE -Wall -Wextra)
endif()

target_include_directories(cloudini_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
  PRIVATE
    $<BUILD_INTERFACE:${LZ4_INCLUDE_DIRS}>
    $<BUILD_INTERFACE:${ZSTD_INCLUDE_DIRS}>
)

target_link_libraries(cloudini_lib
  PRIVATE
    lz4
    zstd
  PUBLIC
    ${PCL_LIBRARIES}
)


if(CLOUDINI_BUILD_BENCHMARKS)
  add_subdirectory(benchmarks)
endif()


include(CTest)
message( STATUS "BUILD_TESTING:   ${BUILD_TESTING} " )
if (BUILD_TESTING)
  add_subdirectory(test)
endif()