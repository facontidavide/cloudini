

add_library(mcap_converter src/mcap_converter.cpp)
target_include_directories(mcap_converter
  PUBLIC
   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
   $<BUILD_INTERFACE:${LZ4_INCLUDE_DIR}>
   $<BUILD_INTERFACE:${ZSTD_INCLUDE_DIR}>
)

target_link_libraries(mcap_converter
  PUBLIC
   cloudini_lib
  PRIVATE
   LZ4::lz4_static
   zstd::libzstd_static
   mcap::mcap
)

########################################################

CPMAddPackage("gh:jarro2783/cxxopts@3.3.1")

add_executable(cloudini_rosbag_converter
 src/cloudini_rosbag_converter.cpp)

target_link_libraries(cloudini_rosbag_converter
  PRIVATE
   mcap_converter
   cxxopts
)

add_executable(mcap_cutter
 src/mcap_cutter.cpp)

target_link_libraries(mcap_cutter
  PRIVATE
    cloudini_lib
    LZ4::lz4_static
    zstd::libzstd_static
    mcap::mcap
    cxxopts
)

install(TARGETS cloudini_rosbag_converter
        RUNTIME DESTINATION bin)

install(FILES cloudini_converter.desktop
        DESTINATION share/applications)

########################################################
# PCD to Cloudini converter tool

find_package(PCL QUIET COMPONENTS common io)

if(PCL_FOUND)
  add_executable(pcd_to_cloudini_converter
    src/pcd_to_cloudini_converter.cpp)

  target_link_libraries(pcd_to_cloudini_converter
    PRIVATE
      cloudini_lib
      cxxopts
      ${PCL_LIBRARIES}
  )

  target_include_directories(pcd_to_cloudini_converter
    PRIVATE
      ${PCL_INCLUDE_DIRS}
  )

  target_compile_definitions(pcd_to_cloudini_converter
    PRIVATE
      ${PCL_DEFINITIONS}
  )

  install(TARGETS pcd_to_cloudini_converter
          RUNTIME DESTINATION bin)

  message(STATUS "PCL found - building pcd_to_cloudini_converter")
else()
  message(STATUS "PCL not found - skipping pcd_to_cloudini_converter")
endif()


#find_package(Qt6 COMPONENTS Core Gui Widgets REQUIRED)
#option(COMPILING_TO_WASM "Set to ON if compiling to WASM" OFF)

# set(CMAKE_AUTOUIC ON)
# set(CMAKE_AUTOMOC ON)
# set(CMAKE_AUTORCC ON)

# qt_add_executable(mcap_encoder
#    qt_app/main.cpp
#    qt_app/mainwindow.cpp
#    qt_app/mainwindow.h
#    qt_app/mainwindow.ui
#    qt_app/bytearray_writable.hpp
#    qt_app/resources.qrc
#    include/message_definitions.hpp
#    )

# target_include_directories(mcap_encoder
#   PRIVATE
#    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#    $<BUILD_INTERFACE:${LZ4_INCLUDE_DIR}>
#    $<BUILD_INTERFACE:${ZSTD_INCLUDE_DIR}>
# )

# target_link_libraries(mcap_encoder PRIVATE
#    Qt::Core
#    Qt::Gui
#    Qt::Widgets
#    mcap_converter
# )

# #if(EMSCRIPTEN)
# #    target_compile_definitions(mcap_encoder PRIVATE USING_WASM=1)
# #endif()
