
if(NOT TARGET fastcdr)
    CPMAddPackage(
        NAME fastcdr_imported
        GITHUB_REPOSITORY eProsima/Fast-CDR
        GIT_TAG v2.2.6
        OPTIONS "BUILD_SHARED_LIBS OFF;BUILD_TESTING OFF"
    )
endif()

if(NOT TARGET mcap)
    CPMAddPackage(
        NAME mcap
        GITHUB_REPOSITORY foxglove/mcap
        GIT_TAG releases/cpp/v2.0.0
        DOWNLOAD_ONLY YES
    )
    # mcap has no CMake support, so we create our own target
    add_library(mcap INTERFACE)
    target_include_directories(mcap INTERFACE "${mcap_SOURCE_DIR}/cpp/mcap/include")
endif()


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
   ${LZ4_LIBRARY}
   ${ZSTD_LIBRARY}
   mcap
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

install(TARGETS cloudini_rosbag_converter
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/cloudini_ros
)


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
