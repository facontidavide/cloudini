
option(COMPILING_TO_WASM "Set to ON if compiling to WASM" OFF)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

if(NOT TARGET fastcdr)
    CPMAddPackage(
        NAME fastcdr_imported
        GITHUB_REPOSITORY eProsima/Fast-CDR
        GIT_TAG v2.2.6
        OPTIONS "BUILD_SHARED_LIBS OFF"
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

find_package(Qt6 REQUIRED COMPONENTS Widgets)

qt_add_executable(mcap_encoder
   converter/main.cpp
   converter/mainwindow.cpp
   converter/mainwindow.h
   converter/mainwindow.ui
   converter/bytearray_writable.hpp
   converter/resources.qrc
   include/message_definitions.hpp
   )

target_include_directories(mcap_encoder
  PRIVATE
   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
   $<BUILD_INTERFACE:${LZ4_INCLUDE_DIR}>
   $<BUILD_INTERFACE:${ZSTD_INCLUDE_DIR}>
)

target_link_libraries(mcap_encoder PRIVATE
   Qt::Core
   Qt::Gui
   Qt::Widgets
   cloudini_lib
   ${LZ4_LIBRARY}
   ${ZSTD_LIBRARY}
   mcap
)

#if(EMSCRIPTEN)
#    target_compile_definitions(mcap_encoder PRIVATE USING_WASM=1)
#endif()
