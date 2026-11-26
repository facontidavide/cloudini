set(SRC_FILES
  test_field_encoders.cpp
  test_intrinsics.cpp
  test_header.cpp
  test_ros_msg.cpp
  test_v6.cpp
  test_wasm_functions.cpp
)

if(PCL_FOUND)
  list(APPEND SRC_FILES test_pcl.cpp)
endif()

if(ament_cmake_FOUND)

    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(test_cloudini ${SRC_FILES})
    target_link_libraries(test_cloudini
      data_path
      ${ament_LIBRARIES})

    if(PCL_FOUND)
      target_link_libraries(test_cloudini ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})
    endif()

else()

    find_package(GTest REQUIRED)

    enable_testing()
    add_executable(test_cloudini ${SRC_FILES})
    add_test(NAME test_cloudini COMMAND test_cloudini)

    target_link_libraries(test_cloudini
      data_path
      GTest::gtest
      GTest::gtest_main)

    if(PCL_FOUND)
      target_link_libraries(test_cloudini ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})
    endif()

endif()

target_link_libraries(test_cloudini cloudini_lib)
# The WebAssembly exports, built natively: test/wasm_shim stands in for emscripten.h
target_sources(test_cloudini PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../src/wasm_functions.cpp)
target_include_directories(test_cloudini BEFORE PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/wasm_shim)
# the internal headers, for the switches that only tests use
target_include_directories(test_cloudini PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../src)

# Every public header compiles on its own, as a user's first include (#169)
file(GLOB CLOUDINI_PUBLIC_HEADERS CONFIGURE_DEPENDS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/../include
     ${CMAKE_CURRENT_SOURCE_DIR}/../include/cloudini_lib/*.hpp)
set(HEADER_CHECK_SOURCES "")
foreach(header ${CLOUDINI_PUBLIC_HEADERS})
  # *_impl.hpp are included by their header, not on their own
  if(header MATCHES "_impl\\.hpp$" OR (header MATCHES "pcl_conversion" AND NOT PCL_FOUND))
    continue()
  endif()
  string(MAKE_C_IDENTIFIER ${header} name)
  set(source ${CMAKE_CURRENT_BINARY_DIR}/header_check/${name}.cpp)
  if(NOT EXISTS ${source})  # written once, so that a reconfigure does not rebuild it (file(CONFIGURE) needs 3.18)
    file(WRITE ${source} "#include <${header}>\n")
  endif()
  list(APPEND HEADER_CHECK_SOURCES ${source})
endforeach()
add_library(header_check OBJECT ${HEADER_CHECK_SOURCES})
target_link_libraries(header_check PRIVATE cloudini_lib)
if(PCL_FOUND)
  target_link_libraries(header_check PRIVATE ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})
endif()
