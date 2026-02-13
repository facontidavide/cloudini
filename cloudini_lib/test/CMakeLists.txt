set(SRC_FILES
  # test_field_encoders.cpp
  # test_intrinsics.cpp
  # test_header.cpp
  test_ros_msg.cpp
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
