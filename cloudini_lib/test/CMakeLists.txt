set(SRC_FILES
  test_field_encoders.cpp
  test_intrinsics.cpp
)

if(ament_cmake_FOUND)

    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(test_cloudini ${SRC_FILES})
    target_link_libraries(test_cloudini ${ament_LIBRARIES})

else()

    find_package(GTest REQUIRED)

    enable_testing()
    add_executable(test_cloudini ${SRC_FILES})
    add_test(NAME test_cloudini COMMAND test_cloudini)

    target_link_libraries(test_cloudini
        GTest::gtest
        GTest::gtest_main)

endif()

target_link_libraries(test_cloudini cloudini_lib)
