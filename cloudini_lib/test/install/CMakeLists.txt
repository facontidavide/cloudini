cmake_minimum_required(VERSION 3.16)
project(cloudini_install_test LANGUAGES CXX)

find_package(cloudini_lib REQUIRED)
add_executable(consumer main.cpp)
target_link_libraries(consumer PRIVATE cloudini::cloudini_lib)

enable_testing()
add_test(NAME installed_consumer COMMAND consumer)
