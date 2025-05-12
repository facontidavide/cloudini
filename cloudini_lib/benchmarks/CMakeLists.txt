find_package(benchmark QUIET)

if(benchmark_FOUND)
  message(STATUS "benchmark found, using system include")
else()
  message(STATUS "benchmark not found, downloading with CPM")
  CPMAddPackage(
    NAME benchmark
    GITHUB_REPOSITORY google/benchmark
    VERSION 1.9.3
    OPTIONS "BENCHMARK_ENABLE_TESTING Off"
  )
endif()



if(PCL_FOUND)
  message(STATUS "PCL found, using system include")

    add_executable(pcd_benchmark pcd_benchmark.cpp )
    
    target_include_directories(pcd_benchmark PRIVATE  ${PCL_INCLUDE_DIRS}})
  
    target_link_libraries(pcd_benchmark
    PRIVATE
      cloudini_lib
      benchmark
      data_path
      ${PCL_LIBRARIES}
  )
else()
  message(STATUS "PCL not found, skipping pcd_benchmark")
endif()