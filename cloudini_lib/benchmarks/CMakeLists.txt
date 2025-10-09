find_package(benchmark REQUIRED)


find_package(Draco QUIET)

if(PCL_FOUND)
  message(STATUS "PCL found, using system include")

  add_executable(pcd_benchmark pcd_benchmark.cpp)

  target_include_directories(pcd_benchmark
    PRIVATE
      ${PCL_INCLUDE_DIRS}
      ${DRACO_INCLUDE_DIRS}
  )

  target_link_libraries(pcd_benchmark
    PRIVATE
      cloudini_lib
      benchmark
      data_path
      ${PCL_LIBRARIES}
      ${DRACO_LIBRARIES}
  )

  if(Draco_FOUND)
    message(STATUS "Draco found")
    target_compile_definitions(pcd_benchmark PRIVATE DRACO_FOUND)
  else()
    message(STATUS "Draco not found, not using it in benchmarks")
  endif()

else()
  message(STATUS "PCL not found, skipping pcd_benchmark")
endif()
