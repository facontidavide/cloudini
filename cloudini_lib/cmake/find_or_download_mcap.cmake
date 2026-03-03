function(find_or_download_mcap)

  # Try system mcap_vendor first (installed on ROS buildfarm via package.xml build_depend)
  if(NOT TARGET mcap AND NOT TARGET mcap_vendor::mcap)
    find_package(mcap_vendor QUIET)
  endif()

  # mcap_vendor exports "mcap_vendor::mcap"; wrap it with compatible mcap/mcap::mcap aliases
  if(TARGET mcap_vendor::mcap AND NOT TARGET mcap)
    add_library(mcap INTERFACE)
    target_link_libraries(mcap INTERFACE mcap_vendor::mcap)
    add_library(mcap::mcap ALIAS mcap)
    return()
  endif()

  if(NOT TARGET mcap)
    message(STATUS "Downloading mcap")
    CPMAddPackage(
        NAME mcap
        GITHUB_REPOSITORY foxglove/mcap
        GIT_TAG releases/cpp/v2.0.2
        DOWNLOAD_ONLY YES
    )
    # mcap has no CMake support, so we create our own target
    add_library(mcap INTERFACE)
    target_include_directories(mcap INTERFACE "${mcap_SOURCE_DIR}/cpp/mcap/include")
    ## create alias mcap::mcap
    add_library(mcap::mcap ALIAS mcap)
  endif()

endfunction()
