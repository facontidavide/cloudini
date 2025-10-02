function(find_or_download_mcap)

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
