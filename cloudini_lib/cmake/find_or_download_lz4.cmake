function(find_or_download_lz4)

  # find_package(LZ4 QUIET)

  # Check if LZ4 targets already exist (e.g., from Arrow)
  if(NOT TARGET LZ4::lz4_static)
    message(STATUS "Downloading and compiling LZ4")

    # lz4 ###
    cpmaddpackage(
      NAME lz4
      URL https://github.com/lz4/lz4/archive/refs/tags/v1.10.0.zip
      DOWNLOAD_ONLY YES)

    set(LZ4_FOUND TRUE FORCE)

    file(GLOB LZ4_SOURCES ${lz4_SOURCE_DIR}/lib/*.c)

    # define a helper to build both static and shared variants
    add_library(lz4_static STATIC ${LZ4_SOURCES})
    set_property(TARGET lz4_static PROPERTY POSITION_INDEPENDENT_CODE ON)
    target_include_directories(lz4_static PUBLIC ${lz4_SOURCE_DIR}/lib)

    add_library(LZ4::lz4_static INTERFACE IMPORTED)
      set_target_properties(LZ4::lz4_static PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES ${lz4_SOURCE_DIR}/lib
        INTERFACE_LINK_LIBRARIES lz4_static)

  endif()

endfunction()
