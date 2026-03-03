function(find_or_download_zstd FORCE_VENDORED)

  if(NOT FORCE_VENDORED)
    find_package(zstd CONFIG QUIET)
    if(NOT TARGET zstd::libzstd_static)
      find_package(ZSTD QUIET)
    endif()
    if(TARGET zstd::libzstd_static)
      # System libzstd.a is typically not compiled with -fPIC and cannot be
      # embedded in a shared library. Override ALL config-specific IMPORTED_LOCATION
      # properties (CMake checks IMPORTED_LOCATION_<CONFIG> before IMPORTED_LOCATION)
      # so that cloudini_lib.so links against the shared zstd instead.
      if(TARGET zstd::libzstd_shared)
        get_property(_zstd_shared_loc TARGET zstd::libzstd_shared PROPERTY IMPORTED_LOCATION)
        if(NOT _zstd_shared_loc)
          get_property(_zstd_shared_loc TARGET zstd::libzstd_shared PROPERTY IMPORTED_LOCATION_RELEASE)
        endif()
        if(_zstd_shared_loc)
          foreach(_cfg "" "_RELEASE" "_DEBUG" "_RELWITHDEBINFO" "_MINSIZEREL")
            set_property(TARGET zstd::libzstd_static PROPERTY "IMPORTED_LOCATION${_cfg}" "${_zstd_shared_loc}")
          endforeach()
        endif()
      endif()
      return()
    endif()
  endif()

  # Check if ZSTD targets already exist (e.g., from Arrow)
  if(NOT TARGET zstd::libzstd_static)
     message(STATUS "Downloading and compiling ZSTD")

    # zstd ###
    cpmaddpackage(
      NAME zstd
      URL https://github.com/facebook/zstd/archive/refs/tags/v1.5.7.zip
      DOWNLOAD_ONLY YES)

    set(LIBRARY_DIR ${zstd_SOURCE_DIR}/lib)
    file(GLOB CommonSources ${LIBRARY_DIR}/common/*.c)
    file(GLOB CompressSources ${LIBRARY_DIR}/compress/*.c)
    file(GLOB DecompressSources ${LIBRARY_DIR}/decompress/*.c)

    add_compile_options(-DZSTD_DISABLE_ASM)

    set(ZSTD_FOUND TRUE PARENT_SCOPE)

    # define a helper to build both static and shared variants
    add_library(libzstd_static STATIC ${CommonSources} ${CompressSources} ${DecompressSources})
    set_property(TARGET libzstd_static PROPERTY POSITION_INDEPENDENT_CODE ON)
    target_include_directories(libzstd_static PUBLIC ${zstd_SOURCE_DIR}/lib)

    add_library(zstd::libzstd_static INTERFACE IMPORTED)
    set_target_properties(zstd::libzstd_static PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES ${zstd_SOURCE_DIR}/lib
        INTERFACE_LINK_LIBRARIES libzstd_static)

  endif()

endfunction()
