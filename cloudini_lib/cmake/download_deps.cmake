

function(find_or_download_lz4)

    find_package(LZ4 QUIET)

    if (LZ4_FOUND)
        message(STATUS "LZ4 found, using system include")
    else()
        message(STATUS "LZ4 not found, downloading with CPM")
            CPMAddPackage(
            NAME lz4
            GITHUB_REPOSITORY lz4/lz4
            VERSION 1.10.0
            DOWNLOAD_ONLY YES
        )

        if(lz4_ADDED)
            option(LZ4_BUILD_CLI "Build the LZ4 command line interface" OFF)
            option(LZ4_BUILD_LEGACY "Build the legacy LZ4 library" OFF)
            add_subdirectory(${lz4_SOURCE_DIR}/build/cmake lz4_build)

            set(LZ4_INCLUDE_DIRS ${lz4_SOURCE_DIR}/lib PARENT_SCOPE)
            set(LZ4_LIBRARY lz4 PARENT_SCOPE)
            set(LZ4_FOUND TRUE PARENT_SCOPE)
        endif()
    endif()

    set(LZ4_INCLUDE_DIRS ${LZ4_INCLUDE_DIRS} PARENT_SCOPE)
    set(LZ4_LIBRARY ${LZ4_LIBRARY} PARENT_SCOPE)
    set(LZ4_FOUND ${LZ4_FOUND} PARENT_SCOPE)

endfunction()

#-------------------------------------------------------------------------------

function(find_or_download_zstd)

    find_package(ZSTD QUIET)

    if (ZSTD_FOUND)
        message(STATUS "Zstd found, using system include")
    else()
        message(STATUS "Zstd not found, downloading with CPM")
            CPMAddPackage(
            NAME zstd
            GITHUB_REPOSITORY facebook/zstd
            VERSION 1.5.7
            DOWNLOAD_ONLY YES
        )

        if(zstd_ADDED)
            option(ZSTD_BUILD_PROGRAMS "BUILD PROGRAMS" OFF)
            option(ZSTD_LEGACY_SUPPORT "LEGACY SUPPORT" OFF)
            option(ZSTD_BUILD_SHARED "BUILD SHARED LIBS" OFF)
            option(ZSTD_BUILD_STATIC "BUILD STATIC LIBS" ON)
            add_subdirectory(${zstd_SOURCE_DIR}/build/cmake zstd_build)

            set(ZSTD_INCLUDE_DIRS ${zstd_SOURCE_DIR}/lib)
            set(ZSTD_LIBRARY libzstd_static)
            set(ZSTD_FOUND TRUE)
        endif()
    endif()

    set(ZSTD_INCLUDE_DIR ${ZSTD_INCLUDE_DIR} PARENT_SCOPE)
    set(ZSTD_LIBRARY ${ZSTD_LIBRARY} PARENT_SCOPE)
    set(ZSTD_FOUND ${ZSTD_FOUND} PARENT_SCOPE)

endfunction()