# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.

#
# * Try to find Facebook zstd library This will define ZSTD_FOUND
#   ZSTD_INCLUDE_DIR ZSTD_LIBRARY
#

if(UNIX)
  find_package(PkgConfig QUIET)
  pkg_search_module(PC_ZSTD libzstd)
endif()

find_path(ZSTD_INCLUDE_DIR zstd.h
  HINTS
    ${PC_ZSTD_INCLUDEDIR}
    ${PC_ZSTD_INCLUDE_DIRS}
)

find_library(ZSTD_STATIC_LIBRARY
  NAMES ${CMAKE_STATIC_LIBRARY_PREFIX}zstd${CMAKE_STATIC_LIBRARY_SUFFIX}
  HINTS
    ${PC_ZSTD_LIBDIR}
    ${PC_ZSTD_LIBRARY_DIRS}
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(ZSTD
  DEFAULT_MSG
  ZSTD_INCLUDE_DIR
  ZSTD_STATIC_LIBRARY
)

if(ZSTD_FOUND)
  message(STATUS "Found Zstd: static=${ZSTD_STATIC_LIBRARY}")
endif()

mark_as_advanced(ZSTD_INCLUDE_DIR ZSTD_STATIC_LIBRARY)

if(NOT TARGET zstd::libzstd_static)
  add_library(zstd::libzstd_static STATIC IMPORTED GLOBAL)
  set_target_properties(zstd::libzstd_static PROPERTIES
    IMPORTED_LOCATION           "${ZSTD_STATIC_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${ZSTD_INCLUDE_DIR}"
  )
endif()
