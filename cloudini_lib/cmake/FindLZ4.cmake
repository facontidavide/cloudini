# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Finds liblz4.
#
# This module defines:
# LZ4_FOUND
# LZ4_INCLUDE_DIR
# LZ4_LIBRARY
#

if(UNIX)
  find_package(PkgConfig QUIET)
  pkg_search_module(PC_LZ4 lz4)
endif()

find_path(LZ4_INCLUDE_DIR
  NAMES lz4.h
  HINTS
    ${PC_LZ4_INCLUDEDIR}
    ${PC_LZ4_INCLUDE_DIRS}
)

# find shared and static libraries
find_library(LZ4_STATIC_LIBRARY
  NAMES ${CMAKE_STATIC_LIBRARY_PREFIX}lz4${CMAKE_STATIC_LIBRARY_SUFFIX}
  HINTS
    ${PC_LZ4_LIBDIR}
    ${PC_LZ4_LIBRARY_DIRS}
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(LZ4
  DEFAULT_MSG
  LZ4_INCLUDE_DIR
  LZ4_STATIC_LIBRARY
)

if(LZ4_FOUND)
  message(STATUS "Found LZ4: static=${LZ4_STATIC_LIBRARY}")

  if(NOT TARGET LZ4::lz4_static)
    add_library(LZ4::lz4_static STATIC IMPORTED GLOBAL)
    set_target_properties(LZ4::lz4_static PROPERTIES
      IMPORTED_LOCATION           "${LZ4_STATIC_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${LZ4_INCLUDE_DIR}"
    )
  endif()
endif()

mark_as_advanced(LZ4_INCLUDE_DIR LZ4_STATIC_LIBRARY)
