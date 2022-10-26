# SPDX-License-Identifier: BSD-3-Clause
# Copyright 2019 Blender Foundation.

# Find Blender's MaterialX dir4ectory (or fall back to spack/homebrew)
# Variables are matching those output by FindUSDPixar.cmake.
# If USD_ROOT_DIR was defined in the environment, use it.
IF(NOT MATERIALX_ROOT_DIR AND NOT $ENV{MATERIALX_ROOT_DIR} STREQUAL "")
  SET(MATERIALX_ROOT_DIR $ENV{MATERIALX_ROOT_DIR})
ENDIF()

SET(_mx_SEARCH_DIRS
  ${MATERIALX_ROOT_DIR}
  /opt/lib/mx
)

FIND_PATH(MATERIALX_INCLUDE_DIR
  NAMES
    mx_funcs.h
  HINTS
    ${_mx_SEARCH_DIRS}
  PATH_SUFFIXES
    libraries/stdlib/genosl/include
  DOC " MaterialX shader include files"
)

IF(${MATERIALX_INCLUDE_DIR_NOTFOUND})
  set(MATERIALX_FOUND FALSE)
ELSE()
  set(MATERIALX_FOUND TRUE)
ENDIF()

MARK_AS_ADVANCED(
  MATERIALX_INCLUDE_DIR
  MATERIALX_ROOT_DIR
  MATERIALX_FOUND
)

UNSET(_mx_SEARCH_DIRS)
