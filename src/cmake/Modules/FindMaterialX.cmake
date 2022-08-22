# SPDX-License-Identifier: BSD-3-Clause
# Copyright 2019 Blender Foundation.

# Find Blender's MaterialX dir4ectory (or fall back to spack/homebrew)
# Variables are matching those output by FindUSDPixar.cmake.
# If USD_ROOT_DIR was defined in the environment, use it.
IF(NOT MATERIALX_ROOT_DIR AND NOT $ENV{MATERIALX_ROOT_DIR} STREQUAL "")
  SET(MATERIALX_ROOT_DIR $ENV{MATERIALX_ROOT_DIR})
ELSE() # Fall back to a spack install
  SET(MATERIALX_ROOT_DIR $ENV{MATERIALX_ROOT}/libraries/stdlib/genosl/include/)
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
    include
  DOC " MaterialX shader include files"
)

IF(${MMATERIALX_INCLUDE_DIR_NOTFOUND})
  set(MATERIALX_FOUND FALSE)
ENDIF()

MARK_AS_ADVANCED(
  MATERIALX_INCLUDE_DIR
)

UNSET(_mx_SEARCH_DIRS)
