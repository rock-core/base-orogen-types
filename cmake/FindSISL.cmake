set(SISL_FOUND FALSE)


if (NOT SISL_PREFIX)
    set(SISL_PREFIX ${CMAKE_INSTALL_PREFIX} CACHE FILEPATH "the installation prefix for the SISL library")
endif()

if(APPLE)
  set(LIB_SUFFIX "dylib")
else(APPLE)
  set(LIB_SUFFIX "so")
endif(APPLE)

find_path(SISL_INCLUDE_DIRS "sisl.h"
    HINTS ${SISL_PREFIX}/include ${CMAKE_INSTALL_PREFIX}/include)
find_library(SISL_LIBRARIES
    NAMES libsisl.${LIB_SUFFIX} libsisl_opt.${LIB_SUFFIX} libsisl.a libsisl_opt.a
    HINTS ${SISL_PREFIX}/lib ${CMAKE_INSTALL_PREFIX}/lib)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(SISL "SISL library not found, NURBS 3D curve wrappers won't be installed" SISL_INCLUDE_DIRS SISL_LIBRARIES)
