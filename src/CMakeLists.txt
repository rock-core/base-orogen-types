include_directories(${CMAKE_SOURCE_DIR})

find_package(SISL)
if (SISL_FOUND)
    pkg_check_modules(BASE REQUIRED "base-lib")
    include_directories(${BASE_INCLUDE_DIRS})
    include_directories(${SISL_INCLUDE_DIRS})
    list(APPEND LIBBASE_FILES ${CMAKE_SOURCE_DIR}/src/spline.cpp)
    list(APPEND LIBBASE_DEPENDENCIES ${SISL_LIBRARIES})
endif()

if (LIBBASE_FILES)
    add_library(wrapperlib ${LIBBASE_FILES})
endif()

