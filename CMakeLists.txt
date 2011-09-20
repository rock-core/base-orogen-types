PROJECT(base)
cmake_minimum_required(VERSION 2.6)

SET (CMAKE_MODULE_PATH 
    ${PROJECT_SOURCE_DIR}
    "${CMAKE_SOURCE_DIR}/.orogen/config"
    ${PROJECT_SOURCE_DIR}/cmake)

add_definitions("-Wall")

include(FindPkgConfig)
find_package(Boost REQUIRED)
include_directories(${BOOST_INCLUDE_DIR})

pkg_check_modules(EIGEN3 REQUIRED "eigen3")
include_directories(${EIGEN2_INCLUDE_DIRS})

find_package(SISL)
if (SISL_FOUND)
    pkg_check_modules(BASELIB REQUIRED "base-lib")
    include_directories(${BASELIB_INCLUDE_DIRS})
    include_directories(${SISL_INCLUDE_DIRS})
    list(APPEND TOOLKIT_ADDITIONAL_SOURCES ${CMAKE_SOURCE_DIR}/src/spline.cpp)
    list(APPEND TOOLKIT_ADDITIONAL_LIBRARIES ${BASELIB_LIBRARIES} ${SISL_LIBRARIES})
endif()

INCLUDE(baseBase)

FIND_PACKAGE(Boost COMPONENTS unit_test_framework)
IF (Boost_UNIT_TEST_FRAMEWORK_FOUND)
    MESSAGE(STATUS "boost/test found ... building test suite")
    ADD_SUBDIRECTORY(test)
ENDIF(Boost_UNIT_TEST_FRAMEWORK_FOUND)

include($ENV{ROCK_CMAKE_MACROS}/RockRuby.cmake)
rock_log_migration()

# FIND_PACKAGE(KDL)
# FIND_PACKAGE(OCL)

