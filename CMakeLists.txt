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
include_directories(${EIGEN3_INCLUDE_DIRS})

pkg_check_modules(BASE_TYPES_SISL "base-types-sisl")
if(NOT BASE_TYPES_SISL_FOUND)
    message(FATAL_ERROR "The dependency base/types has been built"
        " without SISL support."
        " It seems that you have intentially disabled the use of SISL in"
        " base/types. While disabling SISL support might be useful"
        " for an isolated reuse of base/types, base/orogen/types or more"
        " generally the framework Rock strictly requires"
        " SISL support in base/types. Please rebuild base/types with"
        " -DUSE_SISL=ON")
endif()
# src/spline.cpp needs to get access to internal fields of SISLCurve
# (not base-types Spline wrapper). Add SISL's include path
find_package(SISL)
include_directories(${SISL_INCLUDE_DIRS})
list(APPEND TOOLKIT_ADDITIONAL_SOURCES ${CMAKE_SOURCE_DIR}/src/spline.cpp)

INCLUDE(baseBase)

FIND_PACKAGE(Boost COMPONENTS unit_test_framework system)
IF (Boost_UNIT_TEST_FRAMEWORK_FOUND)
    MESSAGE(STATUS "boost/test found ... building test suite")
    ADD_SUBDIRECTORY(test)
ENDIF(Boost_UNIT_TEST_FRAMEWORK_FOUND)

find_package(Rock)
include(RockRuby)
rock_log_migration()
rock_log_export()

# FIND_PACKAGE(KDL)
# FIND_PACKAGE(OCL)

