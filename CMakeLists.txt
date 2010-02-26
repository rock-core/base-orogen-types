PROJECT(base)
cmake_minimum_required(VERSION 2.6)

SET (CMAKE_MODULE_PATH 
    ${PROJECT_SOURCE_DIR}
    "${CMAKE_SOURCE_DIR}/.orogen/config"
    ${PROJECT_SOURCE_DIR}/cmake)

include(FindPkgConfig)
find_package(Boost REQUIRED)
include_directories(${BOOST_INCLUDE_DIR})

find_package(Eigen2 REQUIRED)
include_directories(${EIGEN2_INCLUDE_DIR})

add_subdirectory(src)

set(TOOLKIT_ADDITIONAL_LIBRARIES wrapperlib)
INCLUDE(baseBase)

FIND_PACKAGE(Boost COMPONENTS unit_test_framework)
IF (Boost_UNIT_TEST_FRAMEWORK_FOUND)
    MESSAGE(STATUS "boost/test found ... building test suite")
    ADD_SUBDIRECTORY(test)
ENDIF(Boost_UNIT_TEST_FRAMEWORK_FOUND)

# FIND_PACKAGE(KDL)
# FIND_PACKAGE(OCL)

