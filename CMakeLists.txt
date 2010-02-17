PROJECT(base)
cmake_minimum_required(VERSION 2.6)

SET (CMAKE_MODULE_PATH 
    ${PROJECT_SOURCE_DIR}
    "${CMAKE_SOURCE_DIR}/.orogen/config")
INCLUDE(baseBase)

find_package(Boost REQUIRED)
include_directories(${BOOST_INCLUDE_DIR})

find_package(Eigen2 REQUIRED)
include_directories(${EIGEN2_INCLUDE_DIR})

add_executable(test test.cpp)
target_link_libraries(test)

# FIND_PACKAGE(KDL)
# FIND_PACKAGE(OCL)

