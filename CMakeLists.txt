PROJECT(dfki)
cmake_minimum_required(VERSION 2.4)
if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}")
IF (EXISTS "${CMAKE_SOURCE_DIR}/.orogen")
    SET (CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/.orogen/config")
    INCLUDE(OrocosComponent)
ENDIF (EXISTS "${CMAKE_SOURCE_DIR}/.orogen")

CONFIGURE_FILE(dfki-types.pc.in dfki-types.pc @ONLY)
INSTALL(DIRECTORY ${CMAKE_SOURCE_DIR}/dfki DESTINATION include)
INSTALL(FILES ${CMAKE_BINARY_DIR}/dfki-types.pc DESTINATION lib/pkgconfig)

# FIND_PACKAGE(KDL)
# FIND_PACKAGE(OCL)
