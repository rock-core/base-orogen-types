pkg_check_modules(BASE REQUIRED "base-lib")
include_directories(${BASE_INCLUDE_DIRS})
link_directories(${BASE_LIBRARY_DIRS})


add_definitions("-DOROCOS_TARGET=$ENV{OROCOS_TARGET}")

if (SISL_LIBRARIES)
    set(ADDITIONAL_TESTS test_nurbs.cpp)
endif()
add_executable(base_orogen_types_test test.cpp test_matrixx.cpp ${ADDITIONAL_TESTS} ${TOOLKIT_ADDITIONAL_SOURCES})
target_link_libraries(base_orogen_types_test ${Boost_UNIT_TEST_FRAMEWORK_LIBRARIES} ${BASE_LIBRARIES})

