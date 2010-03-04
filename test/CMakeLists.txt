pkg_check_modules(BASE REQUIRED "base-lib")
include_directories(${BASE_INCLUDE_DIRS})
link_directories(${BASE_LIBRARY_DIRS})

add_executable(test test.cpp test_eigen_wrappers.cpp test_nurbs.cpp ${TOOLKIT_ADDITIONAL_SOURCES})
target_link_libraries(test ${Boost_UNIT_TEST_FRAMEWORK_LIBRARIES} ${BASE_LIBRARIES})

