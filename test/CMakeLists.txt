pkg_check_modules(BASE REQUIRED "base-types-sisl")
include_directories(${BASE_INCLUDE_DIRS})
link_directories(${BASE_LIBRARY_DIRS})

include_directories("${PROJECT_SOURCE_DIR}/.orogen/typekit/__include_dir__")
include_directories("${PROJECT_BINARY_DIR}/__include_dir__")

pkg_check_modules(STD REQUIRED "std-typekit-${OROCOS_TARGET}")
include_directories(${STD_INCLUDE_DIRS})

add_definitions("-DOROCOS_TARGET=${OROCOS_TARGET}")

add_executable(base_orogen_types_test test.cpp test_matrixx.cpp test_nurbs.cpp
    ${TOOLKIT_ADDITIONAL_SOURCES} ../typekit/Opaques.cpp)
target_link_libraries(base_orogen_types_test ${Boost_SYSTEM_LIBRARIES} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARIES} ${BASE_LIBRARIES})

