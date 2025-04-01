cmake_minimum_required(VERSION 3.14)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

set(PUBLIC_LIBRARY_TEST_SOURCE
        ${UNITTEST_DIR}/src/Public/geolib_tests.cpp
        ${UNITTEST_DIR}/src/Public/aircraft_intent_tests.cpp
        ${UNITTEST_DIR}/src/Public/windstack_tests.cpp
        ${UNITTEST_DIR}/src/Public/utility_tests.cpp
        ${UNITTEST_DIR}/src/Public/public_tests.cpp
        ${UNITTEST_DIR}/src/Public/public_atmosphere_tests.cpp
        ${UNITTEST_DIR}/src/Public/tangent_plane_tests.cpp
        ${UNITTEST_DIR}/src/Public/wind_blending_tests.cpp
        ${UNITTEST_DIR}/src/Public/earth_model_tests.cpp
)

add_executable(public_test 
        ${PUBLIC_LIBRARY_TEST_SOURCE}
        ${PUBLIC_TEST_SUPPORT_SOURCE}
        ${UNITTEST_DIR}/src/main.cpp
)
target_link_libraries(public_test
        gtest
        pub
)
target_include_directories(public_test PUBLIC
        ${aaesim_INCLUDE_DIRS}
        ${UNITTEST_DIR}/src
        ${geolib_idealab_INCLUDE_DIRS})
set_target_properties(public_test PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/unittest/bin
        EXCLUDE_FROM_ALL TRUE)
add_custom_target(run_public_test
        ${CMAKE_SOURCE_DIR}/unittest/bin/public_test --gtest_output=xml:public_unit_test_results.xml
        DEPENDS ${CMAKE_SOURCE_DIR}/unittest/bin/public_test
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/unittest/
)
