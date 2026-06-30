cmake_minimum_required(VERSION 3.14)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

set(FMACM_TEST_SOURCE
   ${UNITTEST_DIR}/src/AircraftDynamicsTestFramework/framework_tests.cpp
   ${UNITTEST_DIR}/src/AircraftDynamicsTestFramework/true_weather_tests.cpp
)
set(FMACM_TEST_OUTPUT_DIR ${PROJECT_BINARY_DIR}/unittest/bin)
add_executable(fmacm_test 
   ${FMACM_TEST_SOURCE}
   ${UNITTEST_DIR}/src/main.cpp)
target_link_libraries(fmacm_test
   gtest
   framework
)
target_include_directories(fmacm_test
    PRIVATE
    ${nlohmann_json_INCLUDE_DIR}
)
set_target_properties(fmacm_test PROPERTIES
   RUNTIME_OUTPUT_DIRECTORY ${FMACM_TEST_OUTPUT_DIR}
   EXCLUDE_FROM_ALL TRUE)
add_custom_target(run_fmacm_test
   $<TARGET_FILE:fmacm_test> --gtest_output=xml:${PROJECT_BINARY_DIR}/unittest/fmacm_unit_test_results.xml
   DEPENDS fmacm_test
   WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/unittest/
)
