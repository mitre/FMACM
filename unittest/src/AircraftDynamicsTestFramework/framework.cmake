cmake_minimum_required(VERSION 3.14)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

set(FMACM_TEST_SOURCE
   ${UNITTEST_DIR}/src/AircraftDynamicsTestFramework/framework_tests.cpp
   ${UNITTEST_DIR}/src/AircraftDynamicsTestFramework/true_weather_tests.cpp
)
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
    ${LOG4CPLUS_DIRS}
)
set_target_properties(fmacm_test PROPERTIES
   RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/unittest/bin
   EXCLUDE_FROM_ALL TRUE)
add_custom_target(run_fmacm_test
   ${CMAKE_SOURCE_DIR}/unittest/bin/fmacm_test --gtest_output=xml:fmacm_unit_test_results.xml
   DEPENDS ${CMAKE_SOURCE_DIR}/unittest/bin/fmacm_test
   WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/unittest/
)

