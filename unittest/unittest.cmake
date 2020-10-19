# *************************** UNIT TESTS ******************************** #
# Link all the actual test code along with main.cpp to the executable, 
# so as much of test infrastructure is built into
# the /unittest library as possible.
add_subdirectory(${UNITTEST_DIR})

include(${UNITTEST_DIR}/non_public.cmake OPTIONAL)

SET(TEST_SOURCE
        ${NONPUBLIC_TEST_SOURCE}
        ${UNITTEST_DIR}/src/main.cpp
        ${UNITTEST_DIR}/src/public_tests.cpp)

SET(FMACM_SOURCE
        ${UNITTEST_DIR}/src/main.cpp
        ${UNITTEST_DIR}/src/framework_tests.cpp)

# Build aaesim_test
add_executable(aaesim_test ${TEST_SOURCE})
target_link_libraries(aaesim_test
        unittest
        gtest
        ${NONPUBLIC_LINK_LIBRARIES}
        )
set_target_properties(aaesim_test PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/unittest/bin
        EXCLUDE_FROM_ALL TRUE)

# Build fmacm_test
add_executable(fmacm_test ${FMACM_SOURCE})
target_link_libraries(fmacm_test
        unittest
        gtest
        framework
        )
set_target_properties(fmacm_test PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/unittest/bin
        EXCLUDE_FROM_ALL TRUE)

# add a target for running the unit tests
add_custom_target(run
        DEPENDS ${CMAKE_SOURCE_DIR}/unittest/bin/aaesim_test ${CMAKE_SOURCE_DIR}/unittest/bin/fmacm_test
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/unittest/
        )
add_dependencies(run
        test_aaesim
        test_fmacm)

add_custom_target(test_aaesim
        ${CMAKE_SOURCE_DIR}/unittest/bin/aaesim_test --gtest_output=xml:aaesim_unit_test_results.xml
        DEPENDS ${CMAKE_SOURCE_DIR}/unittest/bin/aaesim_test
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/unittest/
        )

add_custom_target(test_fmacm
        ${CMAKE_SOURCE_DIR}/unittest/bin/fmacm_test --gtest_output=xml:fmacm_unit_test_results.xml
        DEPENDS ${CMAKE_SOURCE_DIR}/unittest/bin/fmacm_test
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/unittest/
        )

# public tests only
add_custom_target(public_test
        ${CMAKE_SOURCE_DIR}/unittest/bin/aaesim_test --gtest_filter=CoreUtils.*:CoreUtils/*.*:*/CoreUtils.*/*:*/CoreUtils/*.* --gtest_color=yes --gtest_output=xml:aaesim_unit_test_results.xml
        DEPENDS ${CMAKE_SOURCE_DIR}/unittest/bin/aaesim_test
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/unittest/
        )

# core tests only
add_custom_target(core_test
        ${CMAKE_SOURCE_DIR}/unittest/bin/aaesim_test --gtest_filter=SimpleAircraft.*:SimpleAircraft/*.*:*/SimpleAircraft.*/*:*/SimpleAircraft/*.* --gtest_color=noaaesim_test --gtest_filter=SimpleAircraft.*:SimpleAircraft/*.*:*/SimpleAircraft.*/*:*/SimpleAircraft/*.* --gtest_color=no --gtest_output=xml:aaesim_unit_test_results.xml
        DEPENDS ${CMAKE_SOURCE_DIR}/unittest/bin/aaesim_test
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/unittest/
        )

# aaesim tests only
add_custom_target(algorithm
        ${CMAKE_SOURCE_DIR}/unittest/bin/aaesim_test --gtest_filter=PrecalcTrajectoryKinematicTM.*:PrecalcTrajectoryKinematicTM/*.*:*/PrecalcTrajectoryKinematicTM.*/*:*/PrecalcTrajectoryKinematicTM/*.*:AircraftIntent.*:AircraftIntent/*.*:*/AircraftIntent.*/*:*/AircraftIntent/*.*:CustomMath.*:CustomMath/*.*:*/CustomMath.*/*:*/CustomMath/*.*:RandomGenerator.*:RandomGenerator/*.*:*/RandomGenerator.*/*:*/RandomGenerator/*.*:DVector.*:DVector/*.*:*/DVector.*/*:*/DVector/*.*:DMatrix.*:DMatrix/*.*:*/DMatrix.*/*:*/DMatrix/*.*:SimulationTime.*:SimulationTime/*.*:*/SimulationTime.*/*:*/SimulationTime/*.*:ADSBEther.*:ADSBEther/*.*:*/ADSBEther.*/*:*/ADSBEther/*.*:ADSBReceiver.*:ADSBReceiver/*.*:*/ADSBReceiver.*/*:*/ADSBReceiver/*.*:ADSBDevice.*:ADSBDevice/*.*:*/ADSBDevice.*/*:*/ADSBDevice/*.*:AircraftState.*:AircraftState/*.*:*/AircraftState.*/*:*/AircraftState/*.*:AirborneApplication.*:AirborneApplication/*.*:*/AirborneApplication.*/*:*/AirborneApplication/*.*:Atmosphere.*:Atmosphere/*.*:*/Atmosphere.*/*:*/Atmosphere/*.*:AngleOperations.*:AngleOperations/*.*:*/AngleOperations.*/*:*/AngleOperations/*.*:Units.*:Units/*.*:*/Units.*/*:*/Units/*.*:Utility_Functions.*:Utility_Functions/*.*:*/Utility_Functions.*/*:*/Utility_Functions/*.*:Bada.*:Bada/*.*:*/Bada.*/*:*/Bada/*.* --gtest_color=no --gtest_output=xml:aaesim_unit_test_results.xml
        DEPENDS ${CMAKE_SOURCE_DIR}/unittest/bin/aaesim_test
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/unittest/
        )

# add a target for running the unit tests
add_custom_target(trim
        ${CMAKE_SOURCE_DIR}/unittest/bin/aaesim_test --gtest_filter=ThreeDoFDynamics.Trim*:ThreeDoFDynamics/*.Trim*:*/ThreeDoFDynamics.Trim*/*:*/ThreeDoFDynamics/*.Trim* --gtest_color=yes --gtest_output=xml:aaesim_unit_test_results.xml
        DEPENDS ${CMAKE_SOURCE_DIR}/unittest/bin/aaesim_test
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/unittest/
        )
        
# *************************** UNIT TESTS ******************************** #
