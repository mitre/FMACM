# *************************** UNIT TESTS ******************************** #
# Link all the actual test code along with main.cpp to the executable, 
# so as much of test infrastructure is built into
# the /unittest library as possible.
add_subdirectory(${UNITTEST_DIR})

include(${UNITTEST_DIR}/src/AircraftDynamicsTestFramework/framework.cmake)

# add a target for running all of the unit test binaries at one time
add_custom_target(run_tests
   WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/unittest/
)
add_dependencies(run_tests
   run_fmacm_test
)
