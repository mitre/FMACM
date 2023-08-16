# *************************** UNIT TESTS ******************************** #
# Link all the actual test code along with main.cpp to the executable, 
# so as much of test infrastructure is built into
# the /unittest library as possible.
add_subdirectory(${UNITTEST_DIR})

# include MITRE-open source test code projects
set(PUBLIC_TEST_SUPPORT_SOURCE
   ${UNITTEST_DIR}/src/utils/public/OldCustomMathUtils.cpp
   ${UNITTEST_DIR}/src/utils/public/PublicUtils.cpp
)
include(${UNITTEST_DIR}/src/Public/public.cmake)
include(${UNITTEST_DIR}/src/AircraftDynamicsTestFramework/framework.cmake)

# add a target for running all of the unit test binaries at one time
add_custom_target(run_tests
   WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/unittest/
)
add_dependencies(run_tests
   run_fmacm_test
   run_public_test
)