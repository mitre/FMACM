cmake_minimum_required(VERSION 3.14)

set(CMAKE_CXX_STANDARD 11)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fno-strict-aliasing -Wunused-result")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -Wno-sign-compare -O1 -g3 -Wunused-result")

CPMAddPackage(
		NAME googletest
		GITHUB_REPOSITORY google/googletest
		GIT_TAG release-1.10.0
		VERSION 1.10.0
		OPTIONS
		"INSTALL_GTEST OFF"
		"gtest_force_shared_crt ON"
)

include(non_public.cmake OPTIONAL)

SET(TEST_SUPPORT_SOURCE
        ${NONPUBLIC_TEST_SUPPORT_SOURCE}
        src/utils/public/OldCustomMathUtils.cpp
        src/utils/public/PublicUtils.cpp
		)


set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)
add_library(unittest
		${TEST_SUPPORT_SOURCE})

target_link_libraries(unittest PRIVATE log4cplusS math pub utility loader gtest )

target_include_directories(unittest PUBLIC
		${aaesim_INCLUDE_DIRS}
		${geolib_idealab_INCLUDE_DIRS})
target_compile_definitions(unittest PUBLIC -D_LINUX_ -Dunix)

# add installation information (will also add install target to makefile)
SET(INSTALL_DIR /devel/aaesim/bamboo/${aaesim_VERSION})
SET(CMAKE_INSTALL_PREFIX ${INSTALL_DIR}) # so that log4cplus installs itself without errors
INSTALL(TARGETS unittest
		DESTINATION ${INSTALL_DIR})
INSTALL(DIRECTORY resources/
		DESTINATION ${INSTALL_DIR}/resources)
