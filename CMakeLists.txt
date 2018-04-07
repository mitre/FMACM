cmake_minimum_required(VERSION 3.0)
project(aaesim VERSION 2.0.0.2)

set (CMAKE_CXX_STANDARD 11)

# Include a suite of helper functions for output and building the aaesim package
include(.cmake/functions.cmake)

# Append -SNAPSHOT to the version name if this is a pre-release version
if (${PROJECT_VERSION_TWEAK})
    set(aaesim_VERSION ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH}-SNAPSHOT)
else()
    set(aaesim_VERSION ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH})
endif()
message("-- Generating build for ${PROJECT_NAME} version ${aaesim_VERSION}")

# Gather git info if git exists
find_package(Git)
if(GIT_FOUND)
    message("Git found: ${GIT_EXECUTABLE}")
    include (.cmake/GetGitRevisionDescription.cmake)
    get_git_head_revision(GIT_BRANCH GIT_HASH)
    git_local_changes(GIT_LOCAL_CHANGES)
endif()

# Write the build_info
set(USERN $ENV{USER})
set(HOSTNAME $ENV{HOSTNAME})
string(TIMESTAMP BUILDDATETIME %Y/%m/%d-%H:%M:%S) # example: 20160101.010101
configure_file(${PROJECT_SOURCE_DIR}/.cmake/build_info.h.template ${PROJECT_SOURCE_DIR}/include/aaesim/build_info.h)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-strict-aliasing")

# cmake -DCMAKE_BUILD_TYPE=Release ..
# cmake -DCMAKE_BUILD_TYPE=Debug ..
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -Wno-sign-compare -O0 -g3")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall -Wno-sign-compare -O3 -g3")

set (CAASD_WIND_DIR /devel/caasdwind/release/v2.1.1/local)
set (caasdwind_LIBRARIES
        ${CAASD_WIND_DIR}/lib/libwind.so
        ${CAASD_WIND_DIR}/lib/libgrib_api.so
        ${CAASD_WIND_DIR}/lib/libopenjpeg.so
		)
set (caasdwind_INCLUDE_DIRS
        ${CAASD_WIND_DIR}/include
		)

# Create some common paths
set (LOADER_DIR   ${CMAKE_SOURCE_DIR}/Loader)
set (UNITSLIB_DIR ${CMAKE_SOURCE_DIR}/unitsLib)
set (UTILITY_DIR  ${CMAKE_SOURCE_DIR}/Utility)
set (MATH_DIR     ${CMAKE_SOURCE_DIR}/Math)
set (CORE_DIR     ${CMAKE_SOURCE_DIR}/Core)
set (PUBLIC_DIR   ${CMAKE_SOURCE_DIR}/Public)
set (GROUND_DIR   ${CMAKE_SOURCE_DIR}/GroundSystems)
set (IM_DIR       ${CMAKE_SOURCE_DIR}/IntervalManagement)
set (PA_DIR       ${CMAKE_SOURCE_DIR}/PairedApproach)
set (UNITTEST_DIR ${CMAKE_SOURCE_DIR}/unittest)
set (GTEST_DIR    ${CMAKE_SOURCE_DIR}/unittest/gtest-1.7.0/)
set (FRAMEWORK_DIR ${CMAKE_SOURCE_DIR}/AircraftDynamicsTestFramework)
set (aaesim_INCLUDE_DIRS
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/unitsLib/units-2.1/scalar/)

add_subdirectory(${UNITSLIB_DIR})
add_subdirectory(${LOADER_DIR})
add_subdirectory(${UTILITY_DIR})
add_subdirectory(${PUBLIC_DIR})
add_subdirectory(${MATH_DIR})

include(${PA_DIR}/pa.cmake OPTIONAL)
include(${IM_DIR}/im.cmake OPTIONAL)
include(${CORE_DIR}/core.cmake OPTIONAL)
include(${UNITTEST_DIR}/unittest.cmake OPTIONAL)
include(${FRAMEWORK_DIR}/framework.cmake OPTIONAL)

set (aaesim_CONFIG_INPUT       ${CMAKE_CURRENT_SOURCE_DIR}/.cmake/aaesim-config.cmake.in)
set (aaesim_CONFIG_OUTPUT_DIR  ${CMAKE_CURRENT_SOURCE_DIR}/lib)
set (aaesim_CONFIG_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

build_config(${aaesim_CONFIG_INPUT} ${PROJECT_VERSION} ${aaesim_CONFIG_OUTPUT_DIR} ${aaesim_CONFIG_INCLUDE_DIR})

# add a target to generate API documentation with Doxygen
FIND_PACKAGE(Doxygen)
if(DOXYGEN_FOUND)
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
    add_custom_target(doc
            ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            COMMENT "Generating API documentation with Doxygen" VERBATIM
            )
endif(DOXYGEN_FOUND)

# add installation information (will also add install target to makefile)
# NOTE: this works if using: make install/local. However, it's not clear how useful that installation actually is because the libs are all missing.
SET(DEPLOY_ROOT_PATH /devel/${PROJECT_NAME}/bamboo)
SET(INSTALL_DIR ${DEPLOY_ROOT_PATH}/${aaesim_VERSION})
SET(CMAKE_INSTALL_PREFIX ${INSTALL_DIR}) # so that log4cplus installs itself without errors
# INSTALL(TARGETS aaesim DESTINATION ${INSTALL_DIR})
