cmake_minimum_required(VERSION 3.14)
project(aaesim VERSION 4.1.0.0) # the nano value is a boolean. 1 == SNAPSHOT, 0 == release

set (CMAKE_CXX_STANDARD 11)

# Include a suite of helper functions for output and building the aaesim package
include(${PROJECT_SOURCE_DIR}/.cmake/functions.cmake)
include(${PROJECT_SOURCE_DIR}/.cmake/CPM.cmake)

CPMAddPackage(
        NAME cppmanifest
        GIT_REPOSITORY https://mustache.mitre.org/scm/idea/cppmanifest.git
        GIT_TAG v1.3
        DOWNLOAD_ONLY TRUE
)
if (cppmanifest_ADDED)
    include(${cppmanifest_SOURCE_DIR}/cppmanifest_configure.cmake)
    cppmanifest_configure(${PROJECT_SOURCE_DIR}/include/cppmanifest)
endif()

# cppmanifest does not do strings in the version, but we want them.
# Append -SNAPSHOT to the version name if this is a pre-release version
if (${PROJECT_VERSION_TWEAK})
    set(aaesim_VERSION ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH}-SNAPSHOT)
else()
    set(aaesim_VERSION ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH})
endif()
message("-- Generating build for ${PROJECT_NAME} version ${aaesim_VERSION}")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-strict-aliasing")

# cmake -DCMAKE_BUILD_TYPE=Release ..
# cmake -DCMAKE_BUILD_TYPE=Debug ..
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -Wno-unused-function -Wno-sign-compare -O0 -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall -Wno-unused-function -Wno-sign-compare -O3 -g3")

# Add stuff to turn on code coverage only if a Debug build
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    include(.cmake/CodeCoverage.cmake)
    APPEND_COVERAGE_COMPILER_FLAGS()
endif()

CPMAddPackage(
    NAME unitslib
    GIT_REPOSITORY https://mustache.mitre.org/scm/uom/cpp-uom.git
    GIT_TAG v2.1
)

# Create some common paths
if (unitslib_ADDED)
    set(unitslib_INCLUDE_DIRS ${unitslib_SOURCE_DIR}/units-2.1/src/)
endif ()
set (LOADER_DIR   ${CMAKE_CURRENT_SOURCE_DIR}/Loader)
set (UTILITY_DIR  ${CMAKE_CURRENT_SOURCE_DIR}/Utility)
set (MATH_DIR     ${CMAKE_CURRENT_SOURCE_DIR}/Math)
set (CORE_DIR     ${CMAKE_CURRENT_SOURCE_DIR}/AAESim)
set (PUBLIC_DIR   ${CMAKE_CURRENT_SOURCE_DIR}/Public)
set (GROUND_DIR   ${CMAKE_CURRENT_SOURCE_DIR}/GroundSystems)
set (IM_DIR       ${CMAKE_CURRENT_SOURCE_DIR}/IntervalManagement)
set (UNITTEST_DIR ${CMAKE_CURRENT_SOURCE_DIR}/unittest)
set (FRAMEWORK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/AircraftDynamicsTestFramework)
set (aaesim_INCLUDE_DIRS
        ${CMAKE_CURRENT_SOURCE_DIR}/include )

add_subdirectory(${LOADER_DIR})
add_subdirectory(${UTILITY_DIR})
add_subdirectory(${PUBLIC_DIR})
add_subdirectory(${MATH_DIR})

include(${IM_DIR}/im.cmake OPTIONAL)
include(${CORE_DIR}/aaesim.cmake OPTIONAL)
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
            ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
            COMMENT "Generating API documentation with Doxygen" VERBATIM
            )
endif(DOXYGEN_FOUND)

# add installation information (will also add install target to makefile)
# NOTE: this works if using: make install/local. However, it's not clear how useful that installation actually is because the libs are all missing.
SET(DEPLOY_ROOT_PATH /devel/${PROJECT_NAME}/bamboo)
SET(INSTALL_DIR ${DEPLOY_ROOT_PATH}/${aaesim_VERSION})
SET(CMAKE_INSTALL_PREFIX ${INSTALL_DIR}) # so that log4cplus installs itself without errors
# INSTALL(TARGETS aaesim DESTINATION ${INSTALL_DIR})
