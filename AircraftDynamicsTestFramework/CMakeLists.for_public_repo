cmake_minimum_required(VERSION 3.14)
project(aaesim VERSION 4.3.0.1) # the nano value is a boolean. 1 == SNAPSHOT, 0 == release

set (CMAKE_CXX_STANDARD 20)

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

CPMAddPackage(
    NAME log4cplus
    GITHUB_REPOSITORY log4cplus/log4cplus
    GIT_TAG REL_2_0_8
    OPTIONS
     "BUILD_SHARED_LIBS FALSE"
     "LOG4CPLUS_BUILD_TESTING FALSE"
     "LOG4CPLUS_BUILD_LOGGINGSERVER FALSE"
)

# Create some common paths
if (unitslib_ADDED)
    set(unitslib_INCLUDE_DIRS ${unitslib_SOURCE_DIR}/units-2.1/src/)
endif ()
if (log4cplus_ADDED)
    set (LOG4CPLUS_DIRS 
        ${log4cplus_BINARY_DIR}/include
        ${log4cplus_SOURCE_DIR}/include)
endif ()

set (LOADER_DIR          ${CMAKE_CURRENT_SOURCE_DIR}/Loader)
set (PUBLIC_DIR          ${CMAKE_CURRENT_SOURCE_DIR}/Public)
set (UNITTEST_DIR        ${CMAKE_CURRENT_SOURCE_DIR}/unittest)
set (FRAMEWORK_DIR       ${CMAKE_CURRENT_SOURCE_DIR}/AircraftDynamicsTestFramework)
set (aaesim_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include )

add_subdirectory(${LOADER_DIR})
add_subdirectory(${PUBLIC_DIR})

include(${UNITTEST_DIR}/unittest.cmake OPTIONAL)
include(${FRAMEWORK_DIR}/framework.cmake OPTIONAL)
