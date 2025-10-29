cmake_minimum_required(VERSION 3.14)
project(aaesim VERSION 5.2.0.0) # the nano value is a boolean. 1 == SNAPSHOT, 0 == release # the nano value is a boolean. 1 == SNAPSHOT, 0 == release

set (CMAKE_CXX_STANDARD 20)

if (NOT DEFINED CMAKE_BUILD_TYPE)
    set (CMAKE_BUILD_TYPE "Debug")
endif()
message ("${PROJECT_NAME}: Build type set to ${CMAKE_BUILD_TYPE}")

option(BUILD_LIBARIES_ONLY "Build ${PROJECT_NAME} libraries only, no binaries" OFF)
if(${BUILD_LIBARIES_ONLY})
    message(STATUS "${PROJECT_NAME}: only building libraries, skipping binaries")
endif()

option(BUILD_TESTING "Enable building ${PROJECT_NAME} tests" ON)
if(NOT ${BUILD_TESTING})
    message(STATUS "${PROJECT_NAME}: skipping test targets")
endif()

# Get CPM
include(${PROJECT_SOURCE_DIR}/.cmake/get_cpm.cmake)

# Include a suite of helper functions for output and building the aaesim package
include(${PROJECT_SOURCE_DIR}/.cmake/functions.cmake)

CPMAddPackage(
  NAME Ccache.cmake
  GITHUB_REPOSITORY TheLartians/Ccache.cmake
  VERSION 1.2.5
)

# CPMAddPackage(
#         NAME cppmanifest
#         GIT_REPOSITORY https://mustache.mitre.org/scm/idea/cppmanifest.git
#         GIT_TAG v1.7
#         DOWNLOAD_ONLY TRUE
# )
# if (cppmanifest_ADDED)
#     include(${cppmanifest_SOURCE_DIR}/cppmanifest_configure.cmake)
#     cppmanifest_configure(${PROJECT_SOURCE_DIR}/include/cppmanifest)
# endif()

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

CPMAddPackage(
    NAME log4cplus
    GITHUB_REPOSITORY log4cplus/log4cplus
    GIT_TAG REL_2_1_2
    OPTIONS
     "BUILD_SHARED_LIBS FALSE"
     "LOG4CPLUS_BUILD_TESTING FALSE"
     "LOG4CPLUS_BUILD_LOGGINGSERVER FALSE"
)
if (log4cplus_ADDED)
    set (LOG4CPLUS_DIRS 
        ${log4cplus_BINARY_DIR}/include
        ${log4cplus_SOURCE_DIR}/include)
endif ()

set (unitslib_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/unitsLib/units-2.1)
set (LOADER_DIR          ${CMAKE_CURRENT_SOURCE_DIR}/Loader)
set (PUBLIC_DIR          ${CMAKE_CURRENT_SOURCE_DIR}/Public)
set (FRAMEWORK_DIR       ${CMAKE_CURRENT_SOURCE_DIR}/AircraftDynamicsTestFramework)
set (aaesim_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include )
set (UNITTEST_DIR        ${CMAKE_CURRENT_SOURCE_DIR}/unittest)

add_subdirectory(${LOADER_DIR})
add_subdirectory(${PUBLIC_DIR})

include(${UNITTEST_DIR}/unittest.cmake OPTIONAL)
include(${FRAMEWORK_DIR}/framework.cmake OPTIONAL)
