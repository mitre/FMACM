cmake_minimum_required(VERSION 3.14)
project(aaesim VERSION 5.3.2)

set (CMAKE_CXX_STANDARD 20)

if (NOT DEFINED CMAKE_BUILD_TYPE)
    set (CMAKE_BUILD_TYPE "Debug")
endif()
message ("${PROJECT_NAME}: Build type set to ${CMAKE_BUILD_TYPE}")

if(CMAKE_SOURCE_DIR STREQUAL PROJECT_SOURCE_DIR)
    set(FMACM_IS_TOP_LEVEL ON)
else()
    set(FMACM_IS_TOP_LEVEL OFF)
endif()

set(FMACM_BUILD_EXECUTABLE_DEFAULT ON)
if(DEFINED BUILD_LIBARIES_ONLY AND BUILD_LIBARIES_ONLY)
    set(FMACM_BUILD_EXECUTABLE_DEFAULT OFF)
endif()
option(FMACM_BUILD_EXECUTABLE "Build the FMACM executable" ${FMACM_BUILD_EXECUTABLE_DEFAULT})
if(DEFINED BUILD_LIBARIES_ONLY AND BUILD_LIBARIES_ONLY)
    message(STATUS "BUILD_LIBARIES_ONLY is deprecated; use FMACM_BUILD_EXECUTABLE instead")
endif()
if(NOT FMACM_BUILD_EXECUTABLE)
    message(STATUS "${PROJECT_NAME}: only building libraries, skipping executable")
endif()

set(FMACM_BUILD_TESTING_DEFAULT OFF)
if(FMACM_IS_TOP_LEVEL)
    set(FMACM_BUILD_TESTING_DEFAULT ON)
    if(DEFINED BUILD_TESTING)
        set(FMACM_BUILD_TESTING_DEFAULT ${BUILD_TESTING})
    endif()
endif()
option(FMACM_BUILD_TESTING "Build ${PROJECT_NAME} tests" ${FMACM_BUILD_TESTING_DEFAULT})
if(NOT FMACM_BUILD_TESTING)
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

if(NOT TARGET mitre::fsloader)
    CPMAddPackage(
        NAME fsloader
        GIT_REPOSITORY https://github.com/mitre/fsloader.git
        GIT_TAG 1.1.0
        OPTIONS
         "FSLOADER_BUILD_EXAMPLE OFF"
         "BUILD_SHARED_LIBS FALSE"
    )
endif()

if(NOT TARGET mitre::oss::simcore)
    CPMAddPackage(
        NAME aircraft_simulation_core
        GIT_REPOSITORY https://github.com/mitre/aircraft_simulation_core.git
        GIT_TAG a84dc78805bf35b6a45c68ac99f1c47fe47ad892 # FIXME switch to a release tag when available
        OPTIONS
         "SIMCORE_BUILD_TESTING OFF"
    )
endif()

set (FRAMEWORK_DIR       ${CMAKE_CURRENT_SOURCE_DIR}/AircraftDynamicsTestFramework)
set (aaesim_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include )
set (UNITTEST_DIR        ${CMAKE_CURRENT_SOURCE_DIR}/unittest)

include(${FRAMEWORK_DIR}/framework.cmake OPTIONAL)
if(FMACM_BUILD_TESTING)
    include(${UNITTEST_DIR}/unittest.cmake OPTIONAL)
endif()
