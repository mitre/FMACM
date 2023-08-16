cmake_minimum_required(VERSION 3.14)



set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fno-strict-aliasing -Wunused-result")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -Wno-sign-compare -O1 -g3 -Wunused-result")

CPMAddPackage(
      NAME googletest
      GITHUB_REPOSITORY google/googletest
      GIT_TAG release-1.12.1
      VERSION 1.12.1
      OPTIONS
      "INSTALL_GTEST OFF"
      "GTEST_FORCE_SHARED_CRT ON"
      "BUILD_GMOCK ON"
)

set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)
