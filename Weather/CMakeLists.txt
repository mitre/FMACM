cmake_minimum_required(VERSION 3.14)

CPMAddPackage(
    NAME caasdwind
    GIT_REPOSITORY https://mustache.mitre.org/scm/aaes/windapi.git
    GIT_TAG 0e64fb6
    OPTIONS
     "BUILD_SHARED_LIBS FALSE"
)

set(caasdwind_INCLUDE_DIRS ${caasdwind_SOURCE_DIR}/windapi/src)

find_path(ECCODES_SYSTEM_INCLUDES NAMES grib_api.h)

set(WEATHER_SOURCE_FILES
    WindCaasd.cpp
    WindFileCache.cpp
    WindLegacy.cpp
)

set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)

add_library(weather STATIC ${WEATHER_SOURCE_FILES})
target_include_directories(weather PUBLIC
    ${weather_INCLUDE_DIR}
    ${caasdwind_INCLUDE_DIRS}
    ${ECCODES_SYSTEM_INCLUDES}
)
target_link_libraries(weather
        windapi pub log4cplusS )
