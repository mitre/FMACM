cmake_minimum_required(VERSION 3.14)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

set(DATA_LOADER_FILES
   loaders/ApplicationLoader.cpp
   loaders/SpeedCommandsLoader.cpp
   loaders/GuidanceDataLoader.cpp
   loaders/FrameworkAircraftLoader.cpp
)

set(DATA_WRITER_FILES
        writers/AircraftStateWriter.cpp
)
set(DATA_READER_FILES
        EnvReader.cpp
        ForeWindReader.cpp
        RefReader.cpp
        HfpReaderPre2020.cpp
        HfpReader2020.cpp
        WaypointSequenceReader.cpp
)

set(SOURCE_FILES
        SpeedCommandsFromStaticData.cpp
        PreloadedAdsbReceiver.cpp
        TestFrameworkAircraft.cpp
        TestFrameworkScenario.cpp
        GuidanceFromStaticData.cpp
        WeatherTruthFromStaticData.cpp
        WindInterpolator.cpp
)

set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)

# OPTIONAL Libraries ------
# a bada implementation is needed to run, but the definition is 
# optional so that compiling will still occur if it is missing
include(bada_library.cmake OPTIONAL) 
#
# the DO-361A Sample Algorithm can be linked into this code, but
# its presence is not required for compiling
include(sample_algorithm_library.cmake OPTIONAL) 
# ------

add_library(framework STATIC ${SOURCE_FILES} ${DATA_READER_FILES} ${DATA_LOADER_FILES} ${DATA_WRITER_FILES})
target_include_directories(framework PUBLIC
        ${aaesim_INCLUDE_DIRS} 
        ${geolib_idealab_INCLUDE_DIRS} 
        ${LOG4CPLUS_DIRS})
target_link_libraries(framework
   ${BADA_LIBRARY}
   ${SAMPLE_ALGORITHM_LIBRARY}
   log4cplusS
   pub)
if (DEFINED BADA_LIBRARY)
   # Add a compile definition to the build
   target_compile_definitions(framework PUBLIC "MITRE_BADA3_LIBRARY")
endif()
if (DEFINED SAMPLE_ALGORITHM_LIBRARY)
   # Add a compile definition to the build 
   target_compile_definitions(framework PUBLIC "SAMPLE_ALGORITHM_LIBRARY")
endif()
