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

set(FMACM_BADA_TARGET "" CACHE STRING "Optional BADA implementation target")
set(FMACM_FIM_SAMPLE_ALGORITHM_TARGET "" CACHE STRING "Optional FIM sample algorithm target")

# OPTIONAL Libraries ------
# These optional local include files can define the legacy BADA_LIBRARY and
# SAMPLE_ALGORITHM_LIBRARY variables. Prefer the FMACM_*_TARGET cache hooks
# for downstream composition.
include(bada_library.cmake OPTIONAL) 
include(sample_algorithm_library.cmake OPTIONAL) 
if("${FMACM_BADA_TARGET}" STREQUAL "" AND DEFINED BADA_LIBRARY)
   set(FMACM_BADA_TARGET "${BADA_LIBRARY}")
endif()
if("${FMACM_FIM_SAMPLE_ALGORITHM_TARGET}" STREQUAL "" AND DEFINED SAMPLE_ALGORITHM_LIBRARY)
   set(FMACM_FIM_SAMPLE_ALGORITHM_TARGET "${SAMPLE_ALGORITHM_LIBRARY}")
endif()
# ------

add_library(framework STATIC ${SOURCE_FILES} ${DATA_READER_FILES} ${DATA_LOADER_FILES} ${DATA_WRITER_FILES})
add_library(mitre::oss::fmacm_framework ALIAS framework)
set_target_properties(framework PROPERTIES
   ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib
)
target_include_directories(framework PUBLIC
        ${aaesim_INCLUDE_DIRS})
target_link_libraries(framework
   PUBLIC
   mitre::fsloader
   log4cplus::log4cplus
   mitre::oss::simcore)
if (FMACM_BADA_TARGET)
   target_link_libraries(framework PUBLIC ${FMACM_BADA_TARGET})
   target_compile_definitions(framework PUBLIC MITRE_BADA3_LIBRARY)
endif()
if (FMACM_FIM_SAMPLE_ALGORITHM_TARGET)
   target_link_libraries(framework PUBLIC ${FMACM_FIM_SAMPLE_ALGORITHM_TARGET})
   target_compile_definitions(framework PUBLIC SAMPLE_ALGORITHM_LIBRARY)
endif()
