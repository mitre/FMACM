cmake_minimum_required(VERSION 3.14)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

set(DATA_READER_FILES
        EnvReader.cpp
        ForeWindReader.cpp
        RefReader.cpp
        HfpReaderPre2020.cpp
        HfpReader2020.cpp
)

set(SOURCE_FILES
        AircraftIntentFromFile.cpp
        Bada.cpp
        BadaPerformanceCalculator.cpp
        fmacm.cpp
        IMSpeedCommandFile.cpp
        TestFrameworkAircraft.cpp
        TestFrameworkApplication.cpp
        TestFrameworkFMS.cpp
        TestFrameworkScenario.cpp
        TrajectoryFromFile.cpp
        WeatherTruthByTime.cpp
        WeatherTruthByDistanceToGo.cpp
)

set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)

add_library(framework STATIC ${SOURCE_FILES} ${DATA_READER_FILES})
target_include_directories(framework PUBLIC
        ${aaesim_INCLUDE_DIRS} ${geolib_idealab_INCLUDE_DIRS})
target_link_libraries(framework
        loader utility math geolib pub log4cplusS)
target_compile_definitions(framework PUBLIC -D_LINUX_ -Dunix)
