cmake_minimum_required(VERSION 3.0)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

set(SOURCE_FILES
        AircraftIntentFromFile.cpp
        Bada.cpp
        BadaWithCalc.cpp
        fmacm.cpp
        IMSpeedCommandFile.cpp
        TestFrameworkAircraft.cpp
        TestFrameworkApplication.cpp
        TestFrameworkFMS.cpp
        TestFrameworkScenario.cpp
        TrajectoryFromFile.cpp
        WeatherTruthByTime.cpp
        )

set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)

add_library(framework STATIC ${SOURCE_FILES})
target_include_directories(framework PUBLIC
        ${aaesim_INCLUDE_DIRS})
target_link_libraries(framework
        loader utility math)
target_compile_definitions(framework PUBLIC -D_LINUX_ -Dunix)
