cmake_minimum_required(VERSION 3.0)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

set (SOURCE_FILES
        AchieveObserver.cpp
        ADSBSVReport.cpp
        AircraftCalculations.cpp
        AircraftControl.cpp
        Aircraft.cpp
        AircraftIntent.cpp
        AircraftState.cpp
        Atmosphere.cpp
        ClosestPointMetric.cpp
        ControlCommands.cpp
        CoreUtils.cpp
        CrossTrackObserver.cpp
        DynamicsObserver.cpp
        EarthModel.cpp
        EllipsoidalEarthModel.cpp
        Environment.cpp
        Guidance.cpp
        HorizontalPath.cpp
        HorizontalTurnPath.cpp
        IMCommandObserver.cpp
        InternalObserver.cpp
        InvalidIndexException.cpp
        LocalTangentPlane.cpp
        LoggingLoadable.cpp
        MaintainMetric.cpp
        MergePointMetric.cpp
        NMObserver.cpp
        NMObserverEntry.cpp
        PrecalcConstraint.cpp
        PrecalcWaypoint.cpp
        RunFile.cpp
        Scenario.cpp
        SimulationTime.cpp
        SingleTangentPlaneSequence.cpp
        StereographicProjection.cpp
        TangentPlaneSequence.cpp
        VerticalPath.cpp
        VerticalPathObserver.cpp
        Waypoint.cpp
        WeatherPrediction.cpp
        WindStack.cpp
)

set (LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)

add_library(pub ${SOURCE_FILES})
target_include_directories(pub PUBLIC
        ${aaesim_INCLUDE_DIRS})
target_link_libraries(pub
        loader utility math)
target_compile_definitions(pub PUBLIC -D_LINUX_ -Dunix)
