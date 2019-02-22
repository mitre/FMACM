cmake_minimum_required(VERSION 3.0)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

set(SOURCE_FILES
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
        SpeedOnPitchControl.cpp
        SpeedOnThrustControl.cpp
        StandardAtmosphere.cpp
        StereographicProjection.cpp
        TangentPlaneSequence.cpp
        ThreeDOFDynamics.cpp
        VerticalPath.cpp
        VerticalPathObserver.cpp
        Waypoint.cpp
        WeatherEstimate.cpp
        WeatherPrediction.cpp
        WeatherTruth.cpp
        Wind.cpp
        WindStack.cpp
        HorizontalPathTracker.cpp
        PositionCalculator.cpp
        AlongPathDistanceCalculator.cpp
        DirectionOfFlightCourseCalculator.cpp
        WindZero.cpp
        )

set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)

add_library(pub STATIC ${SOURCE_FILES})
target_include_directories(pub PUBLIC
        ${aaesim_INCLUDE_DIRS})
target_link_libraries(pub
        loader utility math)
target_compile_definitions(pub PUBLIC -D_LINUX_ -Dunix)
