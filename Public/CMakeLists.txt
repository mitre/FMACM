cmake_minimum_required(VERSION 3.14)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
include(${PROJECT_SOURCE_DIR}/.cmake/CPM.cmake)

# --- geolib wgs84 library ---
CPMAddPackage(
        NAME geolib_idealab
        GIT_REPOSITORY https://github.com/mitre/geodetic_library.git
        VERSION 3.2.7-SNAPSHOT
        GIT_TAG main
)
if (geolib_idealab_ADDED)
    # set a variable for the includes. Use the variable later inside target_include_directories()
    set(geolib_idealab_INCLUDE_DIRS ${geolib_idealab_SOURCE_DIR}/include)
endif ()
# --- end geolib ---

set(SOURCE_FILES
        AchieveObserver.cpp
        ADSBSVReport.cpp
        AircraftCalculations.cpp
        AircraftControl.cpp
        Aircraft.cpp
        AircraftIntent.cpp
        AircraftSpeed.cpp
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
        EuclideanThreeDofDynamics.cpp
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
        PilotDelay.cpp
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
        DataReader.cpp
        EnvReader.cpp
        ForeWindReader.cpp
        HfpReaderPre2020.cpp
        RefReader.cpp
        TvReader.cpp
        HfpReader2020.cpp
        GeolibUtils.cpp
        LatitudeLongitudePoint.cpp
        LineOnEllipsoid.cpp
        ArcOnEllipsoid.cpp)

set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)

add_library(pub STATIC ${SOURCE_FILES})
target_include_directories(pub PUBLIC
        ${geolib_idealab_INCLUDE_DIRS}
        ${aaesim_INCLUDE_DIRS})
target_link_libraries(pub
        loader utility math geolib)
target_compile_definitions(pub PUBLIC -D_LINUX_ -Dunix)
