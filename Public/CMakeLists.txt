cmake_minimum_required(VERSION 3.14)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
include(${PROJECT_SOURCE_DIR}/.cmake/get_cpm.cmake)

CPMAddPackage(
        NAME geolib_idealab
        GIT_REPOSITORY https://github.com/mitre/geodetic_library.git
        GIT_TAG v3.2.8
)
if (geolib_idealab_ADDED)
    # set a variable for the includes. Use the variable later inside target_include_directories()
    set(geolib_idealab_INCLUDE_DIRS ${geolib_idealab_SOURCE_DIR}/include)
endif ()

CPMAddPackage(
        NAME minicsv
        GIT_REPOSITORY https://github.com/shaovoon/minicsv.git
        GIT_TAG v1.8.7
        DOWNLOAD_ONLY TRUE
)
if (minicsv_ADDED)
        set(minicsv_INCLUDE_DIR ${minicsv_SOURCE_DIR})
endif ()

CPMAddPackage(
    NAME nlohmann_json
    GITHUB_REPOSITORY nlohmann/json
    VERSION 3.9.1
)

set(SOURCE_FILES
        ADSBSVReport.cpp
        AircraftCalculations.cpp
        AircraftControl.cpp
        AircraftIntent.cpp
        AircraftSpeed.cpp
        AircraftState.cpp
        Atmosphere.cpp
        BlendWindsVerticallyByAltitude.cpp
        CalcWindGradControl.cpp
        ConfigurationFileReader.cpp
        ControlCommands.cpp
        CoreUtils.cpp
        EarthModel.cpp
        EllipsoidalEarthModel.cpp
        Environment.cpp
        EuclideanTrajectoryPredictor.cpp
        FlightEnvelopeSpeedLimiter.cpp
        HorizontalPath.cpp
        HorizontalTurnPath.cpp
        KinematicDescent4DPredictor.cpp
        KinematicTrajectoryPredictor.cpp
        LegacyPositionEstimator.cpp
        LocalTangentPlane.cpp
        Logging.cpp
        LoggingLoadable.cpp
        NullSpeedLimiter.cpp
        NullWindEvaluator.cpp
        PassThroughAssap.cpp
        PrecalcConstraint.cpp
        PrecalcWaypoint.cpp
        ScenarioUtils.cpp
        SimulationTime.cpp
        SingleTangentPlaneSequence.cpp
        SpeedOnPitchControl.cpp
        SpeedOnThrustControl.cpp
        StatisticalPilotDelay.cpp
        StereographicProjection.cpp
        TangentPlaneSequence.cpp
        ThreeDOFDynamics.cpp
        VectorDifferenceWindEvaluator.cpp
        VerticalPath.cpp
        VerticalPredictor.cpp
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
        TvReader.cpp
        GeolibUtils.cpp
        LatitudeLongitudePoint.cpp
        LineOnEllipsoid.cpp
        ArcOnEllipsoid.cpp
        EuclideanWaypointMonitor.cpp
        CustomMath.cpp
        DMatrix.cpp
        DVector.cpp
        InvalidIndexException.cpp
        RandomGenerator.cpp
        Wgs84PrecalcWaypoint.cpp
        USStandardAtmosphere1976.cpp
        ZeroWindTrueWeatherOperator.cpp
        FullWindTrueWeatherOperator.cpp
)

set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)

add_library(pub STATIC ${SOURCE_FILES})
target_include_directories(pub PUBLIC
        ${geolib_idealab_INCLUDE_DIRS}
        ${aaesim_INCLUDE_DIRS}
        ${minicsv_INCLUDE_DIR})
target_link_libraries(pub
        loader 
        geolib 
        log4cplusS 
        nlohmann_json::nlohmann_json 
        "$<$<AND:$<CXX_COMPILER_ID:GNU>,$<VERSION_LESS:$<CXX_COMPILER_VERSION>,9.1>>:-lstdc++fs>")
