# This file is included by both AAESim and AAESimTest. Therefore, only
# common cmake descriptions should go in this file. 
ENABLE_LANGUAGE(CXX)

# Force libraries to be linked as STATIC
SET(BUILD_SHARED_LIBS false)

# Create some common paths
SET(LOADER_DIR ./Loader)
SET(UTILITY_DIR ./Utility)
SET(UNITSLIB_DIR ./unitsLib)
SET(MATH_DIR ./Math/)
SET(CORE_DIR ./Core)
SET(IM_DIR ./IntervalManagement)
SET(GROUND_DIR ./GroundSystems)
SET(PA_DIR ./PairedApproach)
SET(FRAMEWORK_DIR ./AircraftDynamicsTestFramework)
SET(CAASD_WIND_DIR /devel/caasdwind/release/v1.2.1/local/)

set (CAASD_WIND_LIBS
    ${CAASD_WIND_DIR}/lib/libwind.so
    ${CAASD_WIND_DIR}/lib/libgrib_api.so
    ${CAASD_WIND_DIR}/lib/libopenjpeg.so
)


# Add subdirectories that will become libraries
add_subdirectory(${LOADER_DIR}) # this will also build the utility library
add_subdirectory(${UNITSLIB_DIR})
# add_subdirectory(${IM_DIR})
add_subdirectory(${UTILITY_DIR})


LINK_LIBRARIES(${CAASD_WIND_LIBS})
ADD_DEFINITIONS(-D_LINUX_ -Dunix)

SET(FRAMEWORK_SRC
    ${FRAMEWORK_DIR}/AircraftIntentFromFile.cpp
    ${FRAMEWORK_DIR}/AircraftIntentFromFile.h
    ${FRAMEWORK_DIR}/IMSpeedCommandFile.cpp
    ${FRAMEWORK_DIR}/IMSpeedCommandFile.h
    ${FRAMEWORK_DIR}/TestFrameworkAircraft.cpp
    ${FRAMEWORK_DIR}/TestFrameworkAircraft.h
    ${FRAMEWORK_DIR}/TestFrameworkApplication.cpp
    ${FRAMEWORK_DIR}/TestFrameworkApplication.h
    ${FRAMEWORK_DIR}/TestFrameworkDynamics.cpp
    ${FRAMEWORK_DIR}/TestFrameworkDynamics.h
    ${FRAMEWORK_DIR}/TestFrameworkFMS.cpp
    ${FRAMEWORK_DIR}/TestFrameworkFMS.h
    ${FRAMEWORK_DIR}/TestFrameworkScenario.cpp
    ${FRAMEWORK_DIR}/TestFrameworkScenario.h
    ${FRAMEWORK_DIR}/TrajectoryFromFile.cpp
    ${FRAMEWORK_DIR}/TrajectoryFromFile.h
)

# Create some SRC containers
SET(MATH_SRC         
  ${MATH_DIR}CustomMath.cpp
  ${MATH_DIR}CustomMath.h
  ${MATH_DIR}Statistics.cpp
  ${MATH_DIR}Statistics.h
  ${MATH_DIR}RandomGenerator.cpp
  ${MATH_DIR}RandomGenerator.h
)

set(CORE_SRC
        ${CORE_DIR}/FlightManagementSystem.cpp
        ${CORE_DIR}/FlightManagementSystem.h
        ${CORE_DIR}/FmsWithNavigationNoise.cpp
        ${CORE_DIR}/FmsWithNavigationNoise.h
        ${CORE_DIR}/FmsLoader.cpp
        ${CORE_DIR}/FmsLoader.h
        ${CORE_DIR}/AircraftSensors/ADSBDevice.cpp
        ${CORE_DIR}/AircraftSensors/ADSBDevice.h
        ${CORE_DIR}/Aircraft.cpp
        ${CORE_DIR}/Aircraft.h
        ${CORE_DIR}/AircraftIntent.cpp
        ${CORE_DIR}/AircraftIntent.h
        ${CORE_DIR}/AircraftState.cpp
        ${CORE_DIR}/AircraftState.h
        ${CORE_DIR}/ADSBEther.h
        ${CORE_DIR}/ADSBEther.cpp
        ${CORE_DIR}/AircraftCalculations.h
        ${CORE_DIR}/AircraftCalculations.cpp
        ${CORE_DIR}/AircraftSensors/ADSBReceiver.h
        ${CORE_DIR}/AircraftSensors/ADSBReceiver.cpp
        ${CORE_DIR}/AircraftSensors/ADSBSVReport.h
        ${CORE_DIR}/AircraftSensors/ADSBSVReport.cpp
        ${CORE_DIR}/AircraftSensors/ADSBTransmitter.h
        ${CORE_DIR}/AircraftSensors/ADSBTransmitter.cpp
        ${CORE_DIR}/ASSAPExtrapolate1Hz.h
        ${CORE_DIR}/ASSAPExtrapolate1Hz.cpp
        ${CORE_DIR}/ASSAP.h
        ${CORE_DIR}/ASSAP.cpp
        ${CORE_DIR}/ASSAPImpl.h
        ${CORE_DIR}/ASSAPImpl.cpp
        ${CORE_DIR}/ASSAPLegacy.h
        ${CORE_DIR}/ASSAPLegacy.cpp
        ${CORE_DIR}/ASSAPNone.h
        ${CORE_DIR}/ASSAPNone.cpp
        ${CORE_DIR}/Atmosphere.h
        ${CORE_DIR}/Atmosphere.cpp
        ${CORE_DIR}/Bada.cpp
        ${CORE_DIR}/Bada.h
        ${CORE_DIR}/BadaWithCalc.cpp
        ${CORE_DIR}/BadaWithCalc.h
        ${CORE_DIR}/CalcWindGradControl.cpp
        ${CORE_DIR}/CalcWindGradControl.h
        ${CORE_DIR}/constants.cpp
        ${CORE_DIR}/constants.h
        ${CORE_DIR}/ClosestPointMetric.h
        ${CORE_DIR}/ClosestPointMetric.cpp
        ${CORE_DIR}/CrossTrackObserver.h
        ${CORE_DIR}/CrossTrackObserver.cpp
        ${CORE_DIR}/DMatrix.cpp
        ${CORE_DIR}/DMatrix.h
        ${CORE_DIR}/DVector.cpp
        ${CORE_DIR}/DVector.h
        ${CORE_DIR}/DelayBuffer.h
        ${CORE_DIR}/DelayBuffer.cpp
        ${CORE_DIR}/DistanceObserver.h
        ${CORE_DIR}/DistanceObserver.cpp
        ${CORE_DIR}/DiveDescent4DPredictor.h
        ${CORE_DIR}/DiveDescent4DPredictor.cpp
        ${CORE_DIR}/DriveDescent4DPredictor.h
        ${CORE_DIR}/DriveDescent4DPredictor.cpp
        ${CORE_DIR}/DynamicsObserver.h
        ${CORE_DIR}/DynamicsObserver.cpp
        ${CORE_DIR}/EarthModel.cpp
        ${CORE_DIR}/EarthModel.h
        ${CORE_DIR}/EllipsoidalEarthModel.h
        ${CORE_DIR}/EllipsoidalEarthModel.cpp
        ${CORE_DIR}/Environment.h
        ${CORE_DIR}/Environment.cpp
        ${CORE_DIR}/GeometricDescent4DPredictor.h
        ${CORE_DIR}/GeometricDescent4DPredictor.cpp
        ${CORE_DIR}/Guidance.cpp
        ${CORE_DIR}/Guidance.h
        ${CORE_DIR}/HorizontalPath.cpp
        ${CORE_DIR}/HorizontalPath.h
        ${CORE_DIR}/HorizontalPathObserver.cpp
        ${CORE_DIR}/HorizontalPathObserver.h
        ${CORE_DIR}/HorizontalTurnPath.cpp
        ${CORE_DIR}/HorizontalTurnPath.h
        ${CORE_DIR}/IMCommandObserver.h
        ${CORE_DIR}/IMCommandObserver.cpp
        ${CORE_DIR}/IMUtils.cpp
        ${CORE_DIR}/IMUtils.h
        ${CORE_DIR}/InternalObserver.cpp
        ${CORE_DIR}/InternalObserver.h
        ${CORE_DIR}/InvalidIndexException.cpp
        ${CORE_DIR}/InvalidIndexException.h
        ${CORE_DIR}/KineticDescent4DPredictor.cpp
        ${CORE_DIR}/KineticDescent4DPredictor.h
        ${CORE_DIR}/KineticTrajectoryPredictor.cpp
        ${CORE_DIR}/KineticTrajectoryPredictor.h
        ${CORE_DIR}/LocalTangentPlane.cpp
        ${CORE_DIR}/LocalTangentPlane.h
        ${CORE_DIR}/Logging.cpp
        ${CORE_DIR}/Logging.h
        ${CORE_DIR}/LoggingLoadable.cpp
        ${CORE_DIR}/LoggingLoadable.h
        ${CORE_DIR}/micros.h
        ${CORE_DIR}/MaintainMetric.cpp
        ${CORE_DIR}/MaintainMetric.h
        ${CORE_DIR}/MergePointMetric.cpp
        ${CORE_DIR}/MergePointMetric.h
        ${CORE_DIR}/NAC.cpp
        ${CORE_DIR}/NAC.h
        ${CORE_DIR}/NIC.h
        ${CORE_DIR}/NIC.cpp
        ${CORE_DIR}/NavigationSensor.cpp
        ${CORE_DIR}/NavigationSensor.h
        ${CORE_DIR}/NMObserver.cpp
        ${CORE_DIR}/NMObserverEntry.cpp
        ${CORE_DIR}/NMObserver.h
        ${CORE_DIR}/NMObserverEntry.h
        ${CORE_DIR}/PilotDelay.cpp
        ${CORE_DIR}/PilotDelay.h
        ${CORE_DIR}/PrecalcConstraint.cpp
        ${CORE_DIR}/PrecalcConstraint.h
        ${CORE_DIR}/PrecalcWaypoint.cpp
        ${CORE_DIR}/PrecalcWaypoint.h
        ${CORE_DIR}/PredictedWindEvaluator.h
        ${CORE_DIR}/RunFile.cpp
        ${CORE_DIR}/RunFile.h
        ${CORE_DIR}/Scenario.cpp
        ${CORE_DIR}/Scenario.h
        ${CORE_DIR}/ExternalSensors.cpp
        ${CORE_DIR}/ExternalSensors.h
        ${CORE_DIR}/SimpleAircraft.cpp
        ${CORE_DIR}/SimpleAircraft.h
        ${CORE_DIR}/SimulationTime.cpp
        ${CORE_DIR}/SimulationTime.h
        ${CORE_DIR}/SpeedProfile.cpp
        ${CORE_DIR}/SpeedProfile.h
        ${CORE_DIR}/State.cpp
        ${CORE_DIR}/State.h
        ${CORE_DIR}/SSRTracker.cpp
        ${CORE_DIR}/SSRTracker.h
        ${CORE_DIR}/SSRTracks.cpp
        ${CORE_DIR}/SSRTracks.h
        ${CORE_DIR}/StereographicProjection.cpp
        ${CORE_DIR}/FirstOrderGaussMarkovProcess.cpp
        ${CORE_DIR}/StereographicProjection.h
        ${CORE_DIR}/FirstOrderGaussMarkovProcess.h
        ${CORE_DIR}/AchieveObserver.cpp
        ${CORE_DIR}/AchieveObserver.h
        ${CORE_DIR}/TangentPlaneSequence.cpp
        ${CORE_DIR}/TangentPlaneSequence.h
        ${CORE_DIR}/ThreeDOFDynamics.cpp
        ${CORE_DIR}/ThreeDOFDynamics.h
        ${CORE_DIR}/Track.cpp
        ${CORE_DIR}/Track.h
        ${CORE_DIR}/TrajectoryPredictor.cpp
        ${CORE_DIR}/TrajectoryPredictor.h
        ${CORE_DIR}/TrueDistances.cpp
        ${CORE_DIR}/TrueDistances.h
        ${CORE_DIR}/TSE.cpp
        ${CORE_DIR}/TSE.h
        ${CORE_DIR}/TSENoise.cpp
        ${CORE_DIR}/TSENoise.h
        ${CORE_DIR}/UVector.h
        ${CORE_DIR}/VectorDifferenceWindEvaluator.cpp
        ${CORE_DIR}/VectorDifferenceWindEvaluator.h
        ${CORE_DIR}/VerticalPath.h
        ${CORE_DIR}/VerticalPath.cpp
        ${CORE_DIR}/VerticalPathObserver.cpp
        ${CORE_DIR}/VerticalPathObserver.h
        ${CORE_DIR}/VerticalPredictor.cpp
        ${CORE_DIR}/VerticalPredictor.h
        ${CORE_DIR}/Waypoint.cpp
        ${CORE_DIR}/Waypoint.h
        ${CORE_DIR}/WeatherPrediction.cpp
        ${CORE_DIR}/WeatherPrediction.h
        ${CORE_DIR}/Wind.h
        ${CORE_DIR}/Wind.cpp
        ${CORE_DIR}/WindAltitudes.h
        ${CORE_DIR}/WindAltitudes.cpp
        ${CORE_DIR}/WindCaasd.h
        ${CORE_DIR}/WindCaasd.cpp
        ${CORE_DIR}/WindLegacy.h
        ${CORE_DIR}/WindLegacy.cpp
        ${CORE_DIR}/WindStack.h
        ${CORE_DIR}/WindStack.cpp
        ${CORE_DIR}/WindZero.h
        ${CORE_DIR}/WindZero.cpp
        ${CORE_DIR}/ControlCommands.cpp
        ${CORE_DIR}/ControlCommands.h
        ${CORE_DIR}/AircraftControl.cpp
        ${CORE_DIR}/AircraftControl.h
        ${CORE_DIR}/SpeedOnThrustControl.cpp
        ${CORE_DIR}/SpeedOnThrustControl.h
        ${CORE_DIR}/EquationsOfMotionStateDeriv.h
        ${CORE_DIR}/EquationsOfMotionState.h
        ${CORE_DIR}/SpeedOnPitchControl.cpp
        ${CORE_DIR}/SpeedOnPitchControl.h
        ${CORE_DIR}/SingleTangentPlaneSequence.cpp
        ${CORE_DIR}/SingleTangentPlaneSequence.h
        ${CORE_DIR}/version.h
        ${CORE_DIR}/cppmanifest.h
        ${CORE_DIR}/build_info.h
        )

SET(IM_BUILD_SRC
        ${IM_DIR}/IMAircraft.cpp
        ${IM_DIR}/IMAircraft.h
        ${IM_DIR}/IMAlgorithmLoader.cpp
        ${IM_DIR}/IMAlgorithmLoader.h
        ${IM_DIR}/IMAlgorithmFile.h
        ${IM_DIR}/IMClearanceLoader.cpp
        ${IM_DIR}/IMClearanceLoader.h
        ${IM_DIR}/IMScenario.cpp
        ${IM_DIR}/IMScenario.h
        ${IM_DIR}/MOPSPredictedWindEvaluator.cpp
        ${IM_DIR}/MOPSPredictedWindEvaluator.h
        ${IM_DIR}/PredictionFile.cpp
        ${IM_DIR}/PredictionFile.h
        ${IM_DIR}/TrajectoryFile.cpp
        ${IM_DIR}/TrajectoryFile.h
        ${IM_DIR}/IMKinematicDistBasedMaintain.cpp
        ${IM_DIR}/IMKinematicDistBasedMaintain.h
        ${IM_DIR}/WaypointFile.cpp
        ${IM_DIR}/WaypointFile.h
        )

SET(GROUND_SRC
        ${GROUND_DIR}/TISBTransmitter.cpp
        ${GROUND_DIR}/TISBTransmitter.h
        )

SET(PA_SRC
        ${PA_DIR}/PAScenario.cpp
        ${PA_DIR}/PAScenario.h
        ${PA_DIR}/Paired_Approach_Aircraft_Trajectory.cpp
        ${PA_DIR}/Paired_Approach_Aircraft_Trajectory.h
        ${PA_DIR}/PA_Internal_Observer.cpp
        ${PA_DIR}/PA_Internal_Observer.h
        ${PA_DIR}/airport_class.cpp
        ${PA_DIR}/airport_class.h
        ${PA_DIR}/Aircraft_State_Vector.cpp
        ${PA_DIR}/Aircraft_State_Vector.h
        ${PA_DIR}/Paired_Approach_Application.cpp
        ${PA_DIR}/Paired_Approach_Application.h
        ${PA_DIR}/approach_record_class.cpp
        ${PA_DIR}/approach_record_class.h
        ${PA_DIR}/empirical_dist.cpp
        ${PA_DIR}/empirical_dist.h
        ${PA_DIR}/avutil.cpp
        ${PA_DIR}/avutil.h
        ${PA_DIR}/separation_standard.h
        ${PA_DIR}/sim_defs.h
        ${PA_DIR}/ads_types.h
        ${PA_DIR}/ads_defs.h
        ${PA_DIR}/asg.h
        )

set(SOURCE_FILES ${MATH_SRC} ${PA_SRC} ${CORE_SRC} ${GROUND_SRC} )

# include folders
INCLUDE_DIRECTORIES(
        ${CORE_DIR}/
        ${CORE_DIR}/AircraftSensors/
        ${MATH_DIR}
        ${LOADER_DIR}/
        ${UTILITY_DIR}/
        ${IM_DIR}/
        ${GROUND_DIR}/
        ${PA_DIR}/
        ${FRAMEWORK_DIR}/
        ${UNITSLIB_DIR}/units-2.1/scalar/
        ${UNITSLIB_DIR}/units-2.1/system/
        ${CAASD_WIND_DIR}/include/
)
