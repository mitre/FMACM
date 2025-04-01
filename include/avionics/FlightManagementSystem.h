// ****************************************************************************
// NOTICE
//
// This work was produced for the U.S. Government under Contract 693KA8-22-C-00001
// and is subject to Federal Aviation Administration Acquisition Management System
// Clause 3.5-13, Rights In Data-General, Alt. III and Alt. IV (Oct. 1996).
//
// The contents of this document reflect the views of the author and The MITRE
// Corporation and do not necessarily reflect the views of the Federal Aviation
// Administration (FAA) or the Department of Transportation (DOT). Neither the FAA
// nor the DOT makes any warranty or guarantee, expressed or implied, concerning
// the content or accuracy of these views.
//
// For further information, please contact The MITRE Corporation, Contracts Management
// Office, 7515 Colshire Drive, McLean, VA 22102-7539, (703) 983-6000.
//
// 2023 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#pragma once

#include "public/AircraftControl.h"
#include "public/AircraftState.h"
#include "public/AircraftIntent.h"
#include "public/PrecalcConstraint.h"
#include "public/PrecalcWaypoint.h"
#include "public/PredictedWindEvaluator.h"
#include "public/SimulationTime.h"
#include "public/WaypointPassingMonitor.h"
#include "public/Wgs84PrecalcWaypoint.h"
#include "public/GuidanceCalculator.h"
#include "avionics/FmsIntegratedApplication.h"
#include "avionics/Wgs84KineticClimbPredictor.h"
#include "avionics/Wgs84KineticDescentPredictor.h"
#include "avionics/NavigationSensor.h"
#include "avionics/Wgs84KineticUnionPredictor.h"
#include "avionics/Wgs84AlongPathDistanceMonitor.h"

class FlightManagementSystem {
  public:
   struct InitialConditions {
      Units::Length altitude_msl;
      Units::Speed ias;
      double mach;
      aaesim::open_source::GuidanceFlightPhase flight_phase;
      EarthModel::GeodeticPosition wgs84_position;
      EarthModel::LocalPositionEnu euclidean_position;
      Units::SignedAngle forward_course_enu_along_path;
      double mass_percentile;
      Units::Mass mass;
   };

   struct FmsState {
      Units::Length along_path_distance_to_final_waypoint;
      Units::Length cross_track_error;
      aaesim::LatitudeLongitudePoint estimated_position;
      aaesim::open_source::Guidance current_guidance;
      aaesim::open_source::AircraftState navigation_state;
      aaesim::Wgs84PrecalcWaypoint active_waypoint;
   };

   FlightManagementSystem(aaesim::Wgs84KineticClimbPredictor &kinetic_climb_predictor,
                          aaesim::Wgs84KineticDescentPredictor &kinetic_trajectory_predictor,
                          std::shared_ptr<aaesim::open_source::PredictedWindEvaluator> predicted_wind_evaluator,
                          std::shared_ptr<aaesim::open_source::AircraftControl> aircraft_control,
                          std::shared_ptr<aaesim::NavigationSensor> navigation_sensor,
                          const bool all_trajectory_updates,
                          std::shared_ptr<aaesim::avionics::FmsIntegratedApplication> fms_integrated_application);

   const aaesim::open_source::WeatherPrediction &GetDescentPredictedWinds() const;

   std::pair<const aaesim::open_source::Guidance, std::shared_ptr<aaesim::open_source::AircraftControl>> Update(
         const aaesim::open_source::SimulationTime &simtime, const aaesim::open_source::AircraftState &truth_state);

   virtual void Initialize(const std::string &ac_performance_name, const AircraftIntent &aircraft_intent,
                           const aaesim::open_source::WeatherPrediction &predicted_weather);

   bool IsFinishedRoute() const;

   void DoPathManagement(const aaesim::open_source::AircraftState &state);

   const aaesim::Wgs84KineticDescentPredictor &GetDescentTrajectoryPredictor() const;

   const std::vector<aaesim::open_source::AircraftState> GetNavigationStates() const;

   virtual void UpdateTrajectoryPrediction(const aaesim::open_source::AircraftState &aircraft_state);

   std::pair<Units::Length, Units::Length> GetFinalWaypointEuclideanPosition() const;

   Units::Length GetFinalWaypointAltitudeMsl() const;

   aaesim::open_source::GuidanceFlightPhase GetActiveGuidanceFlightPhase() const;

   InitialConditions GetInitialConditions() const;

   std::shared_ptr<const aaesim::Wgs84TrajectoryPredictor> GetActiveTrajectoryPredictor() const;

   std::shared_ptr<aaesim::open_source::FixedMassAircraftPerformance> GetAircraftPerformance() const;

   std::map<Units::Time, const aaesim::Wgs84PrecalcWaypoint> GetSequencedWaypoints() const;

   aaesim::LatitudeLongitudePoint GetCurrentEstimatedPosition() const;

   const std::map<Units::Time,
                  std::tuple<aaesim::open_source::GuidanceFlightPhase, std::vector<aaesim::Wgs84HorizontalPathSegment>,
                             VerticalPath, const aaesim::open_source::WeatherPrediction>>
         GetFmsTrajectoryPredictionsByTime() const;

   std::map<Units::Time, FmsState> GetFmsStateHistory() const;

   const AircraftIntent GetAircraftIntent() const;

   std::shared_ptr<aaesim::avionics::FmsIntegratedApplication> GetFmsApplication() const;

  private:
   static log4cplus::Logger m_logger;

   aaesim::open_source::Guidance DoGuidance(const aaesim::open_source::AircraftState &aircraft_state);

   std::shared_ptr<aaesim::open_source::AircraftControl> GetActiveController();

   virtual aaesim::open_source::AircraftState DoNavigation(const aaesim::open_source::AircraftState &truth_state);

   void AdvanceGuidanceFlightPhase();

   void SetGuidanceCalculatorForFlightPhase(const Units::Time &simulation_time);

   void PrepareAircraftPerformanceForCruise();

   void PerformWaypointSequence(const aaesim::open_source::AircraftState &state,
                                const aaesim::Wgs84PrecalcWaypoint &waypoint);

   void LogPathManagementData(
         const aaesim::open_source::AircraftState &current_state,
         const aaesim::avionics::Wgs84AlongPathDistanceMonitor::ResultData &path_managment_result) const;

   void LogGuidanceTargets(const aaesim::open_source::Guidance &guidance,
                           const aaesim::open_source::AircraftState &aircraft_state) const;

   void ConstructPathMonitorsFromKineticPredictions();

   FmsState CollectCurrentFmsState(const aaesim::open_source::Guidance &current_guidance,
                                   const aaesim::open_source::AircraftState &navigation_state);
   aaesim::Wgs84KineticUnionPredictor m_guidance_path_predictor;
   std::shared_ptr<aaesim::open_source::AircraftControl> m_aircraft_controller{};
   std::vector<aaesim::open_source::AircraftState> m_navigation_states{};
   AircraftIntent m_aircaft_intent;
   aaesim::open_source::Guidance m_previous_guidance;
   std::shared_ptr<aaesim::open_source::GuidanceCalculator> m_active_guidance_calculator{};
   std::shared_ptr<aaesim::open_source::FixedMassAircraftPerformance> m_aircraft_performance{};
   bool m_allow_trajectory_updates{false};
   std::shared_ptr<aaesim::avionics::Wgs84AlongPathDistanceMonitor> m_decrementing_distance_calculator{};
   std::shared_ptr<aaesim::avionics::Wgs84AlongPathDistanceMonitor> m_climb_phase_distance_calculator{};
   Units::Length m_estimated_wgs84_distance_to_final_waypoint_active_phase{Units::negInfinity()};
   Units::Length m_estimated_wgs84_distance_to_final_waypoint{Units::negInfinity()};
   InitialConditions m_fms_initial_conditions;
   aaesim::open_source::GuidanceFlightPhase m_current_guidance_phase{
         aaesim::open_source::GuidanceFlightPhase::TAKEOFF_ROLL};
   aaesim::LatitudeLongitudePoint m_estimated_ellipsoidal_position{};
   bool m_all_waypoints_sequenced{false};
   std::map<Units::Time, const aaesim::Wgs84PrecalcWaypoint> m_waypoints_sequenced{};
   std::vector<
         std::pair<std::shared_ptr<aaesim::open_source::WaypointPassingMonitor>, const aaesim::Wgs84PrecalcWaypoint>>
         m_path_monitors{};
   std::map<Units::Time,
            std::tuple<aaesim::open_source::GuidanceFlightPhase, std::vector<aaesim::Wgs84HorizontalPathSegment>,
                       VerticalPath, const aaesim::open_source::WeatherPrediction>>
         m_fms_trajectory_predictions_by_time{};
   std::shared_ptr<aaesim::NavigationSensor> m_navigation_sensor{};
   std::map<Units::Time, FmsState> m_state_history{};
   std::shared_ptr<aaesim::avionics::FmsIntegratedApplication> m_fms_application{};
};

inline const aaesim::open_source::WeatherPrediction &FlightManagementSystem::GetDescentPredictedWinds() const {
   return m_guidance_path_predictor.GetDescentWeatherPrediction();
}

inline const std::vector<aaesim::open_source::AircraftState> FlightManagementSystem::GetNavigationStates() const {
   return m_navigation_states;
}

inline std::shared_ptr<aaesim::open_source::AircraftControl> FlightManagementSystem::GetActiveController() {
   return m_aircraft_controller;
}

inline const aaesim::Wgs84KineticDescentPredictor &FlightManagementSystem::GetDescentTrajectoryPredictor() const {
   return m_guidance_path_predictor.GetDescentPredictor();
}

inline std::pair<Units::Length, Units::Length> FlightManagementSystem::GetFinalWaypointEuclideanPosition() const {
   return std::make_pair(m_aircaft_intent.GetWaypointX(m_aircaft_intent.GetNumberOfWaypoints() - 1),
                         m_aircaft_intent.GetWaypointY(m_aircaft_intent.GetNumberOfWaypoints() - 1));
}

inline Units::Length FlightManagementSystem::GetFinalWaypointAltitudeMsl() const {
   return m_aircaft_intent.GetRouteData().m_nominal_altitude[m_aircaft_intent.GetNumberOfWaypoints() - 1];
}

inline aaesim::open_source::GuidanceFlightPhase FlightManagementSystem::GetActiveGuidanceFlightPhase() const {
   return m_current_guidance_phase;
}

inline FlightManagementSystem::InitialConditions FlightManagementSystem::GetInitialConditions() const {
   return m_fms_initial_conditions;
}

inline std::shared_ptr<const aaesim::Wgs84TrajectoryPredictor> FlightManagementSystem::GetActiveTrajectoryPredictor()
      const {
   switch (GetActiveGuidanceFlightPhase()) {
      case aaesim::open_source::GuidanceFlightPhase::TAKEOFF_ROLL:
      case aaesim::open_source::GuidanceFlightPhase::CLIMB:
         return std::make_shared<const aaesim::Wgs84KineticClimbPredictor>(
               m_guidance_path_predictor.GetClimbPredictor());
      case aaesim::open_source::GuidanceFlightPhase::CRUISE_DESCENT:
         return std::make_shared<const aaesim::Wgs84KineticDescentPredictor>(
               m_guidance_path_predictor.GetDescentPredictor());
      default:
         throw std::logic_error("Invalid guidance flight phase encountered: " +
                                aaesim::open_source::GuidanceFlightPhaseAsString(m_current_guidance_phase));
   }
}

inline std::shared_ptr<aaesim::open_source::FixedMassAircraftPerformance>
      FlightManagementSystem::GetAircraftPerformance() const {
   return m_aircraft_performance;
}

inline void FlightManagementSystem::AdvanceGuidanceFlightPhase() {
   switch (m_current_guidance_phase) {
      case aaesim::open_source::GuidanceFlightPhase::TAKEOFF_ROLL:
         m_current_guidance_phase = aaesim::open_source::GuidanceFlightPhase::CLIMB;
         break;
      case aaesim::open_source::GuidanceFlightPhase::CLIMB:
         m_current_guidance_phase = aaesim::open_source::GuidanceFlightPhase::CRUISE_DESCENT;
         break;
      default:
         throw std::logic_error("Invalid guidance flight phase encountered: " +
                                aaesim::open_source::GuidanceFlightPhaseAsString(m_current_guidance_phase));
   }
}

inline void FlightManagementSystem::PrepareAircraftPerformanceForCruise() {
   BoundedValue<double, 0, 1> mass_fraction(m_guidance_path_predictor.GetDescentPredictor()
                                                  .GetVertPredictor()
                                                  ->GetAircraftPerformance()
                                                  ->GetAircraftMassPercentile());
   m_aircraft_performance->UpdateMassFraction(mass_fraction);
}

inline std::map<Units::Time, const aaesim::Wgs84PrecalcWaypoint> FlightManagementSystem::GetSequencedWaypoints() const {
   return m_waypoints_sequenced;
}

inline aaesim::LatitudeLongitudePoint FlightManagementSystem::GetCurrentEstimatedPosition() const {
   return m_estimated_ellipsoidal_position;
}
inline const std::map<
      Units::Time, std::tuple<aaesim::open_source::GuidanceFlightPhase, std::vector<aaesim::Wgs84HorizontalPathSegment>,
                              VerticalPath, const aaesim::open_source::WeatherPrediction>>
      FlightManagementSystem::GetFmsTrajectoryPredictionsByTime() const {
   return m_fms_trajectory_predictions_by_time;
}

inline std::map<Units::Time, FlightManagementSystem::FmsState> FlightManagementSystem::GetFmsStateHistory() const {
   return m_state_history;
}

inline const AircraftIntent FlightManagementSystem::GetAircraftIntent() const { return m_aircaft_intent; }

inline std::shared_ptr<aaesim::avionics::FmsIntegratedApplication> FlightManagementSystem::GetFmsApplication() const {
   return m_fms_application;
}
