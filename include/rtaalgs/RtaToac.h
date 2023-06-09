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
// 2022 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#pragma once

#include <memory>

#include "public/SpeedCommandLimiter.h"
#include "public/PredictedWindEvaluator.h"
#include "rtaalgs/RTAUtils.h"
#include "aaesim/Wgs84KineticDescentPredictor.h"
#include "aaesim/Wgs84AlongPathDistanceCalculator.h"
#include "aaesim/Wgs84HorizontalPathSegment.h"
#include "public/FlightDeckApplication.h"
#include "public/WaypointPassingMonitor.h"
namespace required_time_of_arrival {
class RtaToac : public aaesim::open_source::FlightDeckApplication {
  public:
   struct AlgorithmOutputData {
      Units::Speed nominal_profile_ias;
      Units::Speed dynamics_ias;
      Units::Speed unlimited_toac_ias_command;
      Units::Speed final_toac_ias_command;
      Units::Speed delta_speed;
      Units::Length distance_to_rta_fix;
      Units::Time eta_to_rta_fix;
      Units::Time eta_error;
      Units::Time recomputed_eta_error;
      bool implement_speed_command;
      aaesim::LatitudeLongitudePoint fms_position;
      int speed_command_count;
      std::vector<VerticalPath> delta_speed_prediction;
   };

   RtaToac();
   ~RtaToac() = default;
   bool IsBlendWind() const;
   void Initialize(aaesim::open_source::FlightDeckApplicationInitializer &initializer_visitor) override;
   aaesim::open_source::Guidance Update(const SimulationTime &simtime,
                                        const aaesim::open_source::Guidance &prevguidance,
                                        const aaesim::open_source::DynamicsState &dynamicsstate,
                                        const aaesim::open_source::AircraftState &aircraftstate) override;
   void InitializeAlgorithm(std::shared_ptr<aaesim::Wgs84KineticDescentPredictor> fms_descent_predictor,
                            aaesim::open_source::OwnshipPerformanceParameters ownship_performance_parameters,
                            aaesim::open_source::OwnshipFmsPredictionParameters ownship_prediction_parameters);
   bool IsActive() const override;
   const std::vector<std::pair<Units::Time, AlgorithmOutputData>> &GetAlgorithmOutputData() const;

   class Builder {
     private:
      bool m_enable_wind_blending;
      RTAUtils::SpeedLimiterType m_speed_command_limiter;
      Units::Time m_rta_goal;
      std::string m_rta_fix;
      std::string m_wind_accuracy_evaluator;

     public:
      static std::shared_ptr<aaesim::open_source::PredictedWindEvaluator> CreatePredictedWindEvaluator(
            std::string name);
      Builder();
      ~Builder() = default;
      required_time_of_arrival::RtaToac *Build() const;
      Builder *WithWindBlending(bool enable_wind_blending);
      Builder *WithPredictedWindEvaluator(std::string predicted_wind_evaluator);
      Builder *WithSpeedCommandLimiter(RTAUtils::SpeedLimiterType speed_limiter_type);
      Builder *UseRequiredTimeOfArrivalGoal(Units::Time rta_goal);
      Builder *TerminateAtFix(std::string rta_fix);
      bool IsWindBlendingEnabled() const { return m_enable_wind_blending; };
      RTAUtils::SpeedLimiterType GetSpeedLimiter() const { return m_speed_command_limiter; };
      std::string GetRtaFix() const { return m_rta_fix; };
      Units::Time GetRtaGoal() const { return m_rta_goal; };
      std::shared_ptr<aaesim::open_source::PredictedWindEvaluator> GetPredictedWindEvaluator() const;
   };

  private:
   static log4cplus::Logger m_logger;
   static void DoLogging(const SimulationTime &simulation_time, const AlgorithmOutputData &output_data);
   static void DoVerticalPathLogging(const SimulationTime &simulation_time, const VerticalPath &vertical_path);
   Units::Speed ComputeMinimumFlapSpeed(
         const aaesim::open_source::bada_utils::FlapConfiguration &current_configuration);
   RtaToac(const Builder &builder);
   std::shared_ptr<aaesim::open_source::SpeedCommandLimiter> BuildSpeedLimiterFromType() const;
   AlgorithmOutputData BuildDefaultAlgorithmOutputDataSet();
   std::pair<bool, const Units::Speed> ComputeDeltaIasSpeed(
         const Units::Length &distance_to_rta_fix, const SimulationTime &simtime, const Units::Speed &current_ias,
         const aaesim::open_source::bada_utils::FlapConfiguration &current_configuration);

   std::shared_ptr<const aaesim::Wgs84KineticDescentPredictor> m_initial_fms_predictor;
   VerticalPath m_active_vertial_guidance;
   bool m_wind_blending_enabled;
   std::shared_ptr<aaesim::open_source::SpeedCommandLimiter> m_speed_command_limiter;
   RTAUtils::SpeedLimiterType m_speed_limit_type;
   aaesim::open_source::OwnshipPerformanceParameters m_performance_parameters;
   aaesim::open_source::OwnshipFmsPredictionParameters m_ownship_prediction_parameters;
   AircraftIntent m_ownship_aircraft_intent_to_rtafix;
   std::string m_rta_fix_name;
   Units::Time m_rta_goal;
   WeatherPrediction m_weather_prediction;
   aaesim::Wgs84AlongPathDistanceCalculator m_distance_calculator;
   Units::Speed m_previous_delta_speed;
   Units::Speed m_previous_ias_command;
   std::vector<std::pair<Units::Time, AlgorithmOutputData>> m_algorithm_output_data;
   int m_speed_command_count;
   std::shared_ptr<aaesim::open_source::WaypointPassingMonitor> m_rta_fix_monitor;
   std::shared_ptr<aaesim::open_source::PredictedWindEvaluator> m_predicted_wind_evaluator;
};
}  // namespace required_time_of_arrival

inline bool required_time_of_arrival::RtaToac::IsBlendWind() const { return m_wind_blending_enabled; }

inline const std::vector<std::pair<Units::Time, required_time_of_arrival::RtaToac::AlgorithmOutputData>>
      &required_time_of_arrival::RtaToac::GetAlgorithmOutputData() const {
   return m_algorithm_output_data;
}

inline Units::Speed required_time_of_arrival::RtaToac::ComputeMinimumFlapSpeed(
      const aaesim::open_source::bada_utils::FlapConfiguration &current_configuration) {
   switch (current_configuration) {
      case aaesim::open_source::bada_utils::FlapConfiguration::TAKEOFF:
         return m_performance_parameters.flap_speeds.cas_takeoff_minimum;
      case aaesim::open_source::bada_utils::FlapConfiguration::INITIAL_CLIMB:
         return m_performance_parameters.flap_speeds.cas_climb_minimum;
      case aaesim::open_source::bada_utils::FlapConfiguration::CRUISE:
         return m_performance_parameters.flap_speeds.cas_cruise_minimum;
      case aaesim::open_source::bada_utils::FlapConfiguration::APPROACH:
         return m_performance_parameters.flap_speeds.cas_approach_minimum;
      case aaesim::open_source::bada_utils::FlapConfiguration::LANDING:
         return m_performance_parameters.flap_speeds.cas_landing_minimum;
      case aaesim::open_source::bada_utils::FlapConfiguration::GEAR_DOWN:
         return m_performance_parameters.flap_speeds.cas_gear_out_minimum;
      default:
         return Units::negInfinity();
   }
}
