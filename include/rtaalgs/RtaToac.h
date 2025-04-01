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

#include <memory>
#include <optional>

#include "public/SpeedCommandLimiter.h"
#include "public/PredictedWindEvaluator.h"
#include "rtaalgs/RTAUtils.h"
#include "avionics/Wgs84KineticDescentPredictor.h"
#include "avionics/Wgs84AlongPathDistanceCalculator.h"
#include "avionics/Wgs84HorizontalPathSegment.h"
#include "avionics/FmsIntegratedApplication.h"
#include "public/WaypointPassingMonitor.h"
#include "rtaalgs/RtaGoalSolver.h"
namespace required_time_of_arrival {
class RtaToac final : public aaesim::avionics::FmsIntegratedApplication {
  public:
   struct AlgorithmOutputData {
      Units::Speed nominal_profile_ias;
      Units::Speed dynamics_ias;
      Units::Length distance_to_rta_fix;
      Units::Time eta_to_rta_fix;
      Units::Time eta_error;
      aaesim::LatitudeLongitudePoint fms_position;
      int speed_command_count;
      std::optional<VerticalPath> delta_speed_prediction;
      SpeedValueType selected_speed_type;
      EtaMinimizerOutputData eta_minimizer_output;
      aaesim::open_source::Guidance guidance_to_fms;
   };

   RtaToac() = default;
   virtual ~RtaToac() = default;
   void Initialize(aaesim::avionics::FmsApplicationInitializer &initializer_visitor) override;
   aaesim::open_source::Guidance Update(const aaesim::open_source::SimulationTime &sim_time,
                                        const aaesim::open_source::Guidance &prev_guidance,
                                        const aaesim::open_source::AircraftState &aircraft_state) override;
   void InitializeAlgorithm(std::shared_ptr<aaesim::Wgs84KineticDescentPredictor> fms_descent_predictor,
                            const aaesim::open_source::OwnshipPerformanceParameters &ownship_performance_parameters,
                            const aaesim::open_source::OwnshipFmsPredictionParameters &ownship_prediction_parameters);
   const std::vector<std::pair<Units::Time, AlgorithmOutputData>> &GetAlgorithmOutputData() const;

   class Builder {
     private:
      RTAUtils::SpeedLimiterType m_speed_command_limiter;
      Units::Time m_rta_goal;
      std::string m_rta_fix;
      Units::Time m_arrival_error_tolerance;
      Units::Time m_eta_error_tolerance;

     public:
      Builder();
      ~Builder() = default;
      std::unique_ptr<required_time_of_arrival::RtaToac> Build() const;
      Builder *WithSpeedCommandLimiter(RTAUtils::SpeedLimiterType speed_limiter_type);
      Builder *UseRequiredTimeOfArrivalGoal(Units::Time rta_goal);
      Builder *TerminateAtFix(const std::string &rta_fix);
      Builder *UseArrivalErrorTolerance(Units::Time arrival_error_tolerance);
      Builder *UseEtaErrorTolerance(Units::Time eta_error_tolerance);

      RTAUtils::SpeedLimiterType GetSpeedLimiter() const { return m_speed_command_limiter; };
      std::string GetRtaFix() const { return m_rta_fix; };
      Units::Time GetRtaGoal() const { return m_rta_goal; };
      Units::Time GetArrivalErrorTolerance() const { return m_arrival_error_tolerance; };
      Units::Time GetEtaErrorTolerance() const { return m_eta_error_tolerance; };
   };
   RtaToac(const Builder &builder);

  private:
   static log4cplus::Logger m_logger;
   static void DoLogging(const aaesim::open_source::SimulationTime &simulation_time,
                         const AlgorithmOutputData &output_data);
   static void DoVerticalPathLogging(const aaesim::open_source::SimulationTime &simulation_time,
                                     const VerticalPath &vertical_path);
   std::shared_ptr<aaesim::open_source::SpeedCommandLimiter> BuildSpeedLimiterFromType() const;
   AlgorithmOutputData BuildDefaultAlgorithmOutputDataSet();

   std::shared_ptr<aaesim::Wgs84KineticDescentPredictor> m_reference_fms_predictor{};
   VerticalPath m_active_vertical_guidance{};
   std::shared_ptr<aaesim::open_source::SpeedCommandLimiter> m_speed_command_limiter{};
   RTAUtils::SpeedLimiterType m_speed_limit_type{RTAUtils::SpeedLimiterType::NONE};
   aaesim::open_source::OwnshipPerformanceParameters m_performance_parameters{};
   aaesim::open_source::OwnshipFmsPredictionParameters m_ownship_prediction_parameters{};
   AircraftIntent m_ownship_aircraft_intent_to_rtafix{};
   std::string m_rta_fix_name{};
   Units::Time m_rta_goal{Units::negInfinity()};
   Units::Time m_arrival_error_tolerance{Units::negInfinity()};
   Units::Time m_eta_error_tolerance{Units::negInfinity()};
   aaesim::open_source::WeatherPrediction m_weather_prediction{};
   aaesim::Wgs84AlongPathDistanceCalculator m_distance_calculator{};
   Units::Speed m_previous_ias_command{Units::negInfinity()};
   double m_previous_mach_command{-INFINITY};
   Units::Length m_previous_distance_to_rta_fix{Units::infinity()};
   std::vector<std::pair<Units::Time, AlgorithmOutputData>> m_algorithm_output_data{};
   int m_speed_command_count{0};
   std::shared_ptr<aaesim::open_source::WaypointPassingMonitor> m_rta_fix_monitor{};
};
}  // namespace required_time_of_arrival

inline const std::vector<std::pair<Units::Time, required_time_of_arrival::RtaToac::AlgorithmOutputData>> &
      required_time_of_arrival::RtaToac::GetAlgorithmOutputData() const {
   return m_algorithm_output_data;
}
