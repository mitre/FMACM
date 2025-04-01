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

#include "rtaalgs/RtaGoalSolver.h"

#include <memory>

#include "public/FlightDeckApplication.h"
#include "avionics/Wgs84KineticDescentPredictor.h"

namespace required_time_of_arrival {
class RtaGoalIasSolver final : public RtaGoalSolver {

  public:
   RtaGoalIasSolver(Units::Time rta_goal, Units::Time arrival_error_tolerance,
                    std::shared_ptr<aaesim::Wgs84KineticDescentPredictor> fms_descent_predictor,
                    const AircraftIntent &aircraft_intent_to_rta_fix,
                    const aaesim::open_source::OwnshipPerformanceParameters &ownship_performance_parameters,
                    const aaesim::open_source::OwnshipFmsPredictionParameters &ownship_prediction_parameters,
                    const aaesim::open_source::WeatherPrediction &weather_prediction)
      : m_rta_goal(rta_goal),
        m_arrival_error_tolerance(arrival_error_tolerance),
        m_reference_fms_predictor(fms_descent_predictor),
        m_ownship_aircraft_intent_to_rtafix(aircraft_intent_to_rta_fix),
        m_performance_parameters(ownship_performance_parameters),
        m_ownship_prediction_parameters(ownship_prediction_parameters),
        m_weather_prediction(weather_prediction){};
   ~RtaGoalIasSolver() = default;

   EtaMinimizerOutputData Solve(const aaesim::open_source::SimulationTime &sim_time,
                                const aaesim::open_source::AircraftState &aircraftstate,
                                Units::Length distance_to_rta_fix) override;

   bool EvaluateIsAble(const Units::Length &distance_to_rta_fix, const aaesim::open_source::SimulationTime &sim_time,
                       const aaesim::open_source::AircraftState &current_state) override;

  private:
   struct BisectorResult {
      Units::Time positive_bracket{Units::negInfinity()};
      Units::Time negative_bracket{Units::negInfinity()};
      Units::Speed positive_guess{Units::negInfinity()};
      Units::Speed negative_guess{Units::negInfinity()};
      Units::Speed delta_ias{Units::negInfinity()};
      Units::Time eta_error{Units::negInfinity()};
   };
   std::pair<bool, const Units::Speed> ComputeDeltaIas(
         const Units::Length &distance_to_rta_fix, const aaesim::open_source::SimulationTime &sim_time,
         const Units::Speed &current_ias,
         const aaesim::open_source::bada_utils::FlapConfiguration &current_configuration);
   Units::Speed ComputeMinimumFlapSpeed(
         const aaesim::open_source::bada_utils::FlapConfiguration &current_configuration);
   Units::Time ComputeArrivalErrorForDeltaIas(const aaesim::open_source::SimulationTime &sim_time,
                                              const Units::Speed &delta_ias, const Units::Length &distance_to_rta_fix);
   BisectorResult ComputeBrackets(const Units::Length &distance_to_rta_fix,
                                  const aaesim::open_source::SimulationTime &sim_time, const Units::Speed &current_ias,
                                  const aaesim::open_source::bada_utils::FlapConfiguration &current_configuration);
   bool IsAble(const BisectorResult &bisector_result) const;

   Units::Time m_rta_goal{Units::negInfinity()};
   Units::Time m_arrival_error_tolerance{Units::negInfinity()};
   std::shared_ptr<const aaesim::Wgs84KineticDescentPredictor> m_reference_fms_predictor{};
   AircraftIntent m_ownship_aircraft_intent_to_rtafix{};
   aaesim::open_source::OwnshipPerformanceParameters m_performance_parameters{};
   aaesim::open_source::OwnshipFmsPredictionParameters m_ownship_prediction_parameters{};
   aaesim::open_source::WeatherPrediction m_weather_prediction{};
   aaesim::Wgs84AlongPathDistanceCalculator m_distance_calculator{};
};

inline Units::Speed required_time_of_arrival::RtaGoalIasSolver::ComputeMinimumFlapSpeed(
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

}  // namespace required_time_of_arrival