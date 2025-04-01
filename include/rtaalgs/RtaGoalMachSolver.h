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

#include "utility/BoundedValue.h"
#include "public/FlightDeckApplication.h"
#include "avionics/Wgs84KineticDescentPredictor.h"

namespace required_time_of_arrival {
class RtaGoalMachSolver final : public RtaGoalSolver {

  public:
   inline static constexpr double MINIMUM_ALLOWABLE_MACH{0.6};
   RtaGoalMachSolver(Units::Time rta_goal, Units::Time arrival_error_tolerance,
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
   ~RtaGoalMachSolver() = default;

   EtaMinimizerOutputData Solve(const aaesim::open_source::SimulationTime &sim_time,
                                const aaesim::open_source::AircraftState &aircraft_state,
                                Units::Length distance_to_rta_fix) override;

   bool EvaluateIsAble(const Units::Length &distance_to_rta_fix, const aaesim::open_source::SimulationTime &sim_time,
                       const aaesim::open_source::AircraftState &current_state) override;

  private:
   struct BisectorResult {
      Units::Time positive_bracket{Units::negInfinity()};
      Units::Time negative_bracket{Units::negInfinity()};
      BoundedValue<double, -1, 1> positive_guess{0};
      BoundedValue<double, -1, 1> negative_guess{0};
      BoundedValue<double, -1, 1> delta_mach{0};
      Units::Time eta_error{Units::negInfinity()};
   };
   std::pair<bool, const BoundedValue<double, -1, 1>> ComputeDeltaMach(
         const Units::Length &distance_to_rta_fix, const aaesim::open_source::SimulationTime &sim_time,
         const BoundedValue<double, 0, 1> &current_mach);
   Units::Time ComputeArrivalErrorForDeltaMach(const aaesim::open_source::SimulationTime &sim_time,
                                               const BoundedValue<double, -1, 1> &delta_mach,
                                               const Units::Length &distance_to_rta_fix);
   BisectorResult ComputeBrackets(const Units::Length &distance_to_rta_fix,
                                  const aaesim::open_source::SimulationTime &sim_time,
                                  const BoundedValue<double, 0, 1> &current_mach);
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

}  // namespace required_time_of_arrival