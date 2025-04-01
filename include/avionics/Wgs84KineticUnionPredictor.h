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

#include "avionics/Wgs84KineticClimbPredictor.h"
#include "avionics/Wgs84KineticDescentPredictor.h"
#include "public/WeatherPrediction.h"
#include "public/AircraftIntent.h"
#include "public/Guidance.h"
#include "public/PredictedWindEvaluator.h"
#include "public/WindBlendingAlgorithm.h"
#include "public/BlendWindsVerticallyByAltitude.h"

namespace aaesim {
class Wgs84KineticUnionPredictor final {
  public:
   Wgs84KineticUnionPredictor(aaesim::Wgs84KineticClimbPredictor &kinetic_climb_predictor,
                              aaesim::Wgs84KineticDescentPredictor &kinetic_trajectory_predictor,
                              std::shared_ptr<aaesim::open_source::PredictedWindEvaluator> predicted_wind_evaluator)
      : m_kinetic_climb_predictor(kinetic_climb_predictor),
        m_kinetic_descent_predictor(kinetic_trajectory_predictor),
        m_descent_predicted_weather(),
        m_ascent_predicted_weather(),
        m_predicted_wind_evaluator(predicted_wind_evaluator),
        m_estimated_wgs84_route_length(Units::negInfinity()),
        m_climb_phase_exists(false),
        m_cruise_descent_phase_exists(false){};
   ~Wgs84KineticUnionPredictor() = default;

   void BuildInitialTrajectoryPredictions(const std::string &ac_performance_name, const AircraftIntent &aircraft_intent,
                                          const aaesim::open_source::WeatherPrediction &weather_prediction);
   void UpdateTrajectoryPrediction(const aaesim::open_source::AircraftState &aircraft_state,
                                   const aaesim::open_source::Guidance &previous_guidance,
                                   aaesim::open_source::GuidanceFlightPhase active_phase);

   std::vector<aaesim::Wgs84PrecalcWaypoint> GetOrderedPrecalcWaypoints() const;
   const aaesim::Wgs84KineticClimbPredictor &GetClimbPredictor() const;
   const aaesim::Wgs84KineticDescentPredictor &GetDescentPredictor() const;
   const aaesim::open_source::WeatherPrediction &GetClimbWeatherPrediction() const;
   const aaesim::open_source::WeatherPrediction &GetDescentWeatherPrediction() const;
   bool HasClimb() const { return m_climb_phase_exists; }
   bool HasCruiseDescent() const { return m_cruise_descent_phase_exists; }

  private:
   aaesim::Wgs84KineticClimbPredictor m_kinetic_climb_predictor;
   aaesim::Wgs84KineticDescentPredictor m_kinetic_descent_predictor;
   aaesim::open_source::WeatherPrediction m_descent_predicted_weather;
   aaesim::open_source::WeatherPrediction m_ascent_predicted_weather;
   std::shared_ptr<aaesim::open_source::PredictedWindEvaluator> m_predicted_wind_evaluator;
   std::shared_ptr<aaesim::open_source::WindBlendingAlgorithm> m_wind_blender{
         std::make_shared<aaesim::open_source::BlendWindsVerticallyByAltitude>()};
   Units::Length m_estimated_wgs84_route_length;
   bool m_climb_phase_exists;
   bool m_cruise_descent_phase_exists;
};

}  // namespace aaesim

inline const aaesim::Wgs84KineticClimbPredictor &aaesim::Wgs84KineticUnionPredictor::GetClimbPredictor() const {
   return m_kinetic_climb_predictor;
}

inline const aaesim::Wgs84KineticDescentPredictor &aaesim::Wgs84KineticUnionPredictor::GetDescentPredictor() const {
   return m_kinetic_descent_predictor;
}

inline const aaesim::open_source::WeatherPrediction &aaesim::Wgs84KineticUnionPredictor::GetClimbWeatherPrediction()
      const {
   return m_ascent_predicted_weather;
}

inline const aaesim::open_source::WeatherPrediction &aaesim::Wgs84KineticUnionPredictor::GetDescentWeatherPrediction()
      const {
   return m_descent_predicted_weather;
}
