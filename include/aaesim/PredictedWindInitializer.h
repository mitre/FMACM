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

#include <filesystem>

#include "public/Atmosphere.h"
#include "public/WeatherPrediction.h"
#include "public/AircraftIntent.h"
#include "public/WeatherPrediction.h"
#include "public/Wind.h"
#include "bada/Bada3Factory.h"
#include "weather/WindFileCache.h"

namespace aaesim {
namespace test {
class Wind_calculate_wind_gradients_Test;
class TrajectoryPredictor_startAltitudeInDescentAltList_Test;
class TrajectoryPredictor_startAndEndAltitudeInDescentAltList_Test;
class Wind_populate_predicted_wind_matrices_Test;
class Wind_single_dtg_min_implementation_Test;
};  // namespace test
class PredictedWindInitializer final {
   friend class aaesim::test::Wind_calculate_wind_gradients_Test;
   friend class aaesim::test::TrajectoryPredictor_startAltitudeInDescentAltList_Test;
   friend class aaesim::test::TrajectoryPredictor_startAndEndAltitudeInDescentAltList_Test;
   friend class aaesim::test::Wind_populate_predicted_wind_matrices_Test;
   friend class aaesim::test::Wind_single_dtg_min_implementation_Test;

  public:
   static aaesim::open_source::WeatherPrediction BuildPredictedWeatherForAvionics(
         const std::filesystem::path &forecast_wind_filename, const AircraftIntent &aircraft_intent,
         const aaesim::open_source::PredictedWindOption &predicted_wind_option,
         const std::vector<Units::Length> &predicted_wind_altitudes, const Units::CelsiusTemperature temperature_offset,
         const Atmosphere::AtmosphereType atmosphere_type);

  private:
   inline static const Units::NauticalMilesLength SAMPLING_DISTANCE_FROM_END_OF_ROUTE{60};
   static inline log4cplus::Logger m_logger{log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("PredictedWindInitializer"))};

   static void CreatePredictionUsingCurrentWindOption(const AircraftIntent &aircraft_intent,
                                                      const Units::FeetLength altitude_at_end_of_route,
                                                      const int maximum_wind_index,
                                                      std::set<Units::Length> &forecast_wind_altitudes,
                                                      int current_wind_index_in,
                                                      aaesim::open_source::WeatherPrediction &weather_prediction);

   static void AddPredictedWindAtPtpToWindStack(const Units::Angle latitude, const Units::Angle longitude,
                                                const Units::FeetLength altitude_at_end_of_route,
                                                std::set<Units::Length> &forecast_wind_altitudes,
                                                aaesim::open_source::WeatherPrediction &weather_prediction,
                                                int &current_wind_index);

   static void PopulatePredictedWindMatrices(const AircraftIntent &intent_in,
                                             const std::vector<Units::Length> &predicted_wind_altitudes_in,
                                             aaesim::open_source::WeatherPrediction &weather_prediction);

   static std::set<Units::Length> ValidateWindAltitudeInputs(const std::vector<Units::Length> &wind_altitudes_in);

   static Units::FeetLength GetAdjustedStartPointAltitude(Units::FeetLength altitude_at_cruise);

   static Units::FeetLength GetAdjustedEndPointAltitude(Units::FeetLength altitude_at_end_of_route);

   static std::set<Units::Length> AddRouteAltitudesToList(const std::set<Units::Length> &wind_altitudes_in,
                                                          Units::FeetLength altitude_at_end_of_route,
                                                          Units::FeetLength altitude_at_cruise);

   static void AddSensedWindsToWindStack(const AircraftIntent::RouteData &route_data,
                                         const Units::FeetLength altitude_at_cruise,
                                         std::set<Units::Length> &forecast_wind_altitudes,
                                         aaesim::open_source::WeatherPrediction &weather_prediction,
                                         int &current_wind_index);

   static void AddIntermediateWindAltitudes(std::set<Units::Length> &wind_altitudes_ft);

};  // PredictedWindInitializer
}  // namespace aaesim
