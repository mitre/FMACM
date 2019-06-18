// ****************************************************************************
// NOTICE
//
// This is the copyright work of The MITRE Corporation, and was produced
// for the U. S. Government under Contract Number DTFAWA-10-C-00080, and
// is subject to Federal Aviation Administration Acquisition Management
// System Clause 3.5-13, Rights In Data-General, Alt. III and Alt. IV
// (Oct. 1996).  No other use other than that granted to the U. S.
// Government, or to those acting on behalf of the U. S. Government,
// under that Clause is authorized without the express written
// permission of The MITRE Corporation. For further information, please
// contact The MITRE Corporation, Contracts Office, 7515 Colshire Drive,
// McLean, VA  22102-7539, (703) 983-6000.
//
// Copyright 2019 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#pragma once

#include <Length.h>
#include <Speed.h>
#include <Angle.h>
#include <Temperature.h>
#include <string>
#include "public/WeatherPrediction.h"
#include "public/AircraftIntent.h"
#include "public/AircraftState.h"

class Wind;
//class WeatherPrediction;
//enum PredictedWindOption;

// needed for WindSpeedUtils friend class
namespace aaesim {
   namespace test {
      namespace utils {
         class WindSpeedUtils;
      }
   }
}

enum WindDataFormat
{
   /** 46 x 40 x 70 CSV file from MATLAB */
         RUC_FORMAT,
   /** 50 x 337 x 451 CSV file from MATLAB */
         RAP_FORMAT,
   /** Binary file to be detected by CAASD Wind API */
         BINARY
};

enum WindFileType
{
   FORECAST_FILE,
   TRUTH_FILE
};

class Wind
{
   friend class WindLegacy_readRAPWindFile_Test;

   friend class Wind_interpolateTemp_Test;

   friend class Wind_interpolate_wind_Test;

   friend class Wind_interpolate_wind_scalar_Test;

   friend class Wind_interpolate_wind_matrix_Test;

   friend class WindLegacy_check_box_Test;

   friend class Wind_vertically_interpolate_wind_Test;

   friend class Wind_readRAPTestDataWindFile_Test;

   friend class aaesim::test::utils::WindSpeedUtils;

public:
   static const Units::NauticalMilesLength SAMPLING_DISTANCE_FROM_END_OF_ROUTE;
   static const Units::FeetLength MAXIMUM_ALTITUDE_LIMIT;
   static const Units::FeetLength MINIMUM_ALTITUDE_LIMIT;

   Wind();

   virtual ~Wind();

   /**
    * Updates the forecast wind matrix in the altitude domain. The blending
    * algorithm is provided by Lesley. It will smoothly blend wind velocities from sensed to forecast
    * up to windBlendingAltitudeLimit above and below the current aircraft's altitude
    * and across all lat/longs.
    */
   static void UpdatePredictedWindsAtAltitudeFromSensedWind(const AircraftState &current_state,
                                                            WeatherPrediction &weather_prediction);

   void PopulatePredictedWindMatrices(const AircraftIntent &intent_in,
                                      const std::vector<Units::Length> &predicted_wind_altitudes_in,
                                      WeatherPrediction &weather_prediction);

   static void ValidatePredictedOptOne(const AircraftIntent &aircraft_intent,
                                       PredictedWindOption &predicted_wind_option,
                                       double &altitude_coefficient,
                                       Units::Length &distance_constant);

   void InterpolateTrueWind(const Units::Angle lat_in,
                            const Units::Angle lon_in,
                            const Units::Length altitude,
                            WindStack &east_west,
                            WindStack &north_south);

   void InterpolateForecastWind(const std::shared_ptr<TangentPlaneSequence> &tangentPlaneSequence,
                                const Units::Length x_in,
                                const Units::Length y_in,
                                const Units::Length altitude,
                                Units::Speed &east_west,
                                Units::Speed &north_south);

   virtual Units::KelvinTemperature InterpolateTemperature(
         const Units::Angle latitude_in,
         const Units::Angle longitude_in,
         const Units::Length altitude) = 0;

   virtual Units::Pressure InterpolatePressure(
         const Units::Angle latitude_in,
         const Units::Angle longitude_in,
         const Units::Length altitude) = 0;

   static WeatherPrediction CreateZeroWindPrediction();

   static std::shared_ptr<Wind> GetWindTruthInstance();

   static void SetWindTruthInstance(std::shared_ptr<Wind> &truth_instance);

   static void SetUseWind(const bool useWind);

   static bool UseWind();

protected:

   virtual void InterpolateWind(Units::Angle latitude_in,
                                Units::Angle longitude_in,
                                Units::Length altitude,
                                Units::Speed &u,
                                Units::Speed &v) = 0;

   virtual void InterpolateWindScalar(Units::Angle lat_in,
                                      Units::Angle lon_in,
                                      Units::Length altitude,
                                      Units::Speed &east_west,
                                      Units::Speed &north_south) = 0;

   virtual void InterpolateWindMatrix(Units::Angle lat_in,
                                      Units::Angle lon_in,
                                      Units::Length alt_in,
                                      WindStack &east_west,
                                      WindStack &north_south) = 0;

private:

   static bool m_use_wind;

   void CreatePredictionUsingCurrentWindOption(const AircraftIntent &aircraft_intent,
                                               const Units::FeetLength altitude_at_end_of_route,
                                               const int maximum_wind_index,
                                               std::set<Units::Length> &forecast_wind_altitudes,
                                               int current_wind_index_in,
                                               WeatherPrediction &weather_prediction);

   void CreatePredictionUsingLegacyWindOption(PredictedWindOption predicted_wind_option_in,
                                              const std::set<Units::Length> &wind_altitudes,
                                              const AircraftIntent &aircraft_intent,
                                              const Units::Length altitude_at_beginning_of_route,
                                              const int current_wind_index_in,
                                              const int maximum_wind_index,
                                              WeatherPrediction &weather_prediction);

   void AddSensedWindsToWindStack(const std::shared_ptr<TangentPlaneSequence> &tangent_plane_sequence,
                                  const AircraftIntent::RouteData &fms,
                                  const Units::FeetLength altitude_at_beginning_of_route,
                                  std::set<Units::Length> &forecast_wind_altitudes,
                                  WeatherPrediction &weather_prediction,
                                  int &current_wind_index);

   void AddPredictedWindAtPtpToWindStack(const std::shared_ptr<TangentPlaneSequence> &tangent_plane_sequence,
                                         const Units::FeetLength x_position,
                                         const Units::FeetLength y_position,
                                         const Units::FeetLength altitude_at_end_of_route,
                                         std::set<Units::Length> &forecast_wind_altitudes,
                                         WeatherPrediction &weather_prediction,
                                         int &current_wind_index);

   Units::FeetLength GetAdjustedEndPointAltitude(Units::FeetLength altitude_at_end_of_route);

   Units::FeetLength GetAdjustedStartPointAltitude(Units::FeetLength altitude_at_beginning_of_route);

   std::set<Units::Length> AddRouteAltitudesToList(const std::set<Units::Length> &wind_altitudes_in,
                                                   Units::FeetLength altitude_at_end_of_route,
                                                   Units::FeetLength altitude_at_beginning_of_route);

   std::set<Units::Length> ValidateWindAltitudeInputs(const std::vector<Units::Length> &wind_altitudes_in);

   void AddIntermediateWindAltitudes(std::set<Units::Length> &wind_altitudes_ft);

   static log4cplus::Logger m_logger;
   static Units::Length m_blending_altitude_limit;
   static std::shared_ptr<Wind> m_wind_truth_instance;
};

inline void Wind::SetWindTruthInstance(std::shared_ptr<Wind> &truth_instance) {
   m_wind_truth_instance = truth_instance;
}

inline std::shared_ptr<Wind> Wind::GetWindTruthInstance() {
   return m_wind_truth_instance;
}

inline void Wind::SetUseWind(const bool useWind) {
   m_use_wind = useWind;
}

inline bool Wind::UseWind() {
   return m_use_wind;
}
