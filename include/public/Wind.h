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

class Wind;    // avoid dependency loop

#include <Length.h>
#include <Speed.h>
#include <Angle.h>
#include <Temperature.h>
#include <string>
#include "public/WeatherPrediction.h"
#include "public/AircraftIntent.h"
#include "public/AircraftState.h"

#define FLIGHT_LEVEL_UPPER_BOUND 45 // FIXME this is not flight level! should be 450 or renamed
#define FLIGHT_LEVEL_LOWER_BOUND 0
#define DIST_FROM_FAF_NM 60 // Distance from beginning point to select waypoint from which to winds at descent altitudes.

// needed for WindSpeedUtils friend class
namespace aaesim {
   namespace test {
      namespace utils {
         class WindSpeedUtils;
      }
   }
}

// File wind data formats implemented.

enum WindDataFormat
{
   /** 46 x 40 x 70 CSV file from MATLAB */
         RUC_FORMAT,
   /** 50 x 337 x 451 CSV file from MATLAB */
         RAP_FORMAT,
   /** Binary file to be detected by CAASD Wind API */
         BINARY
};


// Wind files used in processing.

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

   friend class aaesim::test::utils::WindSpeedUtils;//::populateWindMatrices(const Units::Speed windSpeedEast, const Units::Speed windSpeedNorth);

private:
   static log4cplus::Logger m_logger;
   static Units::Length m_blending_altitude_limit;
   static std::shared_ptr<Wind> m_wind_truth_instance;
protected:
   static bool m_use_wind;

public:

   Wind();

   virtual ~Wind();

   static void Initialize();

   /**
    * Updates the forecast wind matrix in the altitude domain. The blending
    * algorithm is provided by Lesley. It will smoothly blend wind velocities from sensed to forecast
    * up to windBlendingAltitudeLimit above and below the current aircraft's altitude
    * and across all lat/longs.
    *
    * @param current_state the current navigation state of the aircraft
    * @param &wind_x the predicted winds in the x
    * @param &wind_y the predicted winds in the y
    *
    * @see Wind::PopulatePredictedWindMatrices
    */
   static void UpdatePredictedWindsAtAltitudeFromSensedWind(const AircraftState &current_state, WeatherPrediction &weather_prediction);

   void PopulatePredictedWindMatrices(const AircraftIntent &intent,
                                      const std::vector<Units::Length> &predicted_wind_altitudes,
                                      WeatherPrediction &weather_prediction);

   static void ValidatePredictedOptOne(const AircraftIntent &intent,
                                       PredictedWindOption &useOpt,
                                       double &altCoef,
                                       Units::Length &distConst);

   void InterpolateTrueWind(const std::shared_ptr<TangentPlaneSequence> &tangentPlaneSequence,
                                   const Units::Length x_in,
                                   const Units::Length y_in,
                                   const Units::Length altitude,
                                   WindStack &east_west,
                                   WindStack &north_south);

   void InterpolateForecastWind(const std::shared_ptr<TangentPlaneSequence> &tangentPlaneSequence,
                                const Units::Length x_in,
                                const Units::Length y_in,
                                const Units::Length altitude,
                                Units::Speed &east_west,
                                Units::Speed &north_south);

   virtual Units::Temperature InterpolateTemperature(const Units::Angle latitude_in,
                                                     const Units::Angle longitude_in,
                                                     const Units::Length altitude) = 0;

   static void CheckWindIntegrity(const WindStack &wind);

   static void CheckWindIntegrity(const int id,
                                  const std::string &str,
                                  const WindStack &wind);

   static WeatherPrediction CreateZeroWindPrediction();

   static std::shared_ptr<Wind> GetWindTruthInstance();
   static void SetWindTruthInstance(std::shared_ptr<Wind> truth_instance);
   static void SetUseWind(const bool useWind);
   static bool IsUseWind();

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

};

inline std::shared_ptr<Wind> Wind::GetWindTruthInstance() {
   return m_wind_truth_instance;
}
