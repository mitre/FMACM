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

#include "public/CalcWindGradControl.h"
#include "public/Environment.h"

CalcWindGradControl::CalcWindGradControl()
   : m_wind_x(),
     m_wind_y(),
     m_altitude(Units::MetersLength(-999.0)),
     m_wind_speed_x(),
     m_wind_speed_y(),
     m_wind_gradient_x(),
     m_wind_gradient_y() {}

CalcWindGradControl::~CalcWindGradControl() = default;

void CalcWindGradControl::ComputeWindGradients(const Units::Length &msl_altitude,
                                               const WeatherPrediction &weather_prediction, Units::Speed &wind_speed_x,
                                               Units::Speed &wind_speed_y, Units::Frequency &wind_gradient_x,
                                               Units::Frequency &wind_gradient_y) {

   bool computex = ((msl_altitude != m_altitude) || (weather_prediction.east_west != m_wind_x));
   bool computey = ((msl_altitude != m_altitude) || (weather_prediction.north_south != m_wind_y));

   if (computex) {
      weather_prediction.GetForecastAtmosphere()->CalculateWindGradientAtAltitude(
            msl_altitude, weather_prediction.east_west, m_wind_speed_x, m_wind_gradient_x);
   }

   if (computey) {
      weather_prediction.GetForecastAtmosphere()->CalculateWindGradientAtAltitude(
            msl_altitude, weather_prediction.north_south, m_wind_speed_y, m_wind_gradient_y);
   }

   if (msl_altitude != m_altitude) {
      m_altitude = msl_altitude;
   }

   if (weather_prediction.east_west != m_wind_x) {
      m_wind_x = weather_prediction.east_west;
   }

   if (weather_prediction.north_south != m_wind_y) {
      m_wind_y = weather_prediction.north_south;
   }

   wind_speed_x = m_wind_speed_x;
   wind_speed_y = m_wind_speed_y;

   wind_gradient_x = m_wind_gradient_x;
   wind_gradient_y = m_wind_gradient_y;
}
