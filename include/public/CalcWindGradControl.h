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

#include <string>

#include <scalar/Length.h>
#include <scalar/Speed.h>
#include <scalar/Frequency.h>

#include "public/WeatherPrediction.h"

namespace aaesim {
namespace open_source {
class CalcWindGradControl {

  public:
   CalcWindGradControl();

   virtual ~CalcWindGradControl();

   /**
    * Returns xy wind direction and gradient.  To cut down unnecessary calls
    * to Atmosphere::calcWindGrad, the stored previous calculation is checked
    * for match.  If inputs match, stored outputs returned.  Else
    * Atmosphere::calcWindGrad is called to get new wind direction and gradient
    * and the new calculation values are stored.
    *
    * @param msl_altitude
    * @param weather_prediction
    * @param wind_speed_x output
    * @param wind_speed_y  output
    * @param wind_gradient_x output
    * @param wind_gradient_y output
    */
   void ComputeWindGradients(const Units::Length &msl_altitude, const WeatherPrediction &weather_prediction,
                             Units::Speed &wind_speed_x, Units::Speed &wind_speed_y, Units::Frequency &wind_gradient_x,
                             Units::Frequency &wind_gradient_y);

  private:
   aaesim::open_source::WindStack m_wind_x;
   aaesim::open_source::WindStack m_wind_y;

   Units::Length m_altitude;
   Units::Speed m_wind_speed_x;
   Units::Speed m_wind_speed_y;
   Units::Frequency m_wind_gradient_x;
   Units::Frequency m_wind_gradient_y;
};
}  // namespace open_source
}  // namespace aaesim
