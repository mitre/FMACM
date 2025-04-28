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

#include <scalar/Length.h>
#include <scalar/Speed.h>
#include <scalar/Angle.h>
#include <scalar/Temperature.h>
#include "public/WeatherPrediction.h"
#include "public/AircraftIntent.h"

class Wind {
  public:
   Wind() = default;
   virtual ~Wind() = default;

   void InterpolateTrueWind(const Units::Angle lat_in, const Units::Angle lon_in, const Units::Length altitude,
                            aaesim::open_source::WindStack &east_west, aaesim::open_source::WindStack &north_south);

   void InterpolateForecastWind(const std::shared_ptr<TangentPlaneSequence> &tangentPlaneSequence,
                                const Units::Length x_in, const Units::Length y_in, const Units::Length altitude,
                                Units::Speed &east_west, Units::Speed &north_south);

   virtual void InterpolateWindScalar(Units::Angle lat_in, Units::Angle lon_in, Units::Length altitude,
                                      Units::Speed &east_west, Units::Speed &north_south) = 0;

   virtual Units::KelvinTemperature InterpolateTemperature(const Units::Angle latitude_in,
                                                           const Units::Angle longitude_in,
                                                           const Units::Length altitude) = 0;

   virtual Units::Pressure InterpolatePressure(const Units::Angle latitude_in, const Units::Angle longitude_in,
                                               const Units::Length altitude) = 0;

   virtual void InterpolateWind(Units::Angle latitude_in, Units::Angle longitude_in, Units::Length altitude,
                                Units::Speed &u, Units::Speed &v) = 0;

   virtual void InterpolateWindMatrix(Units::Angle lat_in, Units::Angle lon_in, Units::Length alt_in,
                                      aaesim::open_source::WindStack &east_west,
                                      aaesim::open_source::WindStack &north_south) = 0;

  private:
   inline static log4cplus::Logger m_logger{log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("Wind"))};
};
