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

#include "public/WindZero.h"

WindZero::WindZero() : m_standard_atmosphere(Units::CelsiusTemperature(0)) {}

WindZero::~WindZero() {}

void WindZero::InterpolateWind(Units::Angle latitude_in, Units::Angle longitude_in, Units::Length alt, Units::Speed &u,
                               Units::Speed &v) {

   u = Units::KnotsSpeed(0);
   v = Units::KnotsSpeed(0);
}

void WindZero::InterpolateWindScalar(Units::Angle lat_in, Units::Angle lon_in, Units::Length altitude,
                                     Units::Speed &east_west, Units::Speed &north_south) {

   east_west = Units::KnotsSpeed(0);
   north_south = Units::KnotsSpeed(0);
}

void WindZero::InterpolateWindMatrix(Units::Angle lat_in, Units::Angle lon_in, Units::Length alt_in,
                                     WindStack &east_west, WindStack &north_south) {

   for (int i = east_west.GetMinRow(); i <= east_west.GetMaxRow(); i++) {
      east_west.Set(i, Units::FeetLength((i - 1) * 1000), Units::KnotsSpeed(0));
   }

   for (int i = north_south.GetMinRow(); i <= north_south.GetMaxRow(); i++) {
      north_south.Set(i, Units::FeetLength((i - 1) * 1000), Units::KnotsSpeed(0));
   }
}

Units::KelvinTemperature WindZero::InterpolateTemperature(Units::Angle latitude_in, Units::Angle longitude_in,
                                                          Units::Length alt) {

   // use the standard atmosphere, ignoring lat/lon
   return m_standard_atmosphere.GetTemperature(alt);
}

Units::Pressure WindZero::InterpolatePressure(Units::Angle latitude_in, Units::Angle longitude_in, Units::Length alt) {
   Units::Pressure pressure;
   Units::Density density;
   m_standard_atmosphere.AirDensity(alt, density, pressure);
   return pressure;
}
