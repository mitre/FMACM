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

#include "public/WindZero.h"

WindZero::WindZero()
      :
      m_standard_atmosphere(Units::CelsiusTemperature(0)) {
}

WindZero::~WindZero() {
}

void WindZero::InterpolateWind(Units::Angle latitude_in,
                               Units::Angle longitude_in,
                               Units::Length alt,
                               Units::Speed &u,
                               Units::Speed &v) {

   u = Units::KnotsSpeed(0);
   v = Units::KnotsSpeed(0);
}

void WindZero::InterpolateWindScalar(Units::Angle lat_in,
                                     Units::Angle lon_in,
                                     Units::Length altitude,
                                     Units::Speed &east_west,
                                     Units::Speed &north_south) {

   east_west = Units::KnotsSpeed(0);
   north_south = Units::KnotsSpeed(0);
}

void WindZero::InterpolateWindMatrix(Units::Angle lat_in,
                                     Units::Angle lon_in,
                                     Units::Length alt_in,
                                     WindStack &east_west,
                                     WindStack &north_south) {

   for (int i = east_west.get_min_row(); i <= east_west.get_max_row(); i++) {
      east_west.set(i, Units::FeetLength((i - 1) * 1000), Units::KnotsSpeed(0));
   }

   for (int i = north_south.get_min_row(); i <= north_south.get_max_row(); i++) {
      north_south.set(i, Units::FeetLength((i - 1) * 1000), Units::KnotsSpeed(0));
   }
}

Units::Temperature WindZero::InterpolateTemperature(Units::Angle latitude_in,
                                                    Units::Angle longitude_in,
                                                    Units::Length alt) {

   // use the standard atmosphere, ignoring lat/lon
   return m_standard_atmosphere.GetTemperature(alt);
}
