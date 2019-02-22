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

#include "public/StandardAtmosphere.h"

StandardAtmosphere::StandardAtmosphere(const Units::Temperature temperatureOffset)
      :
      m_temperature_offset(temperatureOffset) {
}

StandardAtmosphere::~StandardAtmosphere() {
   // nothing to do
}

Units::Temperature StandardAtmosphere::GetTemperatureOffset() const {
   return m_temperature_offset;
}

Units::KelvinTemperature StandardAtmosphere::GetTemperature(const Units::Length altitude_msl) const {
   Units::KelvinTemperature T;
   if (altitude_msl < H_TROP) {
      T = T0 + m_temperature_offset - Units::KelvinPerMeter(6.5 / 1000) * altitude_msl;
   } else {
      T = T_TROP + m_temperature_offset;
   }

   return T;
}
