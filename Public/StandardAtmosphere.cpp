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

// Standard Temperature at Sea Level
// BADA_37_USER_MANUAL eq. 3.2-3
const Units::KelvinTemperature T0_ISA(288.15);

// Temperature of the tropopause
// BADA_37_USER_MANUAL eq. 3.2-4
const Units::KelvinTemperature T_TROP(216.65);

StandardAtmosphere::StandardAtmosphere(const Units::Temperature temperatureOffset) :
      m_temperature_offset(temperatureOffset),
      // BADA_37_USER_MANUAL eq. 3.2-1
      m_tropopause_height(Units::MetersLength(11000) + Units::MetersLength(1000)
            * temperatureOffset / Units::CelsiusTemperature(6.5)),
      m_sea_level_temperature(T0_ISA + m_temperature_offset),  // BADA_37_USER_MANUAL eq. 3.2-2
      m_sea_level_density(RHO0_ISA * T0_ISA / m_sea_level_temperature), // BADA_37_USER_MANUAL eq. 3.2-7
      m_tropopause_density(m_sea_level_density * pow(T_TROP / m_sea_level_temperature,RHO_T_EXPONENT)),
      m_tropopause_pressure(P0_ISA * pow(T_TROP / m_sea_level_temperature, P_T_EXPONENT))
{
}

StandardAtmosphere::~StandardAtmosphere() {
   // nothing to do
}

Units::Temperature StandardAtmosphere::GetTemperatureOffset() const {
   return m_temperature_offset;
}

Units::KelvinTemperature StandardAtmosphere::GetTemperature(const Units::Length altitude_msl) const {
   Units::KelvinTemperature T;
   if (altitude_msl < GetTropopauseHeight()) {
      T = GetSeaLevelTemperature() - Units::KelvinPerMeter(6.5 / 1000) * altitude_msl;
   } else {
      T = T_TROP + m_temperature_offset;
   }

   return T;
}

Units::KelvinTemperature StandardAtmosphere::GetSeaLevelTemperature() const {
   return m_sea_level_temperature;
}


Units::MetersLength StandardAtmosphere::GetTropopauseHeight() const {
   return m_tropopause_height;
}

Units::Density StandardAtmosphere::GetSeaLevelDensity() const {
   return m_sea_level_density;
}

Units::Density StandardAtmosphere::GetTropopauseDensity() const {
   return m_tropopause_density;
}

Units::Pressure StandardAtmosphere::GetTropopausePressure() const {
   return m_tropopause_pressure;
}

