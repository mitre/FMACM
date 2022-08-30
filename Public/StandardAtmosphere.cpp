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

#include "public/StandardAtmosphere.h"
#include "utility/Logging.h"

log4cplus::Logger StandardAtmosphere::m_logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("StandardAtmosphere"));

// Standard Temperature at Sea Level
// BADA_37_USER_MANUAL eq. 3.2-3
const Units::KelvinTemperature T0_ISA(288.15);

// Temperature of the tropopause
// BADA_37_USER_MANUAL eq. 3.2-4
const Units::KelvinTemperature T_TROP(216.65);

const Units::KelvinPerMeter TEMPERATURE_GRADIENT_TROPOSPHERE(-.0065);

StandardAtmosphere* StandardAtmosphere::MakeInstance(
      const Units::KelvinTemperature temperature,
      const Units::Length altitude) {
   if (temperature < T_TROP) {
      LOG4CPLUS_DEBUG(m_logger, "Creating non-standard StandardAtmosphere with temperature " << temperature
            << " at " << Units::FeetLength(altitude) << ".  Tropopause temperature is " << T_TROP);
   }
   if (altitude < Units::zero()) {
      LOG4CPLUS_FATAL(m_logger, "Specified altitude is below sea level:  " << Units::FeetLength(altitude));
      throw std::runtime_error("Invalid altitude");
   }
   if (temperature == T_TROP) {
      LOG4CPLUS_WARN(m_logger, "StandardAtmosphere created using tropopause temperature " << T_TROP
            << " at " << Units::FeetLength(altitude) << ".  Assuming that represents the floor of the tropopause.");
   }

   // sea_level_temperature = temperature + 6.5/1000 * altitude
   Units::KelvinTemperature sea_level_temperature = temperature - TEMPERATURE_GRADIENT_TROPOSPHERE * altitude;
   Units::KelvinTemperature offset(sea_level_temperature - T0_ISA);
   LOG4CPLUS_TRACE(m_logger, "For StandardAtmosphere with " << temperature << " at " << Units::FeetLength(altitude) << ", offset is " << offset);
   return new StandardAtmosphere(offset);
}

StandardAtmosphere* StandardAtmosphere::MakeInstanceFromTemperatureOffset(Units::CelsiusTemperature temperature_offset) {
   return new StandardAtmosphere(temperature_offset);
}

StandardAtmosphere::StandardAtmosphere(const Units::Temperature temperatureOffset) {
   SetTemperatureOffset(temperatureOffset);
}

void StandardAtmosphere::SetTemperatureOffset(const Units::Temperature temperatureOffset) {
   m_temperature_offset = temperatureOffset;
   // BADA_37_USER_MANUAL eq. 3.2-1
   m_tropopause_height = Units::MetersLength(11000) + Units::MetersLength(1000)
            * m_temperature_offset / Units::CelsiusTemperature(6.5);
   m_sea_level_temperature = T0_ISA + m_temperature_offset;  // BADA_37_USER_MANUAL eq. 3.2-2
   m_sea_level_density = RHO0_ISA * T0_ISA / m_sea_level_temperature; // BADA_37_USER_MANUAL eq. 3.2-7
   m_tropopause_density = m_sea_level_density * pow(T_TROP / m_sea_level_temperature, RHO_T_EXPONENT);
   m_tropopause_pressure = P0_ISA * pow(T_TROP / m_sea_level_temperature, P_T_EXPONENT);
   LOG4CPLUS_TRACE(m_logger, "Temperature offset set to " << Units::KelvinTemperature(temperatureOffset));
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
      T = T_TROP;
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

