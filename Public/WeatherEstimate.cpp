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

#include "public/WeatherEstimate.h"
#include "public/Wind.h"
#include "public/CoreUtils.h"
#include "public/WindZero.h"

using namespace aaesim::open_source;

WeatherEstimate::WeatherEstimate()
   : m_shared_members(std::make_shared<shared_members>()),
     m_wind(nullptr),
     m_temperature_checked(true),
     m_temperature_available(false) {}

WeatherEstimate::WeatherEstimate(std::shared_ptr<Wind> wind, std::shared_ptr<Atmosphere> atmosphere)
   : m_shared_members(std::make_shared<shared_members>(atmosphere)),
     m_wind(wind),
     m_temperature_checked(false),
     m_temperature_available(false) {

   if (CoreUtils::InstanceOf<WindZero>(wind.get())) {
      m_temperature_checked = true;
      m_temperature_available = false;
   }
}

std::shared_ptr<Wind> WeatherEstimate::getWind() const { return m_wind; }

std::shared_ptr<Atmosphere> WeatherEstimate::getAtmosphere() const { return m_shared_members->m_atmosphere; }

WeatherEstimate::~WeatherEstimate() {}

void WeatherEstimate::LoadConditionsAt(const Units::Angle latitude, const Units::Angle longitude,
                                       const Units::Length altitude) {

   SetLocation(latitude, longitude, altitude);
   m_wind->InterpolateTrueWind(latitude, longitude, altitude, east_west(), north_south());

   if (IsTemperatureAvailable(latitude, longitude, altitude)) {
      m_temperature = m_wind->InterpolateTemperature(latitude, longitude, altitude);
      m_pressure = m_wind->InterpolatePressure(latitude, longitude, altitude);
      m_density = m_pressure / (m_temperature * R);
   } else {
      m_temperature = m_shared_members->m_atmosphere->GetTemperature(altitude);
      m_shared_members->m_atmosphere->AirDensity(altitude, m_density, m_pressure);
   }
}

Units::Density WeatherEstimate::GetDensity() const { return m_density; }

Units::Pressure WeatherEstimate::GetPressure() const { return m_pressure; }

Units::KelvinTemperature WeatherEstimate::GetTemperature() const { return m_temperature; }

Units::Speed WeatherEstimate::MachToTAS(const double mach, const Units::Length altitude) const {

   Units::MetersPerSecondSpeed speed_of_sound;

   if (m_temperature_available) {
      // assume conditions have been loaded
      speed_of_sound = getAtmosphere()->SpeedOfSound(m_temperature);
   } else {
      speed_of_sound = getAtmosphere()->SpeedOfSound(altitude);
   }

   Units::Speed true_airspeed = mach * speed_of_sound;
   return true_airspeed;
}

Units::Speed WeatherEstimate::MachToCAS(const double mach, const Units::Length altitude) const {

   Units::Speed true_airspeed = MachToTAS(mach, altitude);
   Units::Speed calibrated_airspeed = TAS2CAS(true_airspeed, altitude);

   return calibrated_airspeed;
}

double WeatherEstimate::ESFconstantCAS(const Units::Speed true_airspeed, const Units::Length altitude) const {

   Units::KelvinTemperature temperature;
   if (!m_temperature_available) {
      /* assume LoadConditionsAt has been called with the current location to set m_temperature */
      temperature = m_temperature;
   } else {
      temperature = getAtmosphere()->GetTemperature(altitude);
   }

   double esf = getAtmosphere()->ESFconstantCAS(true_airspeed, altitude, temperature);
   return esf;
}

bool WeatherEstimate::IsTemperatureAvailable(const Units::Angle latitude, const Units::Angle longitude,
                                             const Units::Length altitude) const {

   if (!m_temperature_checked) {
      /*
       * This check changes values of boolean fields, but we consider it
       * a lazy init and therefore const.  Results may depend on the
       * coordinates used the first time through.
       */

      Units::KelvinTemperature t = m_wind->InterpolateTemperature(latitude, longitude, altitude);
      m_temperature_available = (t.value() >= 0);
      m_temperature_checked = true;
      if (m_temperature_available) {
         LOG4CPLUS_INFO(m_logger, "Temperature is available from Wind object.");
      } else {
         LOG4CPLUS_INFO(m_logger, "Temperature is unavailable from wind object, using Atmosphere");
      }
   }

   return m_temperature_available;
}

Units::Speed WeatherEstimate::TAS2CAS(const Units::Speed true_airspeed, const Units::Length altitude) const {

   Units::Speed calibrated_airspeed;

   if (m_temperature_available) {
      // Assume current conditions have been loaded
      calibrated_airspeed = getAtmosphere()->TAS2CAS(true_airspeed, m_pressure, m_density);
   } else {
      calibrated_airspeed = getAtmosphere()->TAS2CAS(true_airspeed, altitude);
   }

   return calibrated_airspeed;
}

double WeatherEstimate::TAS2Mach(const Units::Speed true_airspeed, const Units::Length altitude) const {

   Units::MetersPerSecondSpeed speed_of_sound;

   if (m_temperature_available) {
      // Assume correct conditions have been loaded
      speed_of_sound = getAtmosphere()->SpeedOfSound(m_temperature);
   } else {
      speed_of_sound = getAtmosphere()->SpeedOfSound(altitude);
   }

   double mach = true_airspeed / speed_of_sound;
   return mach;
}

Units::Speed WeatherEstimate::CAS2TAS(const Units::Speed calibrated_airspeed, const Units::Length altitude) const {

   Units::Speed true_airspeed;

   if (m_temperature_available) {
      // assume conditions have been loaded
      true_airspeed = getAtmosphere()->CAS2TAS(calibrated_airspeed, m_pressure, m_density);
   } else {
      true_airspeed = getAtmosphere()->CAS2TAS(calibrated_airspeed, altitude);
   }

   return true_airspeed;
}

double WeatherEstimate::CAS2Mach(const Units::Speed calibrated_airspeed, const Units::Length altitude) const {
   return TAS2Mach(CAS2TAS(calibrated_airspeed, altitude), altitude);
}
