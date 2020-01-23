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
// Copyright 2020 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#pragma once

#include <memory>
#include "public/Atmosphere.h"
#include "public/WindStack.h"
#include "utility/Logging.h"

class Wind;

class WeatherEstimate
{
   friend class Wind_populate_predicted_wind_matrices_Test;

   friend class TrajectoryPredictor_updateWeatherPrediction_Test;

   friend class TrajectoryPredictor_startAltitudeInDescentAltList_Test;

   friend class TrajectoryPredictor_startAndEndAltitudeInDescentAltList_Test;

private:
   static log4cplus::Logger m_logger;

public:
   WindStack east_west;
   WindStack north_south;

   std::shared_ptr<Wind> getWind() const;

   std::shared_ptr<Atmosphere> getAtmosphere() const;

   void LoadConditionsAt(const Units::Angle latitude, const Units::Angle longitude, const Units::Length altitude);
   Units::Density GetDensity() const;
   Units::Pressure GetPressure() const;
   Units::KelvinTemperature GetTemperature() const;

   Units::Speed MachToTAS(const double mach, const Units::Length altitude) const;
   Units::Speed MachToCAS(const double mach, const Units::Length altitude) const;
   Units::Speed TAS2CAS(const Units::Speed true_airspeed, const Units::Length altitude) const;
   Units::Speed CAS2TAS(const Units::Speed calibrated_airspeed, const Units::Length altitude) const;
   double TAS2Mach(const Units::Speed true_airspeed, const Units::Length altitude) const;
   double ESFconstantCAS(const Units::Speed true_airspeed, const Units::Length altitude) const;
   std::pair< std::pair<Units::Angle, Units::Angle>, Units::Length> GetCurrentLocationOfWeather() const;
   void SetAtmosphere(std::shared_ptr<Atmosphere> atmosphere);

protected:
   // Constructors are protected to prevent bare instantiation.
   // Callers should use WeatherPrediction or WeatherTruth.
   WeatherEstimate();

   WeatherEstimate(std::shared_ptr<Wind> wind,
                   std::shared_ptr<Atmosphere> atmosphere);

   virtual ~WeatherEstimate();

   bool IsTemperatureAvailable(const Units::Angle latitude, const Units::Angle longitude, const Units::Length altitude) const;

   void SetLocation(const Units::Angle latitude, const Units::Angle longitude, const Units::Length altitude);

   std::shared_ptr<Wind> m_wind;
   std::shared_ptr<Atmosphere> m_atmosphere;
   Units::KelvinTemperature m_temperature;
   Units::Pressure m_pressure;
   Units::Density m_density;
   mutable bool m_temperature_checked, m_temperature_available;
   std::pair< std::pair<Units::Angle, Units::Angle>, Units::Length> m_location_of_current_conditions;

};

inline void WeatherEstimate::SetAtmosphere(
      std::shared_ptr<Atmosphere> atmosphere) {
   m_atmosphere = atmosphere;
}

inline void WeatherEstimate::SetLocation(const Units::Angle latitude, const Units::Angle longitude, const Units::Length altitude) {
   m_location_of_current_conditions.first = std::pair<Units::Angle, Units::Angle>(latitude, longitude);
   m_location_of_current_conditions.second = altitude;
}

inline std::pair< std::pair<Units::Angle, Units::Angle>, Units::Length> WeatherEstimate::GetCurrentLocationOfWeather() const {
   return m_location_of_current_conditions;
}