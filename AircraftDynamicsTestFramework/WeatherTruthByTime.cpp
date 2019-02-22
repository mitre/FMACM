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

/*
 * WeatherTruthByTime.cpp
 *
 *  Created on: Jan 8, 2019
 */

#include "framework/WeatherTruthByTime.h"
#include <string.h>

using namespace std;

WeatherTruthByTime::WeatherTruthByTime() {
   m_shared_ptr.reset(this);
   m_wind = static_pointer_cast<Wind>(m_shared_ptr);
   m_atmosphere = static_pointer_cast<Atmosphere>(m_shared_ptr);
}

WeatherTruthByTime::~WeatherTruthByTime() {
   // TODO free the Weather instances
}

void WeatherTruthByTime::SetWeatherFromTime(Units::Time time) {
   map<Units::Time, Weather *>::iterator it = m_weather_by_time.lower_bound(time);
   Units::Time t1;
   if (it == m_weather_by_time.end()) {
      map<Units::Time, Weather *>::reverse_iterator rit = m_weather_by_time.rbegin();
      t1 = rit->first;
   } else {
      t1 = it->first;
   }
   m_weather = m_weather_by_time[t1];
}

void WeatherTruthByTime::LoadEnvFile(string env_csv_file) {
   m_weather_by_time.clear();
   //m_weather_by_distance_to_go.clear();

   if (env_csv_file == "") {
      cout << "No env_csv_file specified; using default weather." << endl;
      // parameter missing, use default
      m_weather = new Weather();
      m_weather->Vwx = Units::MetersPerSecondSpeed(0);
      m_weather->Vwy = Units::MetersPerSecondSpeed(0);
      m_weather->dVwx_dh = Units::HertzFrequency(0);
      m_weather->dVwy_dh = Units::HertzFrequency(0);
      m_weather->temperature = Units::CelsiusTemperature(25);
      m_weather_by_time[Units::SecondsTime(0)] = m_weather;
      // m_weather_by_distance_to_go[Units::MetersLength(0)] = m_weather;
      return;
   }

   FILE *env = fopen(env_csv_file.c_str(), "r");
   if (!env) {
      cout << "Could not open env_csv_file: " << env_csv_file << endl;
      exit(1);
   }

   char line[100];
   fgets(line, sizeof(line), env);   // skip the header line
   while (!feof(env)) {
      double time, distToGo, Vwx, Vwy, dVwx_dh, dVwy_dh, temperature;
      fgets(line, sizeof(line), env);
      if (strlen(line) > 6) {
         sscanf(line, "%lf,%lf,%lf,%lf,%lf,%lf,%lf",
               &time, &distToGo, &Vwx, &Vwy,
               &dVwx_dh, &dVwy_dh, &temperature);
         m_weather = new Weather();
         m_weather->Vwx = Units::MetersPerSecondSpeed(Vwx);
         m_weather->Vwy = Units::MetersPerSecondSpeed(Vwy);
         m_weather->dVwx_dh = Units::HertzFrequency(dVwx_dh);
         m_weather->dVwy_dh = Units::HertzFrequency(dVwy_dh);
         m_weather->temperature = Units::KelvinTemperature(temperature);
         m_weather_by_time[Units::SecondsTime(time)] = m_weather;
         //m_weather_by_distance_to_go[Units::MetersLength(distToGo)] = m_weather;
      }
   }
   m_weather = NULL;
   fclose(env);
   Wind::SetUseWind(true);
}

/** Ignore parameters, return temp from set weather */
Units::KelvinTemperature WeatherTruthByTime::GetTemperature(
      const Units::Length h) const {
   return m_weather->temperature;
}

/** Ignore parameters, return temp from set weather */
Units::Temperature WeatherTruthByTime::InterpolateTemperature(
      const Units::Angle latitude_in, const Units::Angle longitude_in,
      const Units::Length altitude) {
   return m_weather->temperature;
}

/** Ignore parameters, return wind components from set weather */
void WeatherTruthByTime::InterpolateWind(Units::Angle latitude_in,
      Units::Angle longitude_in, Units::Length altitude, Units::Speed& u,
      Units::Speed& v) {
   u = m_weather->Vwx;
   v = m_weather->Vwy;
}

/** Ignore parameters, return wind components from set weather */
void WeatherTruthByTime::InterpolateWindScalar(Units::Angle lat_in,
      Units::Angle lon_in, Units::Length altitude, Units::Speed& east_west,
      Units::Speed& north_south) {
   east_west = m_weather->Vwx;
   north_south = m_weather->Vwy;
}

/** Ignore lat/lon, return wind matrix from set weather, centered on alt_in */
void WeatherTruthByTime::InterpolateWindMatrix(Units::Angle lat_in,
      Units::Angle lon_in, Units::Length alt_in, WindStack& east_west,
      WindStack& north_south) {
   // Set up 5-altitude WindStacks which match the gradient
   east_west.setBounds(1, 5);
   north_south.setBounds(1, 5);
   const Units::FeetLength alt_step(1);
   for (int i = 1; i <= 5; i++) {
      double offset = 3 - i;  // zero at center, descending
      Units::Length altitude = alt_in + offset * alt_step;
      Units::Speed u = m_weather->Vwx + offset * alt_step * m_weather->dVwx_dh;
      Units::Speed v = m_weather->Vwy + offset * alt_step * m_weather->dVwy_dh;
      east_west.set(i, altitude, u);
      north_south.set(i, altitude, v);
   }
}

shared_ptr<WeatherTruthByTime> WeatherTruthByTime::GetSharedPtr() const {
   return m_shared_ptr;
}