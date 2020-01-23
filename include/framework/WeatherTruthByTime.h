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
#include <public/StandardAtmosphere.h>
#include <public/WeatherTruth.h>
#include <public/Wind.h>

class WeatherTruthByTime: public StandardAtmosphere, public WeatherTruth, public Wind {
public:
   class Weather
   {
   public:
      Units::Speed Vwx, Vwy;
      Units::Frequency dVwx_dh, dVwy_dh;
      Units::Temperature temperature;
   };

   WeatherTruthByTime();
   virtual ~WeatherTruthByTime();

   void SetWeatherFromTime(Units::Time time);
   void LoadEnvFile(const std::string &env_csv_file);
   std::shared_ptr<WeatherTruthByTime> GetSharedPtr() const;

   virtual Units::KelvinTemperature GetTemperature(const Units::Length h) const;

   virtual Units::KelvinTemperature InterpolateTemperature(const Units::Angle latitude_in,
                                                     const Units::Angle longitude_in,
                                                     const Units::Length altitude);

   virtual Units::Pressure InterpolatePressure(const Units::Angle latitude_in,
                                                     const Units::Angle longitude_in,
                                                     const Units::Length altitude);

protected:

   virtual void InterpolateWind(Units::Angle latitude_in,
                                Units::Angle longitude_in,
                                Units::Length altitude,
                                Units::Speed &u,
                                Units::Speed &v);

   virtual void InterpolateWindScalar(Units::Angle lat_in,
                                      Units::Angle lon_in,
                                      Units::Length altitude,
                                      Units::Speed &east_west,
                                      Units::Speed &north_south);

   virtual void InterpolateWindMatrix(Units::Angle lat_in,
                                      Units::Angle lon_in,
                                      Units::Length alt_in,
                                      WindStack &east_west,
                                      WindStack &north_south);

private:
   std::shared_ptr<WeatherTruthByTime> m_shared_ptr;
   std::map<Units::Time, Weather *> m_weather_by_time;
   Weather *m_weather;
};

