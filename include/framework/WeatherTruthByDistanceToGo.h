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

#include <memory>
#include "public/StandardAtmosphere.h"
#include "public/WeatherTruth.h"
#include "public/Wind.h"

class WeatherTruthByDistanceToGo : public StandardAtmosphere, public aaesim::open_source::WeatherTruth, public Wind {
  public:
   class Weather {
     public:
      Units::Speed Vwx, Vwy;
      Units::Frequency dVwx_dh, dVwy_dh;
      Units::Temperature temperature;
   };

   WeatherTruthByDistanceToGo();
   ~WeatherTruthByDistanceToGo() override;

   void SetWeatherFromDtg(Units::Length dtg);
   void LoadEnvFile(const std::string &env_csv_file);
   std::shared_ptr<WeatherTruthByDistanceToGo> GetSharedPtr() const;

   Units::KelvinTemperature CalculateTemperatureOffset(const Units::Length altitude);

   Units::KelvinTemperature GetTemperature(Units::Length h) const override;

   Units::KelvinTemperature InterpolateTemperature(Units::Angle latitude_in, Units::Angle longitude_in,
                                                   Units::Length altitude) override;

   Units::Pressure InterpolatePressure(Units::Angle latitude_in, Units::Angle longitude_in,
                                       Units::Length altitude) override;

  protected:
   void InterpolateWind(Units::Angle latitude_in, Units::Angle longitude_in, Units::Length altitude, Units::Speed &u,
                        Units::Speed &v) override;

   void InterpolateWindScalar(Units::Angle lat_in, Units::Angle lon_in, Units::Length altitude, Units::Speed &east_west,
                              Units::Speed &north_south) override;

   void InterpolateWindMatrix(Units::Angle lat_in, Units::Angle lon_in, Units::Length alt_in,
                              aaesim::open_source::WindStack &east_west,
                              aaesim::open_source::WindStack &north_south) override;

  private:
   std::shared_ptr<WeatherTruthByDistanceToGo> m_shared_ptr;
   std::map<Units::Length, Weather *> m_weather_by_dtg;
   Weather *m_weather;
};
