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

#include "framework/WindInterpolator.h"
#include "public/WeatherTruth.h"
#include "scalar/Speed.h"
#include "public/NullAtmosphere.h"
#include "public/SimulationTime.h"

#ifdef MITRE_BADA3_LIBRARY
#include "bada/BadaAtmosphere37.h"
#define ATMOSPHERE_IMPL aaesim::bada::BadaAtmosphere37
#else
#include "public/NullAtmosphere.h"
#define ATMOSPHERE_IMPL NullAtmosphere
#endif

namespace fmacm {
class WeatherTruthFromStaticData : public aaesim::open_source::WeatherTruth {
  public:
   enum DataIndexParameter {
      SIMULATION_TIME,
      DISTANCE_TO_GO

   };
   static DataIndexParameter DataIndexFromString(std::string s) {
      std::transform(s.cbegin(), s.cend(), s.begin(), [](unsigned char c) { return std::tolower(c); });
      if (s == "time") return DataIndexParameter::SIMULATION_TIME;
      if (s == "dtg") return DataIndexParameter::DISTANCE_TO_GO;
      std::string msg = "Invalid DataIndexParameter string: '" + s + "'. Must be 'dtg' or 'time'.";
      throw std::runtime_error(msg);
   };

   static WeatherTruthFromStaticData CreateZeroTruthWind();
   WeatherTruthFromStaticData();

   Units::KelvinTemperature Initialize(const std::string &env_csv_file, const Units::Length &altitude,
                                       DataIndexParameter primary_index);

   void Update(const aaesim::open_source::SimulationTime &simulation_time, const Units::Length &current_distance_to_go,
               const Units::Length &altitude_msl);

   void LoadConditionsAt(const Units::Angle latitude, const Units::Angle longitude,
                         const Units::Length altitude) override;

  private:
   struct EnvFileRow {
      EnvFileRow()
         : simtime_seconds(0),
           distance_to_go_meters(0),
           wind_x_enu_mps(0),
           wind_y_enu_mps(0),
           wind_dx_dh_hz(0),
           wind_dy_dh_hz(0),
           temperature_kelvin(0) {}
      double simtime_seconds;
      double distance_to_go_meters;
      double wind_x_enu_mps;
      double wind_y_enu_mps;
      double wind_dx_dh_hz;
      double wind_dy_dh_hz;
      double temperature_kelvin;
   };

   void InitializeWithZeros();
   void LoadEnvFile(const std::string &env_csv_file);
   std::map<Units::Time, fmacm::WindInterpolator::WeatherDataPoint> m_weather_data_points_by_time;
   std::map<Units::Length, fmacm::WindInterpolator::WeatherDataPoint> m_weather_data_points_by_dtg;
   fmacm::WindInterpolator::WeatherDataPoint m_weather_data_point;
   std::shared_ptr<fmacm::WindInterpolator> m_wind_interpolator;
   DataIndexParameter m_data_index;
};

inline void WeatherTruthFromStaticData::InitializeWithZeros() {
   m_data_index = DataIndexParameter::SIMULATION_TIME;
   const auto zero_offset_atmosphere = ATMOSPHERE_IMPL(Units::zero());
   fmacm::WindInterpolator::WeatherDataPoint data_point;
   data_point.temperature =
         Units::AbsKelvinTemperature(zero_offset_atmosphere.GetTemperature(Units::ZERO_LENGTH).value());
   m_weather_data_points_by_dtg[Units::ZERO_LENGTH] = data_point;
   m_weather_data_points_by_time[Units::ZERO_TIME] = data_point;
   m_weather_data_points_by_dtg[Units::Infinity()] = data_point;
   m_weather_data_points_by_time[Units::Infinity()] = data_point;

   Update(aaesim::open_source::SimulationTime::Of(Units::ZERO_TIME), Units::Infinity(), Units::Infinity());
}

inline WeatherTruthFromStaticData WeatherTruthFromStaticData::CreateZeroTruthWind() {
   WeatherTruthFromStaticData zero;
   zero.InitializeWithZeros();
   return zero;
}
}  // namespace fmacm
