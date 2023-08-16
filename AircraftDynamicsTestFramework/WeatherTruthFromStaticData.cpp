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

#include "framework/WeatherTruthFromStaticData.h"

#include <string>

#include "MiniCSV/minicsv.h"

using namespace fmacm;

WeatherTruthFromStaticData::WeatherTruthFromStaticData()
   : m_weather_data_points_by_time(),
     m_weather_data_points_by_dtg(),
     m_weather_data_point(),
     m_data_index(DataIndexParameter::SIMULATION_TIME) {
   m_wind_interpolator = std::make_shared<fmacm::WindInterpolator>();
   m_wind_interpolator->SetUseWind(true);
   m_wind = std::static_pointer_cast<Wind>(m_wind_interpolator);
   m_atmosphere = std::make_shared<StandardAtmosphere>(Units::zero());
}

void WeatherTruthFromStaticData::Update(const aaesim::open_source::SimulationTime &simulation_time,
                                        const Units::Length &current_distance_to_go,
                                        const Units::Length &altitude_msl) {
   if (m_data_index == DataIndexParameter::SIMULATION_TIME) {
      auto it = m_weather_data_points_by_time.lower_bound(simulation_time.GetCurrentSimulationTime());
      Units::Time t1;
      if (it == m_weather_data_points_by_time.end()) {
         auto rit = m_weather_data_points_by_time.rbegin();
         t1 = rit->first;
      } else {
         t1 = it->first;
      }
      m_weather_data_point = m_weather_data_points_by_time[t1];
   } else {
      auto it = m_weather_data_points_by_dtg.lower_bound(current_distance_to_go);
      Units::Length dtg1;
      if (it == m_weather_data_points_by_dtg.end()) {
         auto rit = m_weather_data_points_by_dtg.rbegin();
         dtg1 = rit->first;
      } else {
         dtg1 = it->first;
      }
      m_weather_data_point = m_weather_data_points_by_dtg[dtg1];
   }
   m_temperature = Units::KelvinTemperature(m_weather_data_point.temperature.value());
   m_wind_interpolator->UpdateWindDataPoint(m_weather_data_point);
   LoadConditionsAt(Units::ZERO_ANGLE, Units::ZERO_ANGLE, altitude_msl);
}

Units::KelvinTemperature WeatherTruthFromStaticData::Initialize(const std::string &env_csv_file,
                                                                const Units::Length &altitude,
                                                                DataIndexParameter primary_index) {
   m_data_index = primary_index;
   LoadEnvFile(env_csv_file);
   Update(aaesim::open_source::SimulationTime::Of(Units::ZERO_TIME), Units::Infinity(), altitude);

   const StandardAtmosphere basic_atm(Units::zero());
   Units::Temperature offset_difference = GetTemperature(altitude) - basic_atm.GetTemperature(altitude);
   Units::Temperature offset = basic_atm.GetTemperatureOffset() + offset_difference;
   std::shared_ptr<Atmosphere> atmosphere_with_offset = std::make_shared<StandardAtmosphere>(offset);
   SetAtmosphere(atmosphere_with_offset);

   Update(aaesim::open_source::SimulationTime::Of(Units::ZERO_TIME), Units::Infinity(), altitude);
   return offset;
}

void WeatherTruthFromStaticData::LoadEnvFile(const std::string &env_csv_file) {
   m_weather_data_points_by_time.clear();

   if (env_csv_file.empty()) {
      auto msg = "No env_csv_file specified; please load a weather file.";
      throw std::runtime_error(msg);
   }

   mini::csv::ifstream input_stream(env_csv_file);
   input_stream.set_delimiter(',', ",");
   if (input_stream.is_open()) {
      input_stream.read_line();
      while (input_stream.read_line()) {
         EnvFileRow data_row;
         input_stream >> data_row.simtime_seconds >> data_row.distance_to_go_meters >> data_row.wind_x_enu_mps >>
               data_row.wind_y_enu_mps >> data_row.wind_dx_dh_hz >> data_row.wind_dy_dh_hz >>
               data_row.temperature_kelvin;

         WindInterpolator::WeatherDataPoint data_point;
         data_point.Vwx = Units::MetersPerSecondSpeed(data_row.wind_x_enu_mps);
         data_point.Vwy = Units::MetersPerSecondSpeed(data_row.wind_y_enu_mps);
         data_point.dVwx_dh = Units::HertzFrequency(data_row.wind_dx_dh_hz);
         data_point.dVwy_dh = Units::HertzFrequency(data_row.wind_dy_dh_hz);
         data_point.temperature = Units::AbsKelvinTemperature(data_row.temperature_kelvin);

         m_weather_data_points_by_time[Units::SecondsTime(data_row.simtime_seconds)] = data_point;
         m_weather_data_points_by_dtg[Units::MetersLength(data_row.distance_to_go_meters)] = data_point;
      }
   }
   input_stream.close();
   Wind::SetUseWind(true);
}

Units::KelvinTemperature WeatherTruthFromStaticData::GetTemperature(const Units::Length h) const {
   /** Ignore parameters, return temp from set weather */
   return Units::KelvinTemperature(m_weather_data_point.temperature.value());
}

void WeatherTruthFromStaticData::LoadConditionsAt(const Units::Angle latitude, const Units::Angle longitude,
                                                  const Units::Length altitude) {
   WeatherEstimate::LoadConditionsAt(Units::ZERO_ANGLE, Units::ZERO_ANGLE, altitude);
}
