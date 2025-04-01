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

#include "public/LoggingLoadable.h"

#include "aaesim/AdsbDeviceLoader.h"
#include "aaesim/AircraftApplicationLoader.h"
#include "aaesim/AircraftEntity.h"
#include "aaesim/FmsLoader.h"
#include "aaesim/WindAltitudes.h"
#include "avionics/FlightManagementSystem.h"
#include "public/AircraftIntent.h"
#include "public/WeatherPrediction.h"
#include "public/WeatherTruth.h"

namespace aaesim {
namespace loaders {
class AircraftLoader : public LoggingLoadable {
  public:
   AircraftLoader();
   bool load(DecodedStream *input) override;
   std::shared_ptr<aaesim::AircraftEntity> BuildAircraft(
         const int &start_time, const std::shared_ptr<aaesim::open_source::WeatherTruth> &weather_truth,
         const std::string &wind_forecast_file, const aaesim::open_source::PredictedWindOption &predicted_wind_option,
         const Units::Length &adsb_reception_range_threshold);
   virtual std::string GetAircraftId() const;
   virtual Units::SecondsTime GetStartTimeOverride() const;
   const AircraftIntent GetAircraftIntent() const;

  private:
   void BuildFlightManagementSystem(const std::string &wind_forecast_filename,
                                    const aaesim::open_source::PredictedWindOption &predicted_wind_option);
   void BuildAircraftDynamics();
   void BuildFlightDeckApplication();
   void RegisterLoadableParameters();
   Units::CelsiusTemperature ComputeTrueTemperatureOffset(Atmosphere::AtmosphereType bada_atmosphere_version) const;
   aaesim::open_source::WeatherPrediction BuildPredictedWeatherForAvionics(
         const std::string &wind_forecast_file, const aaesim::open_source::PredictedWindOption &predicted_wind_option,
         Atmosphere::AtmosphereType bada_atmosphere_version);
   void ComputeInitialSpeeds(const FlightManagementSystem::InitialConditions &initial_conditions);
   std::shared_ptr<aaesim::open_source::ThreeDOFDynamics> InitializeDynamics(
         const FlightManagementSystem::InitialConditions &initial_conditions,
         const std::shared_ptr<FlightManagementSystem> &flight_management_system);
   aaesim::open_source::AircraftState FillInitialTruthState(
         const FlightManagementSystem::InitialConditions &initial_conditions,
         const std::shared_ptr<aaesim::open_source::ThreeDOFDynamics> &three_dof_dynamics);
   aaesim::open_source::OwnshipFmsPredictionParameters BuildFmsPredictionParameters();
   aaesim::open_source::OwnshipPerformanceParameters BuildOwnshipPerformanceParameters();
   void InitializeInternalIntervalManagementApplication();
   void InitializeFimApplication();
   void InitializeRtaRctApplication();

   std::string m_aircraft_id;
   int m_unique_id;
   std::string m_ac_type;
   int m_start_time;
   AircraftIntent m_aircraft_intent_loader;
   aaesim::loaders::FmsLoader m_fms_loader;
   aaesim::loaders::AdsbDeviceLoader m_adsb_loader;
   aaesim::loaders::AircraftApplicationLoader m_flightdeck_loader;
   WindAltitudes m_predicted_wind_altitude_loader;
   double m_predicted_temp_offset;
   Units::SecondsTime m_start_time_override;
   std::shared_ptr<aaesim::open_source::WeatherTruth> m_true_weather;
   Units::Speed m_initial_ias;
   Units::Speed m_initial_tas;
   std::shared_ptr<aaesim::open_source::ThreeDOFDynamics> m_three_dof_dynamics;
   std::shared_ptr<aaesim::open_source::ADSBReceiver> m_adsb_receiver;
   std::shared_ptr<FlightManagementSystem> m_fms;
   std::shared_ptr<aaesim::open_source::FlightDeckApplication> m_flightdeck_application;
   aaesim::open_source::WeatherPrediction m_avionic_weather_prediction;
};

inline Units::SecondsTime AircraftLoader::GetStartTimeOverride() const {
   return Units::SecondsTime(m_start_time_override);
}

inline std::string AircraftLoader::GetAircraftId() const { return m_aircraft_id; }

inline const AircraftIntent AircraftLoader::GetAircraftIntent() const { return m_aircraft_intent_loader; }

}  // namespace loaders
}  // namespace aaesim
