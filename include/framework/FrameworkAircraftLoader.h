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
#include "public/FixedMassAircraftPerformance.h"
#include "public/WeatherTruth.h"
#include "framework/TestFrameworkAircraft.h"
#include "framework/GuidanceDataLoader.h"
#include "framework/ApplicationLoader.h"

namespace fmacm {
class FrameworkAircraftLoader final : public LoggingLoadable {
  public:
   FrameworkAircraftLoader();
   bool load(DecodedStream *input) override;
   std::shared_ptr<TestFrameworkAircraft> BuildAircraft();

  private:
   static double m_mass_fraction_default, m_start_time_default;
   int m_start_time{0};
   double m_mass_fraction{0};
   std::string m_ac_type{};
   std::string m_speed_management_type{};
   std::string m_ttv_csv_file{};
   std::string m_forewind_csv_file{};
   std::string m_env_csv_file{};
   std::string m_env_csv_data_index{};
   fmacm::GuidanceDataLoader m_guidance_loader{};
   fmacm::ApplicationLoader m_flightdeck_application_loader{};
   EarthModel::LocalPositionEnu m_initial_local_position{};

   std::shared_ptr<aaesim::open_source::FixedMassAircraftPerformance> BuildAircraftPerformance(
         std::string bada_aircraft_code);
   std::shared_ptr<aaesim::open_source::ThreeDOFDynamics> BuildAircraftDynamics(
         std::shared_ptr<aaesim::open_source::FixedMassAircraftPerformance> &performance,
         std::shared_ptr<fmacm::WeatherTruthFromStaticData> &true_weather,
         std::shared_ptr<fmacm::GuidanceFromStaticData> &guidance,
         const EarthModel::GeodeticPosition &initial_wgs84_position);
   std::shared_ptr<aaesim::open_source::AircraftControl> BuildAircraftControl(
         std::string control_method,
         std::shared_ptr<aaesim::open_source::FixedMassAircraftPerformance> &bada_calculator);
   std::shared_ptr<fmacm::WeatherTruthFromStaticData> BuildTrueWeather(std::string env_csv_file,
                                                                       std::string env_csv_data_index,
                                                                       Units::Length initial_altitude);
   std::shared_ptr<aaesim::open_source::ADSBReceiver> BuildAdsbReceiver(std::string ttv_csv_file);
   aaesim::open_source::AircraftState BuildInitialState(
         std::shared_ptr<aaesim::open_source::ThreeDOFDynamics> &dynamics, Units::Time start_time,
         Units::Length initial_altitude, const EarthModel::GeodeticPosition &wgs84_position);
   EarthModel::LocalPositionEnu ComputeInitialPositionOnPath(std::shared_ptr<GuidanceFromStaticData> &guidance);
   std::tuple<bool, EarthModel::LocalPositionEnu> IsDistanceAlongPathValid(
         std::shared_ptr<fmacm::GuidanceFromStaticData> &guidance,
         const Units::Length &target_distance_along_path) const;
   std::shared_ptr<aaesim::open_source::FlightDeckApplication> BuildFlightdeckApplication(
         std::shared_ptr<aaesim::open_source::FixedMassAircraftPerformance> &performance,
         std::shared_ptr<aaesim::open_source::ADSBReceiver> &receiver, const aaesim::open_source::AircraftState &state,
         const GuidanceFromStaticData::PlannedDescentParameters &guidance_descent_parameters);
};
}  // namespace fmacm
