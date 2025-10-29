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

#include "framework/FrameworkAircraftLoader.h"

#include "framework/ForeWindReader.h"
#include "framework/NullAircraftPerformance.h"
#include "framework/PreloadedAdsbReceiver.h"
#include "loader/NullLoader.h"
#include "public/AircraftControllerFactory.h"
#include "public/EllipsoidalPositionEstimator.h"
#include "public/FullWindTrueWeatherOperator.h"
#include "public/LegacyPositionEstimator.h"
#include "public/NullADSBReceiver.h"
#include "public/NullAtmosphere.h"
#include "public/NullFlightDeckApplication.h"
#include "public/PassThroughAssap.h"
#include "public/SpeedOnPitchControl.h"
#include "public/SpeedOnThrustControl.h"
#include "public/USStandardAtmosphere1976.h"
#include "public/WeatherPrediction.h"
#include "scalar/Length.h"

#ifdef SAMPLE_ALGORITHM_LIBRARY
#include "imalgs/FIMAlgorithmInitializer.h"
#endif

#ifdef MITRE_BADA3_LIBRARY
#include "bada/Bada3Factory.h"
#include "bada/BadaAtmosphere37.h"
#endif

using namespace fmacm;
using namespace aaesim::open_source;

double FrameworkAircraftLoader::m_mass_fraction_default = 0.5;
double FrameworkAircraftLoader::m_start_time_default = 0;

FrameworkAircraftLoader::FrameworkAircraftLoader()
   : m_start_time(m_start_time_default),
     m_mass_fraction(m_mass_fraction_default),
     m_ac_type(),
     m_speed_management_type(),
     m_ttv_csv_file(),
     m_forewind_csv_file(),
     m_env_csv_file(),
     m_env_csv_data_index(),
     m_guidance_loader(),
     m_flightdeck_application_loader() {}

bool FrameworkAircraftLoader::load(DecodedStream *input) {
   set_stream(input);

   register_var("initial_time_seconds", &m_start_time, false);
   register_var("initial_mass_fraction", &m_mass_fraction, false);
   register_var("ac_type", &m_ac_type, true);
   register_var("speed_management_type", &m_speed_management_type, true);
   register_var("env_csv_file", &m_env_csv_file, false);
   register_var("env_data_index", &m_env_csv_data_index, false);
   register_var("ttv_csv_file", &m_ttv_csv_file, false);
   register_var("forewind_csv_file", &m_forewind_csv_file, false);
   register_loadable_with_brackets("fms_guidance_data_files", &m_guidance_loader, true);
   register_loadable_with_brackets("flight_deck_application", &m_flightdeck_application_loader, true);
   return complete();
}

std::shared_ptr<TestFrameworkAircraft> FrameworkAircraftLoader::BuildAircraft() {
   auto bada_calculator = BuildAircraftPerformance(m_ac_type);
   auto guidance_calculator = m_guidance_loader.BuildGuidanceCalculator();
   auto true_weather =
         BuildTrueWeather(m_env_csv_file, m_env_csv_data_index,
                          Units::MetersLength(guidance_calculator->GetVerticalData().m_altitude_meters.back()));
   m_initial_local_position = ComputeInitialPositionOnPath(guidance_calculator);
   EarthModel::GeodeticPosition wgs84;
   m_guidance_loader.GetTangentPlaneSequence()->ConvertLocalToGeodetic(m_initial_local_position, wgs84);
   auto dynamics = BuildAircraftDynamics(bada_calculator, true_weather, guidance_calculator, wgs84);
   auto control = BuildAircraftControl(m_speed_management_type, bada_calculator);
   auto receiver = BuildAdsbReceiver(m_ttv_csv_file);
   auto initial_state =
         BuildInitialState(dynamics, Units::SecondsTime(m_start_time),
                           Units::MetersLength(guidance_calculator->GetVerticalData().m_altitude_meters.back()), wgs84);
   auto flightdeck_application = BuildFlightdeckApplication(bada_calculator, receiver, initial_state,
                                                            m_guidance_loader.GetPlannedDescentParameters());

   TestFrameworkAircraft::Builder builder;
   return builder.WithInitialState(initial_state)
         ->WithAircraftPerformance(bada_calculator)
         ->WithAircraftDynamics(dynamics)
         ->WithAircraftControl(control)
         ->WithTrueWeather(true_weather)
         ->WithAdsbReceiver(receiver)
         ->WithGuidanceCalculator(guidance_calculator)
         ->WithFlightDeckApplication(flightdeck_application)
         ->Build();
}

std::shared_ptr<aaesim::open_source::FixedMassAircraftPerformance> FrameworkAircraftLoader::BuildAircraftPerformance(
      std::string bada_aircraft_code) {
#ifdef MITRE_BADA3_LIBRARY
   aaesim::bada::BadaPerformanceInitialConditions initial_conditions;
   initial_conditions.faf_altitude_msl = Units::FeetLength(1500);
   initial_conditions.initial_flap_configuration = aaesim::open_source::bada_utils::FlapConfiguration::CRUISE;
   initial_conditions.mass_percentile = m_mass_fraction;
   return aaesim::bada::Bada3Factory::CreateBada3ForwardProgression(bada_aircraft_code, initial_conditions);
#else
   return std::make_shared<fmacm::NullAircraftPerformance>();
#endif
}

std::shared_ptr<aaesim::open_source::ThreeDOFDynamics> FrameworkAircraftLoader::BuildAircraftDynamics(
      std::shared_ptr<aaesim::open_source::FixedMassAircraftPerformance> &performance,
      std::shared_ptr<fmacm::WeatherTruthFromStaticData> &true_weather,
      std::shared_ptr<GuidanceFromStaticData> &guidance, const EarthModel::GeodeticPosition &initial_wgs84_position) {
   auto initial_altitude = Units::MetersLength(guidance->GetVerticalData().m_altitude_meters.back());
   Units::KnotsSpeed initial_ias = Units::MetersPerSecondSpeed(guidance->GetVerticalData().m_ias_mps.back());
   DirectionOfFlightCourseCalculator course_calculator = DirectionOfFlightCourseCalculator(
         guidance->GetHorizontalTrajectory(), TrajectoryIndexProgressionDirection::UNDEFINED);
   Units::Angle initial_heading = course_calculator.GetCourseAtPathStart();
   true_weather->Update(aaesim::open_source::SimulationTime::Of(Units::SecondsTime(m_start_time)), Units::infinity(),
                        initial_altitude);
   Units::KnotsSpeed initial_tas = true_weather->getAtmosphere()->CAS2TAS(initial_ias, initial_altitude);
   std::shared_ptr<aaesim::open_source::EllipsoidalPositionEstimator> position_estimator =
         std::make_shared<aaesim::open_source::LegacyPositionEstimator>(m_guidance_loader.GetTangentPlaneSequence(),
                                                                        initial_wgs84_position);
   std::shared_ptr<aaesim::open_source::TrueWeatherOperator> true_weather_operator =
         std::make_shared<aaesim::open_source::FullWindTrueWeatherOperator>(true_weather);
   auto dynamics = std::make_shared<aaesim::open_source::ThreeDOFDynamics>();
   dynamics->Initialize(aaesim::open_source::SimulationTime::Of(Units::SecondsTime(m_start_time)), performance,
                        initial_wgs84_position, m_initial_local_position, initial_altitude, initial_tas,
                        initial_heading, m_mass_fraction, position_estimator, true_weather_operator);
   return dynamics;
}

std::tuple<bool, EarthModel::LocalPositionEnu> FrameworkAircraftLoader::IsDistanceAlongPathValid(
      std::shared_ptr<GuidanceFromStaticData> &guidance_calculator,
      const Units::Length &target_distance_along_path) const {
   static Units::MetersLength along_path_distance_tolerance(10);
   const auto expected_trajectory_index = guidance_calculator->GetHorizontalTrajectory().size() - 1;
   AlongPathDistanceCalculator along_path_calculator(guidance_calculator->GetHorizontalTrajectory(),
                                                     TrajectoryIndexProgressionDirection::DECREMENTING);
   PositionCalculator position_calculator(guidance_calculator->GetHorizontalTrajectory(),
                                          TrajectoryIndexProgressionDirection::DECREMENTING);

   Units::Length computed_along_path_distance;
   Units::Length on_path_position_x(Units::negInfinity()), on_path_position_y(Units::negInfinity());
   Units::UnsignedAngle course_at_point(Units::negInfinity());

   try {
      position_calculator.CalculatePositionFromAlongPathDistance(target_distance_along_path, on_path_position_x,
                                                                 on_path_position_y, course_at_point);
      along_path_calculator.CalculateAlongPathDistanceFromPosition(on_path_position_x, on_path_position_y,
                                                                   computed_along_path_distance);
      bool is_position_valid =
            (along_path_calculator.GetCurrentTrajectoryIndex() == expected_trajectory_index) &&
            (position_calculator.GetCurrentTrajectoryIndex() == expected_trajectory_index) &&
            (Units::abs(computed_along_path_distance - target_distance_along_path) < along_path_distance_tolerance);
      EarthModel::LocalPositionEnu local_position_enu =
            EarthModel::LocalPositionEnu::Of(on_path_position_x, on_path_position_y, Units::zero());
      return std::make_tuple(is_position_valid, local_position_enu);
   } catch (std::exception &e) {
      return std::make_tuple(false, EarthModel::LocalPositionEnu::OfZeros());
   }
}

EarthModel::LocalPositionEnu FrameworkAircraftLoader::ComputeInitialPositionOnPath(
      std::shared_ptr<GuidanceFromStaticData> &guidance_calculator) {
   static Units::MetersLength along_path_distance_step(1);

   Units::Length dtg_guess =
         Units::MetersLength(guidance_calculator->GetHorizontalTrajectory().back().m_path_length_cumulative_meters);
   auto dtg_result = IsDistanceAlongPathValid(guidance_calculator, dtg_guess);
   bool dtg_is_valid = std::get<0>(dtg_result);
   int loop_counter = 0;
   while (!dtg_is_valid && loop_counter < 100) {
      dtg_guess = dtg_guess - along_path_distance_step;
      dtg_result = IsDistanceAlongPathValid(guidance_calculator, dtg_guess);
      dtg_is_valid = std::get<0>(dtg_result);
      ++loop_counter;
   }
   if (!dtg_is_valid) {
      std::string msg =
            "Unable to find a start position on the path that passes validation checks; the HFP file may be malformed";
      throw std::runtime_error(msg);
   }

   return std::get<1>(dtg_result);
}

std::shared_ptr<fmacm::WeatherTruthFromStaticData> FrameworkAircraftLoader::BuildTrueWeather(
      std::string env_csv_file, std::string env_csv_data_index, Units::Length initial_altitude) {
   if (!env_csv_file.empty()) {
      auto weather_truth = std::make_shared<fmacm::WeatherTruthFromStaticData>();
      weather_truth->Initialize(env_csv_file, initial_altitude,
                                fmacm::WeatherTruthFromStaticData::DataIndexFromString(env_csv_data_index));
      return weather_truth;
   } else {
      return std::make_shared<fmacm::WeatherTruthFromStaticData>(
            fmacm::WeatherTruthFromStaticData::CreateZeroTruthWind());
   }
}

std::shared_ptr<aaesim::open_source::AircraftControl> FrameworkAircraftLoader::BuildAircraftControl(
      std::string control_method, std::shared_ptr<aaesim::open_source::FixedMassAircraftPerformance> &bada_calculator) {
   AircraftControllerFactory::DescentSpeedControlStrategy descent_strategy{
         AircraftControllerFactory::DescentSpeedControlStrategy::NONE};
   std::transform(control_method.cbegin(), control_method.cend(), control_method.begin(),
                  [](unsigned char c) { return std::tolower(c); });
   if (control_method == "thrust") {
      descent_strategy = AircraftControllerFactory::DescentSpeedControlStrategy::THRUST;
   } else if (control_method == "pitch") {
      descent_strategy = AircraftControllerFactory::DescentSpeedControlStrategy::PITCH;
   }
   auto descent_config = AircraftControllerFactory::DescentSpeedControlConfig{descent_strategy, Units::KnotsSpeed(20.0),
                                                                              Units::FeetLength(500.0)};
   auto config = AircraftControllerFactory::AircraftControllerConfig{descent_config, Units::DegreesAngle{30}};
   auto aircraft_control = AircraftControllerFactory::BuildForCruiseDescentOnly(config);
   aircraft_control->Initialize(bada_calculator);
   return aircraft_control;
}

std::shared_ptr<aaesim::open_source::ADSBReceiver> FrameworkAircraftLoader::BuildAdsbReceiver(
      std::string ttv_csv_file) {
   if (!ttv_csv_file.empty()) {
      const Units::NauticalMilesLength adsb_reception_distance(90);
      std::shared_ptr<aaesim::open_source::ADSBReceiver> adsb_receiver =
            std::make_shared<fmacm::PreloadedAdsbReceiver>(ttv_csv_file, m_guidance_loader.GetTangentPlaneSequence());
      adsb_receiver->Initialize(adsb_reception_distance);
      return adsb_receiver;
   }
   return std::make_shared<aaesim::open_source::NullADSBReceiver>();
}

aaesim::open_source::AircraftState FrameworkAircraftLoader::BuildInitialState(
      std::shared_ptr<aaesim::open_source::ThreeDOFDynamics> &dynamics, Units::Time start_time,
      Units::Length initial_altitude, const EarthModel::GeodeticPosition &wgs84_position) {
   std::pair<Units::Speed, Units::Speed> wind_components = dynamics->GetWindComponents();
   return aaesim::open_source::AircraftState::Builder(1, start_time)
         .Position(m_initial_local_position.x, m_initial_local_position.y)
         ->Latitude(wgs84_position.latitude)
         ->Longitude(wgs84_position.longitude)
         ->AltitudeMsl(initial_altitude)
         ->GroundSpeed(dynamics->GetDynamicsState().xd, dynamics->GetDynamicsState().yd)
         ->SensedWindComponents(wind_components.first, wind_components.second)
         ->DynamicsState(dynamics->GetDynamicsState())
         ->Build();
}

std::shared_ptr<aaesim::open_source::FlightDeckApplication> FrameworkAircraftLoader::BuildFlightdeckApplication(
      std::shared_ptr<aaesim::open_source::FixedMassAircraftPerformance> &performance,
      std::shared_ptr<aaesim::open_source::ADSBReceiver> &receiver, const aaesim::open_source::AircraftState &state,
      const GuidanceFromStaticData::PlannedDescentParameters &guidance_descent_parameters) {
#ifndef SAMPLE_ALGORITHM_LIBRARY
   return std::make_shared<aaesim::open_source::NullFlightDeckApplication>();
#else

#ifdef MITRE_BADA3_LIBRARY
   auto bada37_atmosphere = aaesim::bada::Bada3Factory::MakeAtmosphereFromTemperatureOffset(
         Atmosphere::AtmosphereType::BADA37, Units::CelsiusTemperature{0});
   aaesim::open_source::WeatherPrediction weather_prediction =
         aaesim::open_source::WeatherPrediction::CreateZeroWindPrediction(bada37_atmosphere);
#else
   aaesim::open_source::WeatherPrediction weather_prediction =
         aaesim::open_source::WeatherPrediction::CreateZeroWindPrediction(std::make_unique<USStandardAtmosphere1976>());
#endif
   if (!m_forewind_csv_file.empty()) {
      testvector::ForeWindReader forecast_wind_reader{m_forewind_csv_file, 1};
      forecast_wind_reader.ReadWind(weather_prediction);
   }
   aaesim::open_source::OwnshipFmsPredictionParameters fms_parameters;
   fms_parameters.expected_cruise_altitude = state.GetAltitudeMsl();
   fms_parameters.maximum_allowable_bank_angle = Units::DegreesAngle(30);
   fms_parameters.transition_altitude = guidance_descent_parameters.planned_transition_altitude;
   fms_parameters.transition_ias = guidance_descent_parameters.planned_transition_ias;
   fms_parameters.transition_mach = guidance_descent_parameters.planned_cruise_mach;
   fms_parameters.weather_prediction = weather_prediction;
   aaesim::open_source::OwnshipPerformanceParameters performance_parameters;
   performance_parameters.aerodynamics = performance->GetAerodynamicsInformation();
   performance_parameters.flap_speeds = performance->GetFlapSpeeds();
   performance_parameters.flight_envelope = performance->GetFlightEnvelopeInformation();
   performance_parameters.mass_data = performance->GetAircraftMassInformation();
   auto surveillance_processor = std::make_shared<aaesim::open_source::PassThroughAssap>();
   surveillance_processor->Initialize(receiver);
   interval_management::open_source::FIMAlgorithmInitializer::Builder initializer_builder;
   interval_management::open_source::FIMAlgorithmInitializer sample_algorithm_initializer =
         initializer_builder.AddOwnshipFmsPredictionParameters(fms_parameters)
               ->AddOwnshipPerformanceParameters(performance_parameters)
               ->AddSurveillanceProcessor(surveillance_processor)
               ->Build();
   auto flightdeck_application = m_flightdeck_application_loader.CreateApplication(weather_prediction);
   flightdeck_application->Initialize(sample_algorithm_initializer);
   return flightdeck_application;
#endif
}
