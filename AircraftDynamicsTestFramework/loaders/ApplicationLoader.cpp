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

#include "framework/ApplicationLoader.h"

#include "public/NullFlightDeckApplication.h"
#include "public/StatisticalPilotDelay.h"

#ifdef SAMPLE_ALGORITHM_LIBRARY
#include "imalgs/FIMAlgorithmAdapter.h"
#include "imalgs/IMUtils.h"
#endif

using namespace fmacm;

ApplicationLoader::ApplicationLoader() {
#ifdef SAMPLE_ALGORITHM_LIBRARY
   m_sample_algorithm_time_goal = interval_management::open_source::IMTimeBasedAchieve();
   m_sample_algorithm_distance_goal = interval_management::open_source::IMDistBasedAchieve();
#endif
}

bool ApplicationLoader::load(DecodedStream *input) {
   set_stream(input);
   register_loadable_with_brackets("pilot_delay_configuration", &m_pilot_delay_configuration, false);
   register_loadable_with_brackets("im_speed_commands_from_file", &m_im_speed_command_file, false);
#ifdef SAMPLE_ALGORITHM_LIBRARY
   register_loadable_with_brackets("sample_algorithm_time_goal", &m_sample_algorithm_time_goal, false);
   register_loadable_with_brackets("sample_algorithm_distance_goal", &m_sample_algorithm_distance_goal, false);
#endif
   m_loaded = complete();
   return m_loaded;
}

std::shared_ptr<aaesim::open_source::FlightDeckApplication> ApplicationLoader::CreateApplication(
      aaesim::open_source::WeatherPrediction &avionic_weather_predictor) {
   if (m_im_speed_command_file.IsLoaded()) {
      Units::Time delay_duration =
            m_pilot_delay_configuration.IsEnabled() ? m_pilot_delay_configuration.DelayDuration() : Units::ZERO_TIME;
      auto speed_commands = m_im_speed_command_file.Build(delay_duration);
      return std::make_shared<SpeedCommandsFromStaticData>(speed_commands);
   }

   auto statistical_pilot_delay = aaesim::open_source::StatisticalPilotDelay::NoDelay();
   if (m_pilot_delay_configuration.IsEnabled()) {
      statistical_pilot_delay = aaesim::open_source::StatisticalPilotDelay::WithDelay(
            m_pilot_delay_configuration.DelayDuration(), Units::zero(), avionic_weather_predictor.getAtmosphere());
   }

#ifdef SAMPLE_ALGORITHM_LIBRARY
   if (m_sample_algorithm_time_goal.IsLoaded()) {
      std::shared_ptr<interval_management::open_source::IMAlgorithm> time_based_achieve_algorithm =
            std::make_shared<interval_management::open_source::IMTimeBasedAchieve>(m_sample_algorithm_time_goal);
      time_based_achieve_algorithm->SetPilotDelay(statistical_pilot_delay);
      auto adapted_algorithm = std::make_shared<interval_management::open_source::FIMAlgorithmAdapter>(
            time_based_achieve_algorithm, IMUtils::IMAlgorithmTypes::TIMEBASEDACHIEVE);
      return std::static_pointer_cast<aaesim::open_source::FlightDeckApplication>(adapted_algorithm);
   }
   if (m_sample_algorithm_distance_goal.IsLoaded()) {
      std::shared_ptr<interval_management::open_source::IMAlgorithm> distance_based_achieve_algorithm =
            std::make_shared<interval_management::open_source::IMDistBasedAchieve>(m_sample_algorithm_distance_goal);
      distance_based_achieve_algorithm->SetPilotDelay(statistical_pilot_delay);
      auto adapted_algorithm = std::make_shared<interval_management::open_source::FIMAlgorithmAdapter>(
            distance_based_achieve_algorithm, IMUtils::IMAlgorithmTypes::DISTANCEBASEDACHIEVE);
      return std::static_pointer_cast<aaesim::open_source::FlightDeckApplication>(adapted_algorithm);
   }
#endif

   return std::make_shared<aaesim::open_source::NullFlightDeckApplication>();
}

bool ApplicationLoader::PilotConfiguration::load(DecodedStream *input) {
   set_stream(input);
   register_var("use_pilot_delay", &m_is_enabled, false);
   register_var("pilot_delay_seconds", &m_delay_duration, false);
   return complete();
}
