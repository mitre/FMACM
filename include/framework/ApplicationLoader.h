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

#include <vector>
#include <map>
#include <memory>

#include "public/LoggingLoadable.h"
#include "public/FlightDeckApplication.h"
#include "public/WeatherPrediction.h"
#include "framework/SpeedCommandsLoader.h"
#include "scalar/Time.h"

#ifdef SAMPLE_ALGORITHM_LIBRARY
#include "imalgs/IMTimeBasedAchieve.h"
#include "imalgs/IMDistBasedAchieve.h"
#endif

namespace fmacm {
class ApplicationLoader final : public Loadable {
  public:
   ApplicationLoader();
   ~ApplicationLoader() = default;

   bool load(DecodedStream *input) override;

   std::shared_ptr<aaesim::open_source::FlightDeckApplication> CreateApplication(
         aaesim::open_source::WeatherPrediction &avionic_weather_predictor);

   bool IsLoaded() const { return m_loaded; }

  private:
   class PilotConfiguration final : public Loadable {
     public:
      PilotConfiguration() : m_is_enabled(false), m_delay_duration(0) {}
      bool load(DecodedStream *input) override;
      bool IsEnabled() const { return m_is_enabled; }
      Units::SecondsTime DelayDuration() const { return m_delay_duration; }

     private:
      bool m_is_enabled;
      Units::SecondsTime m_delay_duration;
   };
   bool m_loaded{false};
   fmacm::loader::SpeedCommandsLoader m_im_speed_command_file{};
   PilotConfiguration m_pilot_delay_configuration{};

#ifdef SAMPLE_ALGORITHM_LIBRARY
   interval_management::open_source::IMTimeBasedAchieve m_sample_algorithm_time_goal;
   interval_management::open_source::IMDistBasedAchieve m_sample_algorithm_distance_goal;
#endif
};
}  // namespace fmacm