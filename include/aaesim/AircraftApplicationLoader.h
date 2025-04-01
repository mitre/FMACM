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
#include "public/FlightDeckApplication.h"
#include "public/FlightDeckApplicationLoader.h"
#include "aaesim/ASSAPLoader.h"
#include "public/ASSAP.h"
#include "public/WeatherPrediction.h"

namespace aaesim {
namespace loaders {
class AircraftApplicationLoader final : public LoggingLoadable {
  public:
   struct PilotDelayConfiguration {
      bool enabled;
      Units::SecondsTime mean;
      Units::SecondsTime standard_deviation;
   };

   AircraftApplicationLoader() = default;
   ~AircraftApplicationLoader() = default;
   bool load(DecodedStream *input) override;
   bool IsLoaded() const;
   std::shared_ptr<aaesim::open_source::FlightDeckApplication> ConstructLoadedAlgorithm(
         aaesim::open_source::WeatherPrediction &weather_prediction);
   std::shared_ptr<aaesim::open_source::ASSAP> ConstructLoadedAssap();
   bool GetPlaceboModeEnabled() const;

  private:
   static log4cplus::Logger m_logger;
   std::map<std::string, std::shared_ptr<aaesim::open_source::FlightDeckApplicationLoader>>
         RegisterFlightdeckAlgorithmLoaders();
   void RegisterGeneralConfigurationParameters();
   aaesim::open_source::StatisticalPilotDelay BuildPilotDelayAlgorithm(std::shared_ptr<Atmosphere> &atmosphere);

   bool m_is_loaded{false};
   bool m_placebo_mode_enabled{false};
   ASSAPLoader m_assap_loader{};
   std::map<std::string, std::shared_ptr<aaesim::open_source::FlightDeckApplicationLoader>> m_flightdeck_loaders{};
   std::string m_loaded_algorithm_name{};
   PilotDelayConfiguration m_loaded_pilot_delay_configuration{};
};
}  // namespace loaders
}  // namespace aaesim

inline bool aaesim::loaders::AircraftApplicationLoader::IsLoaded() const { return m_is_loaded; }

inline bool aaesim::loaders::AircraftApplicationLoader::GetPlaceboModeEnabled() const { return m_placebo_mode_enabled; }