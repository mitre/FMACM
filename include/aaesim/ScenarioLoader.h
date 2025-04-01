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

#include <optional>
#include <filesystem>

#include "loader/DecodedStream.h"
#include "public/Atmosphere.h"
#include "public/Scenario.h"
#include "public/ScenarioEventNotifier.h"
#include "aaesim/AircraftEntity.h"
#include "aaesim/AircraftDataIterationWriter.h"
#include "aaesim/AircraftLoader.h"
#include "aaesim/StartTimeAlgorithm.h"
#include "aaesim/AircraftDataSimulationTimeWriter.h"
#include "aaesim/ScenarioConfiguration.h"
#include "aaesim/DataGatherer.h"
#include "aaesim/ScenarioEntityIterationBuilder.h"
#include "aaesim/KineticPredictionEntityLoader.h"
#include "aaesim/EntityFactoryResult.h"

namespace aaesim {
namespace loaders {
class ScenarioLoader final : public LoggingLoadable {
  public:
   ScenarioLoader();
   bool load(DecodedStream *input) override;
   std::unique_ptr<aaesim::open_source::Scenario> BuildScenario(const std::filesystem::path &scenario_name);

   aaesim::ScenarioConfiguration GetScenarioConfiguration() { return m_scenario_configuration; }

  private:
   inline static aaesim::EntityFactoryResult empty_result{};
   static Units::NauticalMilesLength DEFAULT_ADS_B_RECEPTION_RANGE_THRESHOLD;
   static std::string BADA_DATA_PATH_LINUX_NFS;
   static log4cplus::Logger m_logger;
   static std::map<std::string, Atmosphere::AtmosphereType> m_atmosphere_type_converter;
   void RegisterLoadableParameters();
   const aaesim::EntityFactoryResult BuildAircraftEntityScenarioObjects(
         const aaesim::ScenarioConfiguration &configuration);
   const aaesim::EntityFactoryResult BuildKineticPredictionEntityScenarioObjects(
         const aaesim::ScenarioConfiguration &configuration);
   bool ValidateLoadedScenarioValues();
   aaesim::ScenarioConfiguration BuildScenarioConfiguration(const std::filesystem::path &validated_scenario_filename);
   void ReadScenarioFiles(const std::filesystem::path &full_scenario_filename);
   void AppendFactoryResult(const aaesim::EntityFactoryResult &factory_result);
   void AppendIfUnique(std::shared_ptr<aaesim::DataGatherer> &data_gatherer);

   std::string m_wind_truth_file;
   std::string m_wind_forecast_file;
   std::string m_ttv_file;
   std::string m_bada_data_path;
   double m_rng_seed;
   double m_start_time_rng_seed;
   double m_true_temperature_offset;
   std::string m_atmosphere_type{"BADA3.7"};
   int m_predicted_wind_option;
   int m_number_of_iterations;
   bool m_use_wind_files_loaded;
   bool m_start_time_is_overridden;
   bool m_enable_verbose_output_data;
   bool m_load_completed;
   Units::NauticalMilesLength m_adsb_reception_range_threshold;
   Units::SecondsTime m_mean_interdelivery_time;
   Units::SecondsTime m_standard_deviation_interdelivery_time;
   RandomGenerator m_start_time_latency_generator;
   std::vector<aaesim::loaders::AircraftLoader> m_aircraft_loaders;
   std::vector<std::shared_ptr<aaesim::open_source::ScenarioEventNotifier>> m_scenario_notifiers;
   std::vector<std::shared_ptr<aaesim::AircraftDataSimulationTimeWriter>> m_simtime_writers;
   std::vector<std::shared_ptr<aaesim::DataGatherer>> m_data_gatherers;
   std::vector<aaesim::loaders::KineticPredictionEntityLoader> m_kinetic_prediction_loaders;
   std::vector<std::shared_ptr<aaesim::ScenarioEntityIterationBuilder>> m_all_iteration_builders;
   std::vector<std::string> m_data_gatherer_filenames;
   aaesim::ScenarioConfiguration m_scenario_configuration;
};
}  // namespace loaders
}  // namespace aaesim
