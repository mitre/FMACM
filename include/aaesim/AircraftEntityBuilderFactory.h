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

#include "aaesim/AircraftEntityDataAdapter.h"
#include "aaesim/AircraftEntity.h"
#include "aaesim/AircraftLoader.h"
#include "aaesim/AircraftEntityIterationBuilder.h"
#include "aaesim/AircraftEntitySimTimeGathererAdapater.h"
#include "aaesim/AircraftEntityWithDataCollection.h"
#include "aaesim/NullScenarioIterationBuilder.h"
#include "aaesim/ScenarioConfiguration.h"
#include "aaesim/DataGatherer.h"
#include "aaesim/EntityFactoryResult.h"
#include "aaesim/StartTimeAlgorithmFactory.h"
#include "imalgs/InternalObserverScenarioWriter.h"
#include "aaesim/AircraftTrajectoryWriter.h"
#include "aaesim/FmsWaypointSequenceFile.h"
#include "rtaalgs/RtaToacDataWriter.h"
#include "rtaalgs/RtaRctDataWriter.h"
#include "aaesim/AdsbReceiverWriter.h"
#include "aaesim/AdsbTransmitterWriter.h"
#include "aaesim/PredictionFileKinetic.h"
#include "aaesim/HorizontalPredictionWriter.h"
#include "aaesim/WeatherPredictionWriter.h"
#include "im_internal/PrecalcWaypointWriter.h"
#include "im_internal/FIMAlgorithmDataWriterAdapter.h"
#include "im_internal/PredictionFileKinematicWriterAdapter.h"

namespace aaesim {

class AircraftEntityBuilderFactory final {
  public:
   inline static const std::string ENTITY_LOADER_NAME{"aircraft"};
   static aaesim::EntityFactoryResult ConstructAircraftEntityObjects(
         const aaesim::ScenarioConfiguration &configuration,
         std::vector<aaesim::loaders::AircraftLoader> &aircraft_entity_loaders, RandomGenerator &start_time_rng) {
      aaesim::EntityFactoryResult factory_result;
      if (aircraft_entity_loaders.size() == 0) return factory_result;

      if (configuration.enable_verbose_output_data) factory_result.scenario_notifiers = BuildScenarioEventNotifiers();
      factory_result.data_gatherers =
            BuildDataGatherers(configuration.scenario_simple_name, configuration.enable_verbose_output_data);
      auto start_time_calculator =
            aaesim::StartTimeAlgorithmFactory::BuildStartTimeAlgorithmFromConfiguration(configuration, start_time_rng);
      factory_result.entity_builder = std::make_shared<aaesim::AircraftEntityIterationBuilder>(
            configuration, aircraft_entity_loaders, start_time_calculator);
      return factory_result;
   }

  private:
   static std::vector<std::shared_ptr<aaesim::open_source::ScenarioEventNotifier>> BuildScenarioEventNotifiers() {
      std::vector<std::shared_ptr<aaesim::open_source::ScenarioEventNotifier>> im_event_notifier = {
            std::make_shared<interval_management::open_source::InternalObserverScenarioWriter>()};
      return im_event_notifier;
   }

   static std::vector<std::shared_ptr<aaesim::AircraftDataIterationWriter>> BuildIterationWritersForAircraftEntities(
         const std::string &scenario_name, bool enable_verbose_output_data) {
      std::vector<std::shared_ptr<aaesim::AircraftDataIterationWriter>> aircraft_iteration_writers;
      aircraft_iteration_writers.push_back(std::make_shared<aaesim::AircraftTrajectoryWriter>());
      aircraft_iteration_writers.push_back(std::make_shared<aaesim::FmsWaypointSequenceFile>());
      aircraft_iteration_writers.push_back(std::make_shared<required_time_of_arrival::RtaToacDataWriter>());
      aircraft_iteration_writers.push_back(std::make_shared<required_time_of_arrival::RtaRctDataWriter>());

      if (enable_verbose_output_data) {
         aircraft_iteration_writers.push_back(std::make_shared<aaesim::AdsbTransmitterWriter>());
         aircraft_iteration_writers.push_back(std::make_shared<aaesim::AdsbReceiverWriter>());
         aircraft_iteration_writers.push_back(std::make_shared<aaesim::PredictionFileKinetic>());
         aircraft_iteration_writers.push_back(std::make_shared<aaesim::HorizontalPredictionWriter>());
         aircraft_iteration_writers.push_back(std::make_shared<aaesim::WeatherPredictionWriter>());
      }

      std::for_each(aircraft_iteration_writers.begin(), aircraft_iteration_writers.end(),
                    [scenario_name](std::shared_ptr<aaesim::AircraftDataIterationWriter> writer) {
                       writer->SetScenarioName(scenario_name);
                    });
      return aircraft_iteration_writers;
   }

   static std::vector<std::shared_ptr<aaesim::AircraftDataSimulationTimeWriter>> BuildSimTimeWritersForAircraftEntities(
         const std::string &scenario_name, bool enable_verbose_output_data) {
      std::vector<std::shared_ptr<aaesim::AircraftDataSimulationTimeWriter>> simtime_writers;
      simtime_writers.push_back(aaesim::FIMAlgorithmDataWriterAdapter::Create());
      if (enable_verbose_output_data) {
         simtime_writers.push_back(std::make_shared<interval_management::PrecalcWaypointWriter>());
         simtime_writers.push_back(aaesim::PredictionFileKinematicWriterAdapter::Create());
      }
      std::for_each(simtime_writers.begin(), simtime_writers.end(),
                    [scenario_name](std::shared_ptr<aaesim::AircraftDataSimulationTimeWriter> writer) {
                       writer->SetScenarioName(scenario_name);
                    });
      return simtime_writers;
   }

   static std::vector<std::shared_ptr<aaesim::DataGatherer>> BuildDataGatherers(const std::string &scenario_name,
                                                                                bool enable_verbose_output_data) {
      auto aircraft_iteration_writers =
            BuildIterationWritersForAircraftEntities(scenario_name, enable_verbose_output_data);
      auto simtime_writers = BuildSimTimeWritersForAircraftEntities(scenario_name, enable_verbose_output_data);
      std::vector<std::shared_ptr<aaesim::DataGatherer>> data_gatherers;
      std::for_each(aircraft_iteration_writers.begin(), aircraft_iteration_writers.end(),
                    [&data_gatherers](std::shared_ptr<aaesim::AircraftDataIterationWriter> writer) {
                       data_gatherers.push_back(std::make_shared<aaesim::AircraftEntityDataAdapter>(writer));
                    });
      std::for_each(
            simtime_writers.begin(), simtime_writers.end(),
            [scenario_name, &data_gatherers](std::shared_ptr<aaesim::AircraftDataSimulationTimeWriter> writer) {
               data_gatherers.push_back(std::make_shared<aaesim::AircraftEntitySimTimeGathererAdapater>(writer));
            });
      return data_gatherers;
   }
};
}  // namespace aaesim