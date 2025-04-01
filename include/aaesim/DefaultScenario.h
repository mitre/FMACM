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

#include "public/Scenario.h"

#include <vector>
#include <string>

#include <scalar/Time.h>
#include <scalar/Length.h>
#include "public/SimulationTime.h"
#include "public/ScenarioEventNotifier.h"
#include "aaesim/ScenarioEntityIterationBuilder.h"

namespace aaesim {
class DefaultScenario final : public aaesim::open_source::Scenario {
  public:
   class Builder final {
     public:
      Builder(const std::string &scenario_name)
         : number_of_iterations_(0),
           scenario_notifiers_(),
           scenario_entity_iteration_builders_(),
           data_gatherers_(),
           scenario_name_(scenario_name){};
      ~Builder() = default;
      std::unique_ptr<DefaultScenario> Build() const;
      Builder *NumberOfIterations(int iteration_count);
      Builder *WithScenarioEventNotifiers(
            const std::vector<std::shared_ptr<aaesim::open_source::ScenarioEventNotifier>> &scenario_writers);
      Builder *WithEntityIterationDataBuilders(
            const std::vector<std::shared_ptr<aaesim::ScenarioEntityIterationBuilder>>
                  &scenario_entity_iteration_builders);
      Builder *WithDataGatherers(const std::vector<std::shared_ptr<aaesim::DataGatherer>> &iteration_gatherers);

      const int GetNumberOfIterations() const { return number_of_iterations_; }
      std::string GetScenarioName() const { return scenario_name_; }

      std::vector<std::shared_ptr<aaesim::open_source::ScenarioEventNotifier>> GetScenarioEventNotifiers() const {
         return scenario_notifiers_;
      }
      std::vector<std::shared_ptr<aaesim::DataGatherer>> GetDataGatherers() const { return data_gatherers_; }
      std::vector<std::shared_ptr<aaesim::ScenarioEntityIterationBuilder>> GetEntityBuilders() const {
         return scenario_entity_iteration_builders_;
      }

     private:
      int number_of_iterations_;
      std::vector<std::shared_ptr<aaesim::open_source::ScenarioEventNotifier>> scenario_notifiers_;
      std::vector<std::shared_ptr<aaesim::ScenarioEntityIterationBuilder>> scenario_entity_iteration_builders_;
      std::vector<std::shared_ptr<aaesim::DataGatherer>> data_gatherers_;
      std::string scenario_name_;
   };

   DefaultScenario(const Builder &builder);
   void SimulateAllIterations() override;

  private:
   static log4cplus::Logger m_logger;
   DefaultScenario();
   void SimulateOneIteration(const int iteration_number);
   void PrepareScenarioEntities(const int iteration_number);
   bool AdvanceSimulationClock(const int iteration_number, aaesim::open_source::SimulationTime &simulation_time);
   void ClearScenario();

   int m_number_of_iterations;

   std::vector<std::shared_ptr<aaesim::ScenarioEntityWithDataCollection>> m_all_entities_in_iteration;
   std::vector<std::shared_ptr<aaesim::open_source::ScenarioEventNotifier>> m_scenario_notifiers;
   std::vector<std::shared_ptr<aaesim::ScenarioEntityIterationBuilder>> m_scenario_entity_iteration_builders;
   std::vector<std::shared_ptr<aaesim::DataGatherer>> m_data_gatherers;
};
}  // namespace aaesim
