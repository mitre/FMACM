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

#include "aaesim/KineticPredictionEntity.h"
#include "aaesim/KineticPredictionEntityLoader.h"
#include "aaesim/KineticPredictionEntityIterationBuilder.h"
#include "aaesim/KineticPredictionEntityDataGathererAdapter.h"
#include "aaesim/KineticPredictionEntityWithDataCollection.h"
#include "aaesim/PredictionFileKinetic.h"
#include "aaesim/HorizontalPredictionWriter.h"
#include "aaesim/ScenarioConfiguration.h"
#include "aaesim/EntityFactoryResult.h"

namespace aaesim {
class KineticPredictionEntityFactory final {
  public:
   inline static const std::string ENTITY_LOADER_NAME{"kinetic_prediction"};
   static aaesim::EntityFactoryResult BuildKineticPredictionEntityObjects(
         const aaesim::ScenarioConfiguration &configuration, const std::string &scenario_root_name,
         std::vector<aaesim::loaders::KineticPredictionEntityLoader> &kinetic_prediction_loaders) {
      aaesim::EntityFactoryResult result;
      if (kinetic_prediction_loaders.size() == 0) {
         return result;
      }

      auto vertical_prediction_writer = std::make_shared<aaesim::PredictionFileKinetic>();
      vertical_prediction_writer->SetScenarioName(scenario_root_name);
      auto horizontal_prediction_writer = std::make_shared<aaesim::HorizontalPredictionWriter>();
      horizontal_prediction_writer->SetScenarioName(scenario_root_name);
      std::vector<std::shared_ptr<aaesim::KineticPredictionDataIterationWriter>> iteration_writers;
      iteration_writers.push_back(vertical_prediction_writer);
      iteration_writers.push_back(horizontal_prediction_writer);

      std::for_each(iteration_writers.begin(), iteration_writers.end(),
                    [&result](std::shared_ptr<aaesim::KineticPredictionDataIterationWriter> writer) {
                       result.data_gatherers.push_back(
                             std::make_shared<aaesim::KineticPredictionEntityDataGathererAdapter>(writer));
                    });
      result.entity_builder = std::make_shared<aaesim::KineticPredictionEntityIterationBuilder>(
            configuration, kinetic_prediction_loaders);
      return result;
   }
};
}  // namespace aaesim