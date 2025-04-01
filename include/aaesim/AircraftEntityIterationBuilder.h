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

#include "aaesim/ScenarioEntityIterationBuilder.h"

#include <vector>

#include "aaesim/AircraftLoader.h"
#include "aaesim/StartTimeAlgorithm.h"
#include "aaesim/ScenarioConfiguration.h"
#include "public/WeatherTruth.h"

namespace aaesim {
class AircraftEntityIterationBuilder : public ScenarioEntityIterationBuilder {
  public:
   AircraftEntityIterationBuilder(const aaesim::ScenarioConfiguration &configuration,
                                  std::vector<aaesim::loaders::AircraftLoader> &aircraft_loaders,
                                  std::shared_ptr<aaesim::StartTimeAlgorithm> start_time_calculator);
   virtual ~AircraftEntityIterationBuilder() = default;
   std::vector<std::shared_ptr<aaesim::ScenarioEntityWithDataCollection>> PrepareEntitiesForOneIteration() override;

  private:
   static log4cplus::Logger m_logger;
   AircraftEntityIterationBuilder();
   std::shared_ptr<aaesim::open_source::WeatherTruth> InitializeTrueWeather();
   void InitializeAdsbEtherState();

   aaesim::ScenarioConfiguration m_scenario_configuration;
   std::vector<aaesim::loaders::AircraftLoader> m_aircraft_loaders;
   std::shared_ptr<aaesim::StartTimeAlgorithm> m_start_time_calculator;
   std::shared_ptr<aaesim::open_source::WeatherTruth> m_true_weather;
};
}  // namespace aaesim
