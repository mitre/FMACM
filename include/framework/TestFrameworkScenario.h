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
#include "public/LoggingLoadable.h"

#include <string>
#include <vector>
#include <scalar/Angle.h>
#include <scalar/Time.h>

#include "framework/TestFrameworkAircraft.h"
#include "framework/FrameworkAircraftLoader.h"
#include "public/SimulationTime.h"

#ifdef SAMPLE_ALGORITHM_LIBRARY
#include "imalgs/FIMAlgorithmDataWriter.h"
#include "imalgs/PredictionFileKinematic.h"
#endif

class TestFrameworkScenario final : public aaesim::open_source::Scenario, public LoggingLoadable {
  public:
   TestFrameworkScenario();

   ~TestFrameworkScenario() = default;

   void SimulateAllIterations() override;

   bool load(DecodedStream *input) override;

  private:
   static const Units::SecondsTime SIMULATION_TIME_STEP;
   static log4cplus::Logger m_logger;

   bool AdvanceAllAircraft(aaesim::open_source::SimulationTime &time);
   void PostLoad(const std::string &bada_data_path, std::vector<fmacm::FrameworkAircraftLoader> &aircraft_loaders);

   std::vector<std::shared_ptr<TestFrameworkAircraft>> m_aircraft_in_scenario;

#ifdef SAMPLE_ALGORITHM_LIBRARY
   std::unique_ptr<interval_management::open_source::FIMAlgorithmDataWriter> m_sample_algorithm_writer;
   std::unique_ptr<interval_management::open_source::PredictionFileKinematic> m_sample_algorithm_kinematic_writer;
#endif
};
