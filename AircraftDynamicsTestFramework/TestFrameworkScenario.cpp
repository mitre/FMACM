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

#include "framework/TestFrameworkScenario.h"

#include "framework/AircraftStateWriter.h"

#ifdef MITRE_BADA3_LIBRARY
#include "bada/Bada3Factory.h"
#endif

log4cplus::Logger TestFrameworkScenario::m_logger = log4cplus::Logger::getInstance("TestFrameworkScenario");

TestFrameworkScenario::TestFrameworkScenario() : Scenario(), m_aircraft_in_scenario() {
#ifdef SAMPLE_ALGORITHM_LIBRARY
   m_sample_algorithm_writer = std::make_unique<interval_management::open_source::FIMAlgorithmDataWriter>();
   m_sample_algorithm_kinematic_writer = std::make_unique<interval_management::open_source::PredictionFileKinematic>();
#endif
}

bool TestFrameworkScenario::load(DecodedStream *input) {
   std::string bada_data_path;
   std::vector<fmacm::FrameworkAircraftLoader> aircraft_loaders;

   set_stream(input);
   register_var("bada_data_path", &bada_data_path, true);
   register_named_vector_item("aircraft", &aircraft_loaders, true);
   complete();

   PostLoad(bada_data_path, aircraft_loaders);

   return true;
}

void TestFrameworkScenario::PostLoad(const std::string &bada_data_path,
                                     std::vector<fmacm::FrameworkAircraftLoader> &aircraft_loaders) {
#ifdef MITRE_BADA3_LIBRARY
   aaesim::bada::Bada3Factory::SetBadaDataPath(bada_data_path, Atmosphere::AtmosphereType::BADA37);
#endif

   std::for_each(aircraft_loaders.begin(), aircraft_loaders.end(), [this](fmacm::FrameworkAircraftLoader loader) {
      m_aircraft_in_scenario.push_back(loader.BuildAircraft());
   });
}

void TestFrameworkScenario::SimulateAllIterations() {
#ifdef SAMPLE_ALGORITHM_LIBRARY
   m_sample_algorithm_writer->SetScenarioName(GetScenarioName());
   m_sample_algorithm_kinematic_writer->SetScenarioName(GetScenarioName());
#endif

   fmacm::AircraftStateWriter fmacm_state_writer;
   fmacm_state_writer.SetScenarioName(GetScenarioName());

   LOG4CPLUS_INFO(m_logger, "Running FMACM scenario " << GetScenarioName());
   aaesim::open_source::SimulationTime time;
   bool scenario_complete = false;
   while (!scenario_complete) {
      scenario_complete = AdvanceAllAircraft(time);
      time.Increment();
   }
   LOG4CPLUS_INFO(m_logger, "FMACM scenario complete; writing data files.");
   std::for_each(m_aircraft_in_scenario.cbegin(), m_aircraft_in_scenario.cend(),
                 [&fmacm_state_writer](const std::shared_ptr<TestFrameworkAircraft> &aircraft) {
                    fmacm_state_writer.Gather(aircraft->GetAircraftStates());
                 });

#ifdef SAMPLE_ALGORITHM_LIBRARY
   m_sample_algorithm_writer->Finish();
   m_sample_algorithm_kinematic_writer->Finish();
#endif

   fmacm_state_writer.Finish();
}

bool TestFrameworkScenario::AdvanceAllAircraft(aaesim::open_source::SimulationTime &time) {
   bool all_aircraft_complete = true;
   auto advance_aircraft = [this, &all_aircraft_complete, &time](std::shared_ptr<TestFrameworkAircraft> &aircraft) {
      bool aircraft_finished = aircraft->Update(time);

#ifdef SAMPLE_ALGORITHM_LIBRARY
      m_sample_algorithm_writer->Gather(0, time, "IMACID", aircraft->GetFlightDeckApplication());
      m_sample_algorithm_kinematic_writer->Gather(0, time.GetCurrentSimulationTime(), "IMACID",
                                                  aircraft->GetFlightDeckApplication());
#endif

      all_aircraft_complete = all_aircraft_complete && aircraft_finished;
   };
   std::for_each(m_aircraft_in_scenario.begin(), m_aircraft_in_scenario.end(), advance_aircraft);
   return all_aircraft_complete;
}
