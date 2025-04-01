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

#include "aaesim/DataGatherer.h"
#include "aaesim/AircraftDataSimulationTimeWriter.h"

namespace aaesim {
class AircraftEntitySimTimeGathererAdapater final : public aaesim::DataGatherer {
  public:
   AircraftEntitySimTimeGathererAdapater(std::shared_ptr<aaesim::AircraftDataSimulationTimeWriter> data_writer)
      : m_data_writer(data_writer) {}
   ~AircraftEntitySimTimeGathererAdapater() = default;
   void GatherData(const int iteration_number, std::shared_ptr<const aaesim::AircraftEntity> aircraft_entity) override {
      /*nothing to do*/
   }
   void GatherData(const int iteration_number, const aaesim::open_source::SimulationTime &simulation_time,
                   std::shared_ptr<const aaesim::AircraftEntity> aircraft_entity) override {
      m_data_writer->Gather(iteration_number, simulation_time, *aircraft_entity);
   }
   void GatheringComplete() override { m_data_writer->Finish(); }
   std::string GetFilename() override { return m_data_writer->GetOutputFilename(); }

  private:
   std::shared_ptr<aaesim::AircraftDataSimulationTimeWriter> m_data_writer;
};
}  // namespace aaesim