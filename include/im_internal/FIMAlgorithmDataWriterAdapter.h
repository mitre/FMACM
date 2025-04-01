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

#include "aaesim/AircraftDataSimulationTimeWriter.h"

#include "imalgs/FIMAlgorithmDataWriter.h"

namespace aaesim {
class FIMAlgorithmDataWriterAdapter final : public aaesim::AircraftDataSimulationTimeWriter {
  public:
   FIMAlgorithmDataWriterAdapter(std::shared_ptr<interval_management::open_source::FIMAlgorithmDataWriter> &data_writer)
      : aaesim::AircraftDataSimulationTimeWriter("", data_writer->GetFileSuffix()), m_data_writer(data_writer){};
   ~FIMAlgorithmDataWriterAdapter() = default;
   void Finish() override { m_data_writer->Finish(); }
   void SetScenarioName(const std::string &scenario_name) override { m_data_writer->SetScenarioName(scenario_name); }
   void Gather(const int &iteration_number, const aaesim::open_source::SimulationTime &time,
               const aaesim::AircraftEntity &aircraft) override {
      const bool record_data =
            !aircraft.IsFinished() && (time.GetCurrentSimulationTime().value() >= aircraft.GetStartTime());
      if (!record_data) return;
      m_data_writer->Gather(iteration_number, time, aircraft.GetAircraftId(), aircraft.GetFlightDeckApplication());
   }
   static std::shared_ptr<FIMAlgorithmDataWriterAdapter> Create() {
      auto writer = std::make_shared<interval_management::open_source::FIMAlgorithmDataWriter>();
      return std::make_shared<FIMAlgorithmDataWriterAdapter>(writer);
   }

  private:
   std::shared_ptr<interval_management::open_source::FIMAlgorithmDataWriter> m_data_writer;
};
}  // namespace aaesim