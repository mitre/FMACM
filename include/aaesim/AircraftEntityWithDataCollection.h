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

#include "aaesim/ScenarioEntityWithDataCollection.h"

#include "aaesim/AircraftEntity.h"

namespace aaesim {
class AircraftEntityWithDataCollection : public aaesim::ScenarioEntityWithDataCollection {
  public:
   AircraftEntityWithDataCollection(std::shared_ptr<aaesim::AircraftEntity> delegate)
      : aaesim::ScenarioEntityWithDataCollection(), m_delegate(delegate) {}
   void GatherDataByIteration(const int &iteration_number,
                              const std::vector<std::shared_ptr<aaesim::DataGatherer>> &data_gatherers) override {
      std::for_each(data_gatherers.begin(), data_gatherers.end(),
                    [this, iteration_number](std::shared_ptr<aaesim::DataGatherer> data_gatherer) {
                       data_gatherer->GatherData(iteration_number, m_delegate);
                    });
   }
   void GatherDataBySimTime(const int iteration_number, const aaesim::open_source::SimulationTime &simulation_time,
                            const std::vector<std::shared_ptr<aaesim::DataGatherer>> &data_gatherers) override {
      std::for_each(data_gatherers.begin(), data_gatherers.end(),
                    [this, iteration_number, simulation_time](std::shared_ptr<aaesim::DataGatherer> data_gatherer) {
                       data_gatherer->GatherData(iteration_number, simulation_time, m_delegate);
                    });
   }

   virtual bool Update(const aaesim::open_source::SimulationTime &simulation_time) override {
      return m_delegate->Update(simulation_time);
   }
   virtual const int GetStartTime() const override { return m_delegate->GetStartTime(); }
   virtual bool IsFinished() const override { return m_delegate->IsFinished(); }

  private:
   std::shared_ptr<aaesim::AircraftEntity> m_delegate;
};
}  // namespace aaesim
