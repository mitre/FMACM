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

#include <string>
#include <scalar/Time.h>
#include "public/Logging.h"

namespace aaesim {
namespace open_source {
class SimulationTime final {
  public:
   static const Units::SecondsTime GetSimulationTimeStep() { return m_simulation_time_step; }

   static const SimulationTime Of(const Units::SecondsTime time);

   // visible for testing
   static void SetSimulationTimeStep(Units::Time in) { m_simulation_time_step = in; }

   SimulationTime();

   ~SimulationTime() = default;

   SimulationTime(const SimulationTime &in);

   SimulationTime &operator=(const SimulationTime &in);

   void Increment();

   Units::SecondsTime GetCurrentSimulationTime() const;

   std::string GetCurrentSimulationTimeAsString() const;

   int GetCycle() const;

   void SetCycle(int cycle_in);

  private:
   void Copy(SimulationTime const &in);

   int m_cycle;
   Units::SecondsTime m_current_time;
   static Units::SecondsTime m_simulation_time_step;
   static log4cplus::Logger m_logger;
};
}  // namespace open_source
}  // namespace aaesim
