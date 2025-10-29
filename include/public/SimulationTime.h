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

#include "scalar/Time.h"

namespace aaesim::open_source {
class SimulationTime final {
  public:
   static inline const Units::SecondsTime SIMULATION_TIME_STEP = Units::SecondsTime(1.0);

   // visible for testing
   static void SetSimulationTimeStep(Units::Time in) { m_simulation_time_step = in; }

   static const Units::SecondsTime GetSimulationTimeStep() { return m_simulation_time_step; }

   static const SimulationTime Of(const Units::SecondsTime time) {
      int cyc = static_cast<int>(Units::SecondsTime(time / m_simulation_time_step).value());
      SimulationTime simtime;
      simtime.SetCycle(cyc);
      return simtime;
   }

   SimulationTime() = default;
   ~SimulationTime() = default;
   SimulationTime(const SimulationTime &in) { Copy(in); }

   SimulationTime &operator=(const SimulationTime &in) {
      if (this != &in) {
         Copy(in);
      }
      return *this;
   }

   bool operator<(const SimulationTime &in) const { return m_cycle < in.m_cycle; }

   bool operator>(const SimulationTime &in) const { return not(*this < in); }

   void Increment() {
      ++m_cycle;
      m_current_time += m_simulation_time_step;
   }

   Units::SecondsTime GetCurrentSimulationTime() const { return m_current_time; }

   std::string GetCurrentSimulationTimeAsString() const { return std::to_string(m_current_time.value()); }

   int GetCycle() const { return m_cycle; }

   // visible for testing
   void SetCycle(int cycle_in) {
      m_cycle = cycle_in;
      m_current_time = m_simulation_time_step * m_cycle;
   }

  private:
   void Copy(SimulationTime const &in) {
      m_current_time = in.m_current_time;
      m_cycle = in.m_cycle;
      m_simulation_time_step = in.m_simulation_time_step;
   }

   int m_cycle{0};
   Units::SecondsTime m_current_time{Units::zero()};
   inline static Units::SecondsTime m_simulation_time_step{SIMULATION_TIME_STEP};
};
}  // namespace aaesim::open_source
