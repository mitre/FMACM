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

#include <sstream>
#include "public/SimulationTime.h"

using namespace aaesim::open_source;

Units::SecondsTime SimulationTime::m_simulation_time_step(1.0);
log4cplus::Logger SimulationTime::m_logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("SimulationTime"));

SimulationTime::SimulationTime() : m_cycle(0), m_current_time(Units::SecondsTime(0.0)) {}

void SimulationTime::Increment() {
   ++m_cycle;
   m_current_time += m_simulation_time_step;
}

Units::SecondsTime SimulationTime::GetCurrentSimulationTime() const { return m_current_time; }

int SimulationTime::GetCycle() const { return m_cycle; }

void SimulationTime::SetCycle(int cycle_in) {
   m_cycle = cycle_in;
   m_current_time = m_simulation_time_step * m_cycle;
}

SimulationTime::SimulationTime(const SimulationTime &in) { Copy(in); }

SimulationTime &SimulationTime::operator=(const SimulationTime &in) {
   if (this != &in) {
      Copy(in);
   }
   return *this;
}

const SimulationTime SimulationTime::Of(const Units::SecondsTime time) {
   int cyc = static_cast<int>(Units::SecondsTime(time / m_simulation_time_step).value());

   SimulationTime simtime;
   simtime.SetCycle(cyc);

   if (time != simtime.GetCurrentSimulationTime()) {
      LOG4CPLUS_ERROR(SimulationTime::m_logger, "Inconsistent SimulationTime::Of result computing cycle from time "
                                                      << time.value() << " for step "
                                                      << SimulationTime::GetSimulationTimeStep().value());
   }

   return simtime;
}

void SimulationTime::Copy(SimulationTime const &in) {
   this->m_current_time = in.m_current_time;
   this->m_cycle = in.m_cycle;
   this->m_simulation_time_step = in.m_simulation_time_step;
}

std::string SimulationTime::GetCurrentSimulationTimeAsString() const {
   std::ostringstream strs;
   strs << this->m_current_time.value();
   return strs.str();
}
