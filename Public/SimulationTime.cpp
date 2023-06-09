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
// 2022 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#include <sstream>
#include "public/SimulationTime.h"

Units::SecondsTime SimulationTime::simulation_time_step(1.0);
log4cplus::Logger SimulationTime::logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("SimulationTime"));

SimulationTime::SimulationTime() : cycle(0), current_time(Units::SecondsTime(0.0)) {}

SimulationTime::~SimulationTime() = default;

void SimulationTime::increment() {
   ++cycle;
   current_time += simulation_time_step;
}

Units::SecondsTime SimulationTime::get_current_simulation_time() const { return current_time; }

int SimulationTime::get_sim_cycle() const { return cycle; }

void SimulationTime::set_cycle(int cycle_in) {
   cycle = cycle_in;
   current_time = simulation_time_step * cycle;
}

SimulationTime::SimulationTime(const SimulationTime &in) { copy(in); }

SimulationTime &SimulationTime::operator=(const SimulationTime &in) {
   if (this != &in) {
      copy(in);
   }
   return *this;
}

const SimulationTime SimulationTime::make(const Units::SecondsTime time) {
   int cyc = static_cast<int>(Units::SecondsTime(time / simulation_time_step).value());

   SimulationTime simtime;
   simtime.set_cycle(cyc);

   if (time != simtime.get_current_simulation_time()) {
      LOG4CPLUS_WARN(SimulationTime::logger, "Inconsistent SimulationTime::make result computing cycle from time "
                                                   << time.value() << " for step "
                                                   << SimulationTime::get_simulation_time_step().value() << std::endl);
   }

   return simtime;
}

void SimulationTime::copy(SimulationTime const &in) {
   this->current_time = in.current_time;
   this->cycle = in.cycle;
   this->simulation_time_step = in.simulation_time_step;
}

std::string SimulationTime::getCurrentSimulationTimeAsString() const {
   std::ostringstream strs;
   strs << this->current_time.value();
   return strs.str();
}
