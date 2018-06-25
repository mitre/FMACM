// ****************************************************************************
// NOTICE
//
// This is the copyright work of The MITRE Corporation, and was produced
// for the U. S. Government under Contract Number DTFAWA-10-C-00080, and
// is subject to Federal Aviation Administration Acquisition Management
// System Clause 3.5-13, Rights In Data-General, Alt. III and Alt. IV
// (Oct. 1996).  No other use other than that granted to the U. S.
// Government, or to those acting on behalf of the U. S. Government,
// under that Clause is authorized without the express written
// permission of The MITRE Corporation. For further information, please
// contact The MITRE Corporation, Contracts Office, 7515 Colshire Drive,
// McLean, VA  22102-7539, (703) 983-6000. 
//
// Copyright 2018 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#include <sstream>
#include "public/SimulationTime.h"


Units::SecondsTime SimulationTime::simulation_time_step(1.0);
log4cplus::Logger SimulationTime::logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("SimulationTime"));

SimulationTime::SimulationTime(void) 
{
	cycle = 0;
}
SimulationTime::~SimulationTime(void)
{

}
void SimulationTime::init(void)
{
	cycle = 0;
	this->current_time = Units::SecondsTime(0.0);
	simulation_time_step = Units::SecondsTime(1.0);
}

void SimulationTime::increment(void)
{
	cycle++;
	current_time += simulation_time_step;
}

Units::SecondsTime SimulationTime::get_current_simulation_time() const
{
	return current_time;
}

int SimulationTime::get_sim_cycle() const
{
	return cycle;
}

void SimulationTime::set_cycle(int cycle_in)
{
	cycle = cycle_in;
	current_time = simulation_time_step * cycle;
}

SimulationTime::SimulationTime(const SimulationTime &in) {
	copy(in);
}

SimulationTime &SimulationTime::operator=(const SimulationTime &in) {
	if (this != &in) {
		copy(in);
	}
	return *this;
}

const SimulationTime SimulationTime::make(const Units::SecondsTime time) {

  // Computes cycle from time and returns SimulationTime object from time
  // and cycle.  Warning produced if cycle calculation does not give exact
  // calculation using time and time step.
  //
  // time:input time
  //
  // returns simulation time object.

  SimulationTime simtime;

  int cyc = (int) (time/simulation_time_step);

  simtime.set_cycle(cyc);

  if (time != simtime.get_current_simulation_time()) {
    LOG4CPLUS_WARN(SimulationTime::logger, "Inconsistent SimulationTime::make result computing cycle from time "
		   << time.value() << " for step " << SimulationTime::get_simulation_time_step().value() << std::endl);
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
