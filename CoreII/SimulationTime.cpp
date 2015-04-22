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
// Copyright 2015 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#include "SimulationTime.h"


double SimulationTime::simulation_time_step = 1.0;

SimulationTime::SimulationTime(void) 
{

}
SimulationTime::~SimulationTime(void)
{

}
void SimulationTime::init(void)
{
	cycle = 0;
	this->current_time = 0.0;
	simulation_time_step = 1.0;
}

void SimulationTime::increment(void)
{
	cycle++;
	current_time += simulation_time_step;
}

double SimulationTime::get_current_simulation_time()
{
	return current_time;
}



int SimulationTime::get_sim_cycle()
{
	return cycle;
}

void SimulationTime::set_time(double time_in)
{
	current_time = time_in;
}

void SimulationTime::set_cycle(int cycle_in)
{
	cycle = cycle_in;
}

void SimulationTime::set_step(double step_in)
{
	simulation_time_step = step_in;
}
