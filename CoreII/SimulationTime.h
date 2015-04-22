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

#pragma once

class SimulationTime
{
public:
	//Data:


	//Methods:
	SimulationTime(void);
	~SimulationTime(void);
	void init(void);
	void increment(void);
	double get_current_simulation_time();
	static double get_simulation_time_step()
	{
		return simulation_time_step;		
	}
	static void set_simulation_time_step(double in)
	{
		simulation_time_step = in;;		
	}

	int get_sim_cycle();

	void set_time(double time_in);
	void set_cycle(int cycle_in);
	void set_step(double step_in);

private:
	//Data:
	int cycle;
	double current_time;
	static double simulation_time_step;


};
