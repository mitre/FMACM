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

/*
 * TestFrameworkScenario.h
 *
 *  Created on: Mar 2, 2015
 *      Author: sbowman
 */
#ifndef TESTFRAMEWORKSCENARIO_H_
#define TESTFRAMEWORKSCENARIO_H_

#include "Scenario.h"
#include "TestFrameworkAircraft.h"
#include "State.h"
#include "SimulationTime.h"
#include "Loadable.h"
#include "string"
#include "list"
#include <string>
#include <fstream>
#include <iostream>
#include <vector>
#include <Angle.h>
#include <Time.h>

class TestFrameworkScenario: public Scenario {
public:
	TestFrameworkScenario();
	virtual ~TestFrameworkScenario();

	void process_one_scenario();
	void initializeIterationState(int numberOfAircraft);
	void initializeIterationMetrics(int numberOfAircraft);
	void process_one_iteration(int iter);
	bool process_one_cycle(SimulationTime& time, State& old_state, State& new_state);
	bool process_all_aircraft(SimulationTime& time, State& old_state, State& new_state);
	void load_one_scenario_from_scenario_class_into_actor_lists();

	bool load(DecodedStream *input);

private:

	void post_load_aircraft(double simulation_time_step,
				int predictedWindOpt,
				bool blendWind);

	void post_load(string bada_data_path,
		       string wind_truth,
		       string wind_forecast,
		       int predictedWindOpt,
		       bool blendWind,
		       double simulation_time_step);

	// Default declarations for the loadable parameters
	static const Units::SecondsTime mDefaultSimulationTimeStep;

	// scenario initialization
	State initial_state;
	static const int number_of_iterations = 1;
	static const int number_of_aircraft = 1;
	list<TestFrameworkAircraft> aircraft_list;
	double seed;
	list<TestFrameworkAircraft> master_aircraft_list; // this list is loaded and then copied for each iteration

	//scenario data used for all iterations:
	double mean_inter_delivery_time;
	double stdev_inter_delivery_time;
	double start_time_seed;
	double earth_radius;

};

#endif /* TESTFRAMEWORKSCENARIO_H_ */
