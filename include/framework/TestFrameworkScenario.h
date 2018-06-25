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

/*
 * TestFrameworkScenario.h
 *
 *  Created on: Mar 2, 2015
 *      Author: sbowman
 */
#ifndef TESTFRAMEWORKSCENARIO_H_
#define TESTFRAMEWORKSCENARIO_H_

#include "public/Scenario.h"
#include "framework/TestFrameworkAircraft.h"
#include "public/SimulationTime.h"
#include "loader/Loadable.h"
#include <list>
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
	bool process_one_cycle(SimulationTime& time);
	bool process_all_aircraft(SimulationTime& time);
	void load_one_scenario_from_scenario_class_into_actor_lists();

	bool load(DecodedStream *input);

private:

	void post_load_aircraft(Units::Time simulation_time_step,
				int predictedWindOpt,
				bool blendWind);

	void post_load(std::string bada_data_path,
		       std::string wind_truth,
		       std::string wind_forecast,
		       int predictedWindOpt,
		       bool blendWind,
		       Units::Time simulation_time_step);

	void recordState(const AircraftState &aircraftState) const;

	// Default declarations for the loadable parameters
	static const Units::SecondsTime mDefaultSimulationTimeStep;

	// scenario initialization
	static const int number_of_iterations = 1;
	static const int number_of_aircraft = 1;
	std::vector<TestFrameworkAircraft> aircraft_list;
	double seed;
	std::vector<TestFrameworkAircraft> master_aircraft_list; // this list is loaded and then copied for each iteration

	//scenario data used for all iterations:
	double mean_inter_delivery_time;
	double stdev_inter_delivery_time;
	double start_time_seed;
	double earth_radius;
	FILE *acstates;
};

#endif /* TESTFRAMEWORKSCENARIO_H_ */
