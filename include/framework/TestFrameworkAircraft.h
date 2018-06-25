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
 * TestFrameworkAircraft.h
 *
 *  Created on: Mar 2, 2015
 *      Author: sbowman
 */

#ifndef TESTFRAMEWORKAIRCRAFT_H_
#define TESTFRAMEWORKAIRCRAFT_H_

#include "public/Aircraft.h"
#include "TestFrameworkApplication.h"
#include "public/AircraftControl.h"
#include "public/AircraftState.h"
#include "public/SimulationTime.h"
#include "TestFrameworkDynamics.h"
#include "TrajectoryFromFile.h"
#include "public/WeatherPrediction.h"
#include "AircraftIntentFromFile.h"
#include "TestFrameworkFMS.h"
#include "math/DMatrix.h"
#include "loader/DecodedStream.h"
#include <queue>
#include <string>
#include <vector>
#include <fstream>
#include <memory>
#include "Speed.h"
#include "Length.h"

class TestFrameworkAircraft: public Aircraft {
public:
	TestFrameworkAircraft();
	virtual ~TestFrameworkAircraft();
	TestFrameworkAircraft(const TestFrameworkAircraft &in);
	TestFrameworkAircraft& operator=(const TestFrameworkAircraft &in);

	/**
	 * Override the base class loader
	 */
	bool load(DecodedStream *input);

	bool update(const SimulationTime& time);

	void post_load(Units::Time simulation_time_step, int predictedWindOpt, bool blendWind);

	void init(Units::Length adsbReceptionRangeThreshold);

	//Other Member Data:
	int start_time;
	int id;
	AircraftState truth_state_vector_old;
	//StateModel state_model;
	AircraftIntentFromFile mAircraftIntent;
	TestFrameworkDynamics mDynamics;
	AircraftIntent mTargetIntentNotUsed; // NOT USED, but some signatures require it still
	TrajectoryFromFile precalc_traj;
	TestFrameworkApplication airborne_app;
	TestFrameworkFMS mFms;
	AircraftControl mAircraftControl;	// for FMS

private:
	void init_truth_state_vector_old(void);

	// helper method to calculate the end time of the aircraft
	double calculate_end_time(AircraftState vector_in);

	void copy(const TestFrameworkAircraft &in);

	// Data Members
	bool mIsFinished;

	// Wind data, setup as 5x2 matrices.
	// 1st Column:altitudes in feet.
	// 2nd Column:wind speed in feet/sec.
//	DMatrix mPredictedWindX;
//	DMatrix mPredictedWindY;
	// WeatherPrediction mPredictedWind;

	// Initialization loadables
	Units::FeetLength mInitialAltitude;
	Units::KnotsSpeed mInitialIas;
	double mInitialMach; // default 0.78

	// std::shared_ptr<AircraftControl> mAircraftControl;
//	static FILE *mGuidanceOut;
};

#endif /* TESTFRAMEWORKAIRCRAFT_H_ */
