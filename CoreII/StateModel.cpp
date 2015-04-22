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

#include "StateModel.h"


StateModel::StateModel(void)
{
	model_loaded = false;
	finished = false;
	pathLengthLeft = 99999.99999; // Something really big.
}

StateModel::~StateModel(void)
{
}

// master load method to load all the new State model elements
bool StateModel::load(DecodedStream *input)
{
	bool result = false;

	set_stream(input); // set input stream

	// register the state models that have load methods
	dynamics = new ThreeDOFDynamics();
	register_loadable_with_brackets("dynamics", dynamics);
	register_loadable_with_brackets("airborne_app", &airborne_app);
	register_loadable_with_brackets("precalc_traj_file", &precalc_traj);

	// do the actual read
	result = complete(); 

	model_loaded = result;

	return result;
}

// method to initialize the aircraft and the new State model elements
// takes aircraft intent to initialize the Fms and initial state values
void StateModel::init(int id_in, bool blendWind,
		      double start_time, AircraftIntent *intent_in,
		      AircraftIntent *target_in, Units::Length &initialAltitude,
		      Units::Speed &initialIas, double initialMach)
{

	// initialize the FMS information
	Fms.copy_waypoints_from_intent( *intent_in); // copy in waypoint data from aircraft intent
	Fms.init(); // initialize the FMS data

	dynamics->setFms(&Fms);

	if (precalc_traj.is_loaded())
	{
	  precalc_traj.calculateWaypoints(*intent_in);
	}


	// initialize eom dynamics
	const TrajectoryFromFile::VerticalData verticalTrajectoryFromFile = this->precalc_traj.getVerticalData();
	const Units::MetersLength finalAltitude = Units::MetersLength(verticalTrajectoryFromFile.mAlt[0]);
	dynamics->init(precalc_traj.mMassPercentile, finalAltitude, Units::MetersLength(initialAltitude),
		      Units::MetersPerSecondSpeed(initialIas), initialMach, start_time);
}

bool StateModel::is_loaded()
{
	return (model_loaded && dynamics->is_loaded());
}

bool StateModel::is_finished()
{
	return (Fms.is_finished() || (this->pathLengthLeft < -200.0));
}

void StateModel::recomputePathLengthLeft(double x, double y) {
	// Calls getPathLengthFromPos to compute the path length.
	//
	// x,y:position in meters.

	double dummyCourse;

	AircraftCalculations::getPathLengthFromPos(x,y,precalc_traj.h_traj, pathLengthLeft, dummyCourse);
}

