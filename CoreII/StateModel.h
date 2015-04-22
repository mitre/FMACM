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
#include "Loadable.h"
#include "FlightManagementSystem.h"
#include "ThreeDOFDynamics.h"
#include "AircraftState.h"
#include "AircraftIntent.h"
#include "AirborneApplication.h"
#include "AircraftCalculations.h"
#include "TrajectoryFromFile.h"


// class to unify the new State model members
class StateModel : public Loadable
{
public:
	StateModel(void);
	~StateModel(void);

	// master load method to load all the new State model elements
	bool load(DecodedStream *input);

	// method to initialize the aircraft and the new State model elements
	// takes aircraft intent to initialize the Fms and initial state values
	void init(int id_in, bool BlendWind, double start_time,
		  AircraftIntent *intent_in, AircraftIntent *target_in,
		  Units::Length &initialAltitude, Units::Speed &initialSpeed,
		  double initialMach);

	void recomputePathLengthLeft(double x, double y);

	bool is_loaded();
	bool is_finished();

	double pathLengthLeft; // Computed path length from position-used to help determine
					       // if processing is finished in aircraft.

	// State model elements
	FlightManagementSystem Fms;
	ThreeDOFDynamics *dynamics;
	AirborneApplication airborne_app;
	TrajectoryFromFile precalc_traj;

private:
	bool model_loaded;
	bool finished;
};
