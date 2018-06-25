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

#pragma once

#include "public/AircraftState.h"
#include "public/InternalObserver.h"
#include "public/AircraftIntent.h"
#include "public/PrecalcConstraint.h"
#include "public/PrecalcWaypoint.h"
#include "public/HorizontalPath.h"

// Data storage class for flight management systems
class TestFrameworkFMS
{
public:
	static Units::DegreesAngle MAX_BANK_ANGLE;

	TestFrameworkFMS(void);
	~TestFrameworkFMS(void);

	int Mode;
	double DeltaTrack;
	double TurnRadius;
	double RangeStartTurn;
	double RangeToNextWpM1;
	double RangeToNextWp;

	double xWp[128];
	double yWp[128];
	double AltWp[128];

	double Track[127];
	double psi[127];
	double Length[127];

	int number_of_waypoints;
	int NextWp;
	double nominal_IAS_at_waypoint[128];
	double MACH_at_waypoint[128];
	PrecalcConstraint constraints[128];

	// primary calculation method to update the FMS model
	void update(AircraftState state, std::vector<PrecalcWaypoint> &precalcWaypoints,
		    std::vector<HorizontalPath> &hTraj);

	// get the psi of the given index if in range, returns -999.9 if out of range
	double get_psi(int index); 

	// init method to initialize the FMS data
	void init();
	
	// method to read in the waypoint information from an AircraftIntent
	void copy_waypoints_from_intent(AircraftIntent intent_in);

	bool is_finished();

};


