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

#include "PrecalcWaypoint.h"
#include "constants.h"


PrecalcWaypoint::PrecalcWaypoint(void)
{
	leg_length = 0.0; // in nautical miles
	course_angle = 0.0; // in degrees
	
	x_pos = 0.0; // in nautical miles
	y_pos = 0.0; // in nautical miles

	constraints.constraint_dist = 0.0; // distance constraints
	constraints.constraint_altHi = 0.0; // altitude max constraints
	constraints.constraint_altLow = 0.0; // altitude min constraints

	loaded = false;
}


PrecalcWaypoint::~PrecalcWaypoint(void)
{
}

// method to check if the model loaded properly
bool PrecalcWaypoint::is_loaded()
{
	return loaded;
}

// load method to read in the Dynamics values
bool PrecalcWaypoint::load(DecodedStream *input)
{
	set_stream(input);

	bool f = load_datum(leg_length); // loads in Nautical Miles
	if(!f)
	{
		Loadable::report_error("could not load leg length\n");
		exit(-32);
	}
	leg_length *= NM_M; // converts nautical miles to meters

	f = load_datum(course_angle);
	if(!f)
	{
		Loadable::report_error("could not load course angle\n");
		exit(-33);
	}
	course_angle *=  M_PI / 180; // loads in degrees

	f = load_datum(constraints.constraint_dist);
	if(!f)
	{
		Loadable::report_error("could not load distance constraint\n");
		exit(-34);
	}
	constraints.constraint_dist *= NM_M;

	f = load_datum(constraints.constraint_altHi);
	if(!f)
	{
		Loadable::report_error("could not load max altitude constraint\n");
		exit(-35);
	}
	constraints.constraint_altHi *= FT_M;

	f = load_datum(constraints.constraint_altLow);
	if(!f)
	{
		Loadable::report_error("could not load min altitude constraint\n");
		exit(-36);
	}
	constraints.constraint_altLow *= FT_M;

	loaded = true;

	return loaded;
}
