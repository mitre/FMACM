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

#include "Waypoint.h"
#include "constants.h"

Waypoint::Waypoint(void)
{
}

Waypoint::~Waypoint(void)
{
}


bool Waypoint::load(DecodedStream *input)
{
	set_stream(input);

	bool f = load_datum(waypoint_name);

	if(!f)
	{
		Loadable::report_error("could not load waypoint_name\n");
		exit(-23);
	}

	//----------------------------------------------------

	//double lat_in_deg;

	f = loadAngleDegrees(waypoint_Lat);
	
	if(!f)
	{
		Loadable::report_error("could not load waypoint_Latitude\n");
		exit(-24);
	}

	// waypoint_Lat = lat_in_deg * DTORAD;

	//----------------------------------------------------

	//double lon_in_deg;
	
	f = loadAngleDegrees(waypoint_Lon);
	
	if(!f)
	{
		Loadable::report_error("could not load waypoint_Longitude\n");
		exit(-25);
	}

	//waypoint_Lon = lon_in_deg * DTORAD;

	//----------------------------------------------------
	f = loadLengthFeet(waypoint_Alt);

	if(!f)
	{
		Loadable::report_error("could not load waypoint_altitude\n");
		exit(-26);
	}

	//----------------------------------------------------
	f = loadAngleDegrees(waypoint_Descent_angle);

	if(!f)
	{
		Loadable::report_error("could not load waypoint_decent_angle\n");
		exit(-27);
	} 

	//----------------------------------------------------
	
	//double nominal_IAS_in_knot;
	
	f = loadSpeedKnots(nominal_IAS_at_waypoint);


	if(!f)
	{
		Loadable::report_error("could not load waypoint_nominal_IAS\n");
		exit(-28);
	}

	//nominal_IAS_at_waypoint = nominal_IAS_in_knot * KT2FPS;

	//----------------------------------------------------
	f = load_datum(MACH_at_waypoint);

	if(!f)
	{
		Loadable::report_error("could not load waypoint_MACH\n");
		exit(-29);
	}

	//----------------------------------------------------
	f = loadAccelerationKnotsPerSecond(waypoint_Descent_rate);

	if(!f)
	{
		Loadable::report_error("could not load waypoint_decent_rate\n");
		exit(-30);
	}

	f = loadLengthFeet(altHi);
		
	if(!f)
	{
		input->push_back();
		altHi = Units::FeetLength(50000);
	}

	// altHi *= FT_M;

	f = loadLengthFeet(altLow);
		
	if(!f)
	{
		input->push_back();
		altLow = Units::FeetLength(0);
	}

	//altLow *= FT_M;

	f = loadSpeedKnots(speedHi);
		
	if(!f)
	{
		input->push_back();
		speedHi = Units::KnotsSpeed(1000);
	}

	//speedHi *= KTS_MPS;

	f = loadSpeedKnots(speedLow);

	if(!f)
	{
		input->push_back();
		speedLow = Units::KnotsSpeed(0);
	}

	//speedLow *= KTS_MPS;

	return true;
}
