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

#include <string>
#include "Loadable.h"
#include "DecodedStream.h" // this is needed for Loadable 
#include "Waypoint.h"
#include <fstream>
#include "constants.h"

class AircraftIntent : public Loadable
{
public:
	AircraftIntent(void);
	~AircraftIntent(void);

	// load method inherited from Loadable
	bool load(DecodedStream *input);

	// ------------------------------------------------------
	// stuff for loading 

	void load_waypoints_from_list(list<Waypoint> &waypoint_list); // call this after the laod to finish things up 

	//Other Data:
	int id;
	int number_of_waypoints;
	double machTransitionCas;
	std::string waypoint_name[MAX_NUM_WAYPOINTS];
	double		waypoint_Lat[MAX_NUM_WAYPOINTS]; // radians
	double		waypoint_Lon[MAX_NUM_WAYPOINTS]; // radians
	double		waypoint_y[MAX_NUM_WAYPOINTS]; // meters
	double		waypoint_x[MAX_NUM_WAYPOINTS]; // meters
	double		waypoint_Alt[MAX_NUM_WAYPOINTS]; // meters
	double		waypoint_Descent_angle_degree[MAX_NUM_WAYPOINTS]; //descent_angle (degrees)
	double		nominal_IAS_at_waypoint[MAX_NUM_WAYPOINTS]; // feet/second
	double		MACH_at_waypoint[MAX_NUM_WAYPOINTS];
	double		waypoint_Descent_rate_knot_per_second[MAX_NUM_WAYPOINTS];

	// scenario parameters.

	// ------------------------------------------------------

	struct
		{		
		std::string Name[128];
		double xWp[128]; // meters
		double yWp[128]; // meters
		double LatWp[128]; // radians
		double LonWp[128]; // radians
		double AltWp[128]; // meters
		int number_of_waypoints;
		double nominal_IAS_at_waypoint[128]; // feet/second
		double MACH_at_waypoint[128];
		double altHi[128]; // meters
		double altLow[128]; // meters
		double speedHi[128]; // meters/second
		double speedLow[128]; // meters/second
		} Fms;

protected:
		void copy(const AircraftIntent &in); // helper method for copy constructor and assignment operator
	

};
