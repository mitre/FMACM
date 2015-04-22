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

#include "AircraftIntent.h"
#include "AircraftCalculations.h"

AircraftIntent::AircraftIntent(void)
{
	id = 0; // hardcoding this to match TestFrameworkAircraft.cpp
	number_of_waypoints = 0;
}

AircraftIntent::~AircraftIntent(void)
{
}

void AircraftIntent::load_waypoints_from_list(list<Waypoint>& waypoint_list)
{
	int c = 0;
	list<Waypoint>::iterator i = waypoint_list.begin();
	list<Waypoint>::iterator e = waypoint_list.end();

	while(i != e)
	{
		waypoint_name[c] = (*i).get_waypoint_name();
		waypoint_Lat[c] = (*i).get_waypoint_Lat();
		waypoint_Lon[c] = (*i).get_waypoint_Lon();

		// waypoint_x and waypoint_y are set after loading.
	
		waypoint_Alt[c] = (*i).get_waypoint_Alt()*FT_M; 
		waypoint_Descent_angle_degree[c] = (*i).get_waypoint_Descent_angle_degree();
		nominal_IAS_at_waypoint[c] = (*i).get_nominal_IAS_at_waypoint();
		MACH_at_waypoint[c] = (*i).get_MACH_at_waypoint();
		waypoint_Descent_rate_knot_per_second[c] = (*i).get_waypoint_Descent_rate_knot_per_second();

		assert((*i).get_waypoint_name().size() < 16);

		Fms.Name[c] =  (*i).get_waypoint_name();

		// Fms.xWp and Fms.yWp are set after loading.

		Fms.AltWp[c] = (*i).get_waypoint_Alt()*FT_M;
		Fms.LatWp[c] = (*i).get_waypoint_Lat();
		Fms.LonWp[c] = (*i).get_waypoint_Lon();
		Fms.nominal_IAS_at_waypoint[c] = (*i).get_nominal_IAS_at_waypoint();
		Fms.MACH_at_waypoint[c] = (*i).get_MACH_at_waypoint();

		// the new constraint values
		Fms.altHi[c] = (*i).getAltHi();
		Fms.altLow[c] = (*i).getAltLow();
		Fms.speedHi[c] = (*i).getSpeedHi();
		Fms.speedLow[c] = (*i).getSpeedLow();

//------
		//gwang 2009-09: handle NASA ASTAR route (waypoint format)
		if(Fms.AltWp[c] == 0)
		{
			Fms.AltWp[c] = Fms.AltWp[c-1];
		}

		//end gwang 2009-09

		//gwang 2009-09: handle the NASA ASTAR route (waypoint format)
		//If Mach is zero, it means there is no restriction. 
		//It should be equal to the value of the Mach at the previous waypoint.
		//The logic is as follows:
		double mach = MACH_at_waypoint[c];
		double ias = nominal_IAS_at_waypoint[c]; //FPS
		if(mach != 0) //This is for most cases. IAS should be the IAS at mach-IAS transition.
		{
			//Fms.nominal_IAS_at_waypoint[c] = KT2FPS * machTransitionCas;
			Fms.nominal_IAS_at_waypoint[c] = machTransitionCas;
			Fms.MACH_at_waypoint[c] = MACH_at_waypoint[c];
		}
		else if(c >= 1 && Fms.MACH_at_waypoint[c-1] != 0
			&& ias == 0)
		{
			//Fms.nominal_IAS_at_waypoint[c] = KT2FPS * machTransitionCas;
			Fms.nominal_IAS_at_waypoint[c] = machTransitionCas;
			Fms.MACH_at_waypoint[c] = Fms.MACH_at_waypoint[c-1];
		}
		else if(c >= 1 && ias != 0)
		{
			Fms.nominal_IAS_at_waypoint[c] = ias;
			Fms.MACH_at_waypoint[c] = 0;
		}
		else if(c >= 1 && Fms.nominal_IAS_at_waypoint[c-1] != 0
			&& ias == 0)
		{
			Fms.nominal_IAS_at_waypoint[c] = Fms.nominal_IAS_at_waypoint[c-1];
			Fms.MACH_at_waypoint[c] = 0;
		}
		else
		{
			Fms.nominal_IAS_at_waypoint[c] = ias;
			Fms.MACH_at_waypoint[c] = mach;
		}
//------
		i++;
		c++;
	}

	number_of_waypoints = waypoint_list.size();
	Fms.number_of_waypoints = number_of_waypoints;


	//Fms.MachTransitionCas_FPS = machTransitionCas * NM2FT;
	

}

// helper method for copy constructor and assignment operator
void AircraftIntent::copy(const AircraftIntent &in)
{
	this->id = in.id;
	this->number_of_waypoints = in.number_of_waypoints;

	// for loop to copy all waypoint information
	for( int loop = 0; loop < MAX_NUM_WAYPOINTS; loop++)
	{
		this->waypoint_name[loop] = in.waypoint_name[loop];
		this->waypoint_Lat[loop] = in.waypoint_Lat[loop];
		this->waypoint_Lon[loop] = in.waypoint_Lon[loop];
		this->waypoint_y[loop] = in.waypoint_y[loop]; 
		this->waypoint_x[loop] = in.waypoint_x[loop];
		this->waypoint_Alt[loop] = in.waypoint_Alt[loop];
		this->waypoint_Descent_angle_degree[loop] = in.waypoint_Descent_angle_degree[loop];
		this->nominal_IAS_at_waypoint[loop] = in.nominal_IAS_at_waypoint[loop];
		this->MACH_at_waypoint[loop] = in.MACH_at_waypoint[loop];
		this->waypoint_Descent_rate_knot_per_second[loop] = in.waypoint_Descent_rate_knot_per_second[loop];
	}
	this->machTransitionCas = in.machTransitionCas;

	// loop to copy FMS values, 128 is hardcoded size of arrays in FMS
	for(int loop2 = 0; loop2 < 128; loop2++)
	{
		this->Fms.Name[loop2] = in.Fms.Name[loop2];
		this->Fms.xWp[loop2] = in.Fms.xWp[loop2];
		this->Fms.yWp[loop2] = in.Fms.yWp[loop2];
		this->Fms.LatWp[loop2] = in.Fms.LatWp[loop2];
		this->Fms.LonWp[loop2] = in.Fms.LonWp[loop2];
		this->Fms.AltWp[loop2] = in.Fms.AltWp[loop2];
		this->Fms.nominal_IAS_at_waypoint[loop2] = in.Fms.nominal_IAS_at_waypoint[loop2];
		this->Fms.MACH_at_waypoint[loop2] = in.Fms.MACH_at_waypoint[loop2];
		this->Fms.altHi[loop2] = in.Fms.altHi[loop2];
		this->Fms.altLow[loop2] = in.Fms.altLow[loop2];
		this->Fms.speedHi[loop2] = in.Fms.speedHi[loop2];
		this->Fms.speedLow[loop2] = in.Fms.speedLow[loop2];
	}

	this->Fms.number_of_waypoints = in.Fms.number_of_waypoints;
}

bool AircraftIntent::load(DecodedStream *input)
{
	bool results;
	list<Waypoint> waypoint_list;

	set_stream(input); 

	// register all the variables used by the Aircraft Intent
	register_var("number_of_waypoints",&number_of_waypoints);
	register_var("MachTransitionCas",&machTransitionCas); 

	// register the waypoint list
	register_named_list("waypoints",&waypoint_list, true);

	//do the actual reading:
	results = complete(); 

	// convert the input to Feet Per Second
	machTransitionCas = machTransitionCas * KT2FPS;

	// loads all the waypoint information from the list
	load_waypoints_from_list(waypoint_list);

	return results;
}

