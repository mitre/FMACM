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

class Waypoint :public Loadable
{
public:
	Waypoint(void);
	~Waypoint(void);

	bool load(DecodedStream *input);

	inline	string	get_waypoint_name()	
	{
		return 	waypoint_name;	
	}

	inline	double	get_waypoint_Lat()	
	{
		return	Units::RadiansAngle(waypoint_Lat).value();
	}

	inline	double	get_waypoint_Lon()	
	{
		return	Units::RadiansAngle(waypoint_Lon).value();
	}

	inline	double	get_waypoint_Alt()	
	{
		return	Units::FeetLength(waypoint_Alt).value();
	}

	inline	double get_waypoint_Descent_angle_degree()	
	{
		return	Units::DegreesAngle(waypoint_Descent_angle).value();
	}

	inline double get_nominal_IAS_at_waypoint()	
	{
	    // Returns IAS in FPS
		return	Units::FeetPerSecondSpeed(nominal_IAS_at_waypoint).value();
	}

	inline	double	get_MACH_at_waypoint()	
	{
		return MACH_at_waypoint;	
	}
	
	inline double get_waypoint_Descent_rate_knot_per_second()
	{
		return Units::KnotsPerSecondAcceleration(waypoint_Descent_rate).value();
	} 

	double getAltHi() const {
		return Units::MetersLength(altHi).value();
	}

	double getAltLow() const {
		return Units::MetersLength(altLow).value();
	}

	double getSpeedHi() const {
		return Units::MetersPerSecondSpeed(speedHi).value();
	}

	double getSpeedLow() const {
		return Units::MetersPerSecondSpeed(speedLow).value();
	}

private:
	string waypoint_name;
	Units::Angle waypoint_Lat;
	Units::Angle waypoint_Lon;
	Units::Length waypoint_Alt;	// feet
	Units::Angle waypoint_Descent_angle; //descent_angle (degrees)
	Units::Speed nominal_IAS_at_waypoint; // FPS
	double MACH_at_waypoint;
	Units::Acceleration waypoint_Descent_rate;

	// previously public data members which store the altitude and speed constraints
	// Getters are public.
	Units::Length altHi;	// orig. meters
	Units::Length altLow;	// orig. meters
	Units::Speed speedHi; // orig. meters/second
	Units::Speed speedLow; // orig. meters/second

};
