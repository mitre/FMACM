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


// Aircraft data storage class, for the purpose of the various coversion methods the internal values are assumed to be in feet

class AircraftState
{
public:
	AircraftState(void);
	~AircraftState(void);
	
	AircraftState(const AircraftState &in);
	AircraftState& operator=(const AircraftState &in);

	bool is_turning();

	// operator < to allow sorting
	bool operator<(const AircraftState &in) const;

//Input Data:
	//none

	// position methods
	double get_X_in_Nautical_Miles(); // get x position relative to Tangency Point in nautical miles
	double get_Y_in_Nautical_Miles(); // get y position relative to Tangency Point in nautical miles

	// acceleration methods
	double get_acceleration_in_knots(); // gets the aircraft acceleration in knots
	double get_x_acceleration_in_knots(); // gets the aircraft x acceleration in knots
	double get_y_acceleration_in_knots(); // gets the aircraft y acceleration in knots

	// heading methods
	double get_heading(); // get the aircraft heading in radians, clockwise from North (mathmatical 90 degrees)
	double get_heading_in_degrees(); // get the aircraft heading in degrees, clockwise from North
	double get_heading_in_radians_mathmatical(); // gets the aircraft heading in radians, counter-clockwise from 0 degrees (mathmatical)

	// psi getter/setters
	void set_psi(double psi_in);
	double get_psi();

	// speed methods
	double get_speed_in_knots(); // gets the aircraft speed in knots
	double get_speed_in_FPS(); // gets the aircraft speed in feet per second

	void set_speed_in_Knots(double speed_knots); // sets aircraft speed in knots
	double get_groundspeed_in_FPS(); // gets the aircraft ground speed in feet per second
	double get_groundspeed_in_knots(); // gets the aircraft ground speed in knots


	// Lat_Long conversion method
	void get_Lat_Long(double &lat_out, double &long_out) const;

//Other Data:
	int id;
	double time;
	double x, y, z; //position (ft)
	double xd, yd, zd; //speed (ft/s)
	double xdd, ydd, zdd; //acceleration (f/s^2)
	double gamma; 


	// Additional variables required by sense & avoid algorithm server
	// 

	// Winds
	double wind_measurement_speed_kts; // unused
	double wind_measurement_direction_deg; // unused

	double Vwx, Vwy; // true wind direction meters/second
	double Vw_para, Vw_perp; // true wind factors meters/second

	double yaw;
	double yaw_rate;
	double pitch_attitude;
	double roll_attitude;
	double psi; // aircraft psi measured from east counter-clockwise

	double distToGo; // For state-model-output in meters.
};
