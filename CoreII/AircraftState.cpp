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

#include "AircraftState.h"
#include <math.h>
#include "constants.h"
#include "CustomMath.h"
#include "micros.h"

AircraftState::AircraftState(void) 
{
	id = -1;
	time = -1;
	x = 0; // assumed to be in feet
	y = 0; // assumed to be in feet	
	z = 0; // assumed to be in feet
	xd = 0; // assumed to be in FPS
	yd = 0; // assumed to be in FPS
	zd = 0;	// assumed to be in FPS
	xdd = 0; // assumed to be in FPSS
	ydd = 0; // assumed to be in FPSS
	zdd = 0; // assumed to be in FPSS


	gamma = 0.0;
	wind_measurement_speed_kts     = 0.0;
	wind_measurement_direction_deg = 0.0;
	yaw                            = 0.0;
	yaw_rate                       = 0.0;
	pitch_attitude                 = 0.0;
	roll_attitude                  = 0.0;
	psi							   = 0.0;

	Vwx = 0.0; // meters/second
	Vwy = 0.0; // meters/second

	Vw_para = 0.0; // meters/second
	Vw_perp = 0.0; // meters/second

	distToGo = -99999.99999; // meters
}
AircraftState::~AircraftState(void)
{
  
}

AircraftState::AircraftState (const AircraftState &in)
{
  id   = in.id;
  time = in.time;
  x    = in.x;
  y    = in.y;
  z    = in.z;
  xd   = in.xd;
  yd   = in.yd;
  zd   = in.zd;
  xdd  = in.xdd;
  ydd  = in.ydd;
  zdd  = in.zdd;


  gamma = in.gamma;
  wind_measurement_speed_kts     = in.wind_measurement_speed_kts;
  wind_measurement_direction_deg = in.wind_measurement_direction_deg;
  yaw                            = in.yaw;
  yaw_rate                       = in.yaw_rate;
  pitch_attitude                 = in.pitch_attitude;
  roll_attitude                  = in.roll_attitude;
  psi							 = in.psi;

  Vwx = in.Vwx;
  Vwy = in.Vwy;

  Vw_para = in.Vw_para;
  Vw_perp = in.Vw_perp;

  distToGo = in.distToGo;
}

AircraftState& AircraftState::operator= (const AircraftState &in)
{
  if (this != &in) {
    id   = in.id;
    time = in.time;
    x    = in.x;
    y    = in.y;
    z    = in.z;
    xd   = in.xd;
    yd   = in.yd;
    zd   = in.zd;
    xdd  = in.xdd;
    ydd  = in.ydd;
    zdd  = in.zdd;

	gamma = in.gamma;
    wind_measurement_speed_kts     = in.wind_measurement_speed_kts;
    wind_measurement_direction_deg = in.wind_measurement_direction_deg;
    yaw                            = in.yaw;
    yaw_rate                       = in.yaw_rate;
    pitch_attitude                 = in.pitch_attitude;
    roll_attitude                  = in.roll_attitude;
    psi							   = in.psi;

    Vwx = in.Vwx;
    Vwy = in.Vwy;

    Vw_para = in.Vw_para;
    Vw_perp = in.Vw_perp;

    distToGo = in.distToGo;
  }
  
  return *this;
}


bool AircraftState::is_turning()
{
	//Determine if a turn is taking place: source nav_NSE.cpp of WinSS

	double spd = sqrt(SQR(xd) + SQR(yd));			
	double turn_rate = (xd*ydd - yd*xdd) / spd;

	return ((ABS(turn_rate) > 1.5));
}

// operator < to allow sorting

bool AircraftState::operator<(const AircraftState &in) const
{
	bool result = false; // return value initialized to false

	// check if id is less, or if matching the time is less
	if( this->id < in.id || ( this->id == in.id && this->time < in.time ))
	{
		result = true; // sets return value to true
	}

	return result;
}


// Lat_Long conversion method
void AircraftState::get_Lat_Long(double &lat_out, double &long_out) const
{
//	StereographicProjection::xy_to_ll(x,y, &lat_out, &long_out); // call the Stereographic Projection to convert the aircraft X/Y to Lat/Long
}

// heading methods
// get the aircraft heading in radians, clockwise from North (mathmatical 90 degrees)
//gwang 2013-10: the function name should be get_ground_track, because xd and yd are ground speeds
double AircraftState::get_heading()
{
	double result = 0.0;

	result = atan3(xd, yd); // takes the atan of x/y instead of y/x to find the angle from North clockwise, the result is in radians

	return result;
}
// get the aircraft heading in degrees, clockwise from North
double AircraftState::get_heading_in_degrees()
{
	double result = 0.0;

	result = atan3(xd, yd); // takes the atan of x/y instead of y/x to find the angle from North clockwise, the result is in radians

	result *= RADTOD; // convert to degrees

	return result;
}
// gets the aircraft heading in radians, counter-clockwise from 0 degrees (mathmatical)
double AircraftState::get_heading_in_radians_mathmatical()
{
	double result = 0.0;

	result = atan3(yd, xd); // gets mathmatical 0 degrees counterclockwise position in radians

	return result;
}


// speed methods
// gets the aircraft speed in knots
double AircraftState::get_speed_in_knots()
{
	double result = 0.0;

	result = sqrt(SQR(xd) + SQR(yd)); // calculate the speed in feet per second
	result /= KT2FPS; // convert the speed value from feet per second to knots

	return result;
}
// gets the aircraft speed in feet per second
double AircraftState::get_speed_in_FPS()
{
	double result = 0.0;

	result = sqrt(SQR(xd) + SQR(yd)); // calculate the speed in feet per second

	return result;
}
// gets the aircraft ground speed in feet per second
double AircraftState::get_groundspeed_in_FPS()
{
	double result = 0.0;

	//gwang 2013-10:
	//double xdot = get_speed_in_FPS()*sin(get_heading())*cos(gamma); // calculates xdot from heading and descent angle
	//double ydot = get_speed_in_FPS()*cos(get_heading())*cos(gamma); // calculates ydot from heading and descent angle
	//result = sqrt(SQR(xdot) + SQR(ydot)); // calculate the ground speed value in FPS
	result = sqrt(SQR(xd) + SQR(yd)); // calculate the ground speed value in FPS
	//end gwang
	return result;
}
// gets the aircraft ground speed in knots
double AircraftState::get_groundspeed_in_knots()
{
	double result = 0.0;

	//gwang 2013-10
	//double xdot = get_speed_in_FPS()*sin(get_heading())*cos(gamma); // calculates xdot from heading and descent angle
	//double ydot = get_speed_in_FPS()*cos(get_heading())*cos(gamma); // calculates ydot from heading and descent angle
	//result = sqrt(SQR(xdot) + SQR(ydot)); // calculate the ground speed value in FPS
	//result /= KT2FPS; // convert to knots
	result = sqrt(SQR(xd) + SQR(yd))/KT2FPS; // calculate the ground speed value in FPS
	//end gwang

	return result;
}



// get position methods
// get x position relative to Tangency Point in nautical miles
double AircraftState::get_X_in_Nautical_Miles()
{
	double result = 0.0;

	result = x / NM2FT; // convert x value to nautical miles

	return result;
}
// get y position relative to Tangency Point in nautical miles
double AircraftState::get_Y_in_Nautical_Miles()
{
	double result = 0.0;

	result = y / NM2FT; // convert y value to nautical miles

	return result;
}

// acceleration methods
// gets the aircraft acceleration in knots
double AircraftState::get_acceleration_in_knots()
{
	double result = 0.0;

	result = sqrt(SQR(xdd) + SQR(ydd)); // calculate the acceleration in feet per second per second
	result /= KT2FPS; // converts the acceleration to knots per second

	return result;
}
// gets the aircraft x acceleration in knots
double AircraftState::get_x_acceleration_in_knots()
{
	double result = 0.0;

	result = xdd / KT2FPS; // convert x acceleration value to knots per second

	return result;
}
// gets the aircraft x acceleration in knots
double AircraftState::get_y_acceleration_in_knots()
{
	double result = 0.0;

	result = ydd / KT2FPS; // convert y acceleration value to knots per second

	return result;
}



// sets aircraft speed in knots
void AircraftState::set_speed_in_Knots(double speed_knots)
{
	// save current x and y velocities
	double curr_xd = xd;
	double curr_yd = yd;

	// set the new x and y velocity based on the current heading and convert to feet per second
	xd = sin(get_heading()) * speed_knots * KT2FPS;
	yd = cos(get_heading()) * speed_knots * KT2FPS; 
}

// psi getter/setters
void AircraftState::set_psi(double psi_in)
{
	psi = psi_in;
}
double AircraftState::get_psi()
{
	return psi;
}
