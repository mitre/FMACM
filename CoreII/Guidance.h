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

// simple data storage class to store the 
class Guidance
{
public:
	Guidance(void);
	~Guidance(void);

	void setValid(bool inValid);
	bool is_valid();

	double indicated_airspeed; // in FPS
	double heading; // in radians
	double reference_altitude; // in feet
	double altitude_rate; // in FPS
	double psi; // measured from east counter-clockwise radians
	double cross_track; // measure the cross track error in meters
	bool use_cross_track; // flag to indicate if using the cross track error
	bool level; // flag to indicate if aircraft should be level or descending


private:

	bool valid; // Set to true if guidance is valid or rather it's data is valid.
				// 19 Sep 13 This initially setup to be used by Aircraft and set by the
				// particular models and returned through AirborneApplication::update.

};
