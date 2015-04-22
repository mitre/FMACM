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

#include "Guidance.h"

Guidance::Guidance(void)
{
	indicated_airspeed = 0;
	heading = 0;
	reference_altitude = 0;
	altitude_rate = 0; 
	psi = 0;
	cross_track = 0; 
	use_cross_track = false; 
	level = true;
	setValid(true);
}

Guidance::~Guidance(void)
{
}

void Guidance::setValid(bool validIn)
{
	valid = validIn;
}

bool Guidance::is_valid()
{
	return valid;
}
