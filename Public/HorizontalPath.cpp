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
// Copyright 2017 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#include "public/HorizontalPath.h"


HorizontalPath::HorizontalPath(void) : segment("")
{
	x = 0; // in meters
	y = 0; // in meters
	L = 0; // leg length in meters
	course = 0;
}

bool HorizontalPath::operator==(const HorizontalPath &that) const {
	return ((this->x == that.x) &&
			(this->y == that.y) &&
			(this->segment == that.segment) &&
			(this->L == that.L) &&
			(this->course == that.course) &&
			((this->segment == "straight") ||
			 (this->turns == that.turns)));
}


HorizontalPath::~HorizontalPath(void)
{
}
