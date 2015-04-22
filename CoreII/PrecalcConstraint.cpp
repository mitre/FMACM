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

#include "PrecalcConstraint.h"


PrecalcConstraint::PrecalcConstraint(void)
{
	active_flag = 0;
	constraint_dist = 0.0; // distance constraints-meters.
	constraint_altHi = 0.0; // altitude max constraints-meters.
	constraint_altLow = 0.0; // altitude min constraints-meters.
	constraint_speedHi = 0.0; // speed max constraint-meters per second.
	constraint_speedLow = 0.0; // speed min constraint-meters per second.
	index = -1;
	violation_flag = false;
}


PrecalcConstraint::~PrecalcConstraint(void)
{
}

bool PrecalcConstraint::operator<(const PrecalcConstraint &in)
{
	bool result = false;

	if( this->constraint_dist < in.constraint_dist )
	{
		result = true;
	}

	return result;
}
