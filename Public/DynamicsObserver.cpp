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
// Copyright 2018 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#include "public/DynamicsObserver.h"

DynamicsObserver::DynamicsObserver(void)
{
	iter = -1;
	id = -1;
	time = -99999.0;
	achieved_groundspeed = 0.0;
	speed_command = 0.0;
	IAS_command = 0.0;
}

DynamicsObserver::~DynamicsObserver(void)
{
}

// < operator to enable sorting of Dynamic Observer objects
bool DynamicsObserver::operator<(const DynamicsObserver &dyn_in) const
{
	bool result = false;

	if( this->iter <= dyn_in.iter && this->id < dyn_in.id )
	{
		result = true;
	}

	return result;
}
