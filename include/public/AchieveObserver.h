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

#pragma once

#include <string>
#include <Length.h>
#include <Time.h>

// Holds data used to produce time to go metrics for the achieve by algorithms.

class AchieveObserver
{
public:
	AchieveObserver(void);
	AchieveObserver(int iter,int aircraftId,double tm,
					  double target_ttg_to_ach,double own_ttg_to_ach,
					  double curr_distance,double reference_distance);
	~AchieveObserver(void);
	std::string hdr();
	std::string toString();

private:
	int iteration;
	int id; // aircraft id
	Units::Time time;
	Units::Time targTtgToAch;
	Units::Time ownTtgToAch;
	Units::Length currDist;
	Units::Length refDist;
};
