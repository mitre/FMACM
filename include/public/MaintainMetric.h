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

#pragma once

#include "math/Statistics.h"

#ifndef CYCLE_THRESHOLD
#define CYCLE_THRESHOLD 10
#endif


// Storage metrics class for statistics gathered during the maintain phase
// of the flight.

class MaintainMetric
{
public:
	MaintainMetric(void);
	~MaintainMetric(void);

	// Adds data to be added for each pass through an IM::update method.
	void addErr(double err);

	// Sets time aircraft went by achieve by point.
	void setAchieve(double time);

	// Boolean to determine if achieveBy set (achieveBy < 0.0)
	bool hasAchieve();

	// Computes total maintain time subtracting the achieveByTime
	// from the current time.
	void computeTotalMaintainTime(double cTime);

	// Gets mean spacing error.
	double getMeanErr();

	// Gets standard deviation of spacing error.
	double getStdErr();

	// Gets 95th bound of spacing error.
	double getBound95();

	// Gets total maintain time.
	double getTotMaintain();

	// Gets number of cycles with spacing errors > cycle threshold
	int getNumCycles();

	// Returns whether there are data samples to collect metrics from.
	bool hasSamples();

private:
	// Running sum of time spacing errors between IM and target ac.
	Statistics spacingError;

	// Time went by achieve by point.
	double achieveByTime;

	// Time spent in maintain stage, (current time - achieve by time)
	double totalMaintainTime;

	// Number of cycles with a spacing error > 10 secs.
	int numCyclesOutsideThreshold;
};
