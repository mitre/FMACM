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

#include <cstdio>
#include "public/AchieveObserver.h"

using namespace std;

AchieveObserver::AchieveObserver(void) {
	// Default constructor.

	iteration = -1;
	id = -1;
	time = Units::SecondsTime(-99999.0);
	targTtgToAch = Units::SecondsTime(-99999.0);
	ownTtgToAch = Units::SecondsTime(-99999.0);
	currDist = Units::MetersLength(-99999.0);
	refDist = Units::MetersLength(-99999.0);
}


AchieveObserver::AchieveObserver(int iter,int aircraftId,double tm,double target_ttg_to_ach,double own_ttg_to_ach,double curr_distance,double reference_distance) {
	// Constructor to set values.
	//
	// iter:iteration.
	// aircraftId:aircraft id.
	// tm:time (seconds).
	// target_ttg_to_ach:target time to go to achieve (seconds).
	// own_ttg_to_ach:own time to go to achieve (seconds).
	// curr_distance:current distance (meters).
	// reference_distance:reference distance (meters).

	iteration = iter;
	id = aircraftId;
	time = Units::SecondsTime(tm);
	targTtgToAch = Units::SecondsTime(target_ttg_to_ach);
	ownTtgToAch = Units::SecondsTime(own_ttg_to_ach);
	currDist = Units::MetersLength(curr_distance);
	refDist = Units::MetersLength(reference_distance);
}


AchieveObserver::~AchieveObserver(void) {
	// Destructor.
}


string AchieveObserver::hdr() {
	// Creates output header for csv file output.
	//
	// returns header csv string.

	string str = "Iteration,AircrafId,Time(s),Targ_TTG_to_Ach(s),Own_TTG_to_Ach(s),CurrDistance(m),RefDistance(m)";

	return str;
}


string AchieveObserver::toString() {
	// Creates string of object for csv file output.
	//
	// returns data csv string.

	string str;

	char *txt = new char[301];

	sprintf(txt,"%d,%d,%lf,%lf,%lf,%lf,%lf",
		iteration,id,
		Units::SecondsTime(time).value(),
		Units::SecondsTime(targTtgToAch).value(),
		Units::SecondsTime(ownTtgToAch).value(),
		Units::MetersLength(currDist).value(),
		Units::MetersLength(refDist).value());

	str = txt;

	delete[] txt;

	return str;
}
