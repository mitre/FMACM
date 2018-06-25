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
#include "public/LoggingLoadable.h"
#include "public/PrecalcConstraint.h"
#include "UnsignedAngle.h"

class PrecalcWaypoint : public LoggingLoadable
{
public:
	PrecalcWaypoint(void);
	~PrecalcWaypoint(void);

	// equals operator
	bool operator==(const PrecalcWaypoint &obj) const;

	// load method to read in the Dynamics values
	bool load(DecodedStream *input);

	// method to check if the model loaded properly
	bool is_loaded(); 

	double leg_length; // in meters
	Units::UnsignedRadiansAngle course_angle; // in radians
	
	double x_pos; // in meters
	double y_pos; // in meters

    // Added for RF legs
    double x_cp; // center point for RF leg
    double y_cp;
    double radius_cp; // in meters

	// added for research
	Units::RadiansAngle bankAngle;
	Units::MetersPerSecondSpeed groundspeed;

	PrecalcConstraint constraints;

    void setLoaded(bool val) {loaded = val;};

private:
	bool loaded;
};

