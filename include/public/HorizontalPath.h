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
#include <string>
#include "public/HorizontalTurnPath.h"

class HorizontalPath
{
public:
	HorizontalPath(void);
	~HorizontalPath(void);
	bool operator==(const HorizontalPath &that) const;

	double x; // in meters
	double y; // in meters
	std::string segment;
	double L; // leg length in meters
	double course;
	HorizontalTurnPath turns;
};

