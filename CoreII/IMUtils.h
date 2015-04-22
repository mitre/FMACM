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

#include <vector>

// Class containing utility functions primarily used with the IM classes.

class IMUtils
{
public:

	// Returns index using a value into a vector of similar values.
	static int IM_index(int startIx, double v, std::vector<double> &vVect);

	// Interpolates using a value into a value to compute an output value of a different kind.
	static double interp(int upperIx, double v, std::vector<double> &vVect, std::vector<double> &oVect);

	// Method to check index validity.
	static bool indexValid(int upperIx, double v, std::vector<double> &vVect);
};
