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
#ifndef AAESIM_COREUTILS_H
#define AAESIM_COREUTILS_H

#include <vector>
#include <Time.h>
#include <Length.h>

#include "utility/Logging.h"
#include "public/AircraftState.h"
#include "public/HorizontalPath.h"

// Class containing utility functions used by the Core AAESim library.
class CoreUtils
{
public:
    static const int UNINITIALIZED_AIRCRAFT_ID;

    // Returns index using a value into a vector of similar values.
    static int nearestIndex(int startIx, double v, std::vector<double> &vVect);

    // Interpolates using a value into a value to compute an output value of a different kind.
    static double interpolate(int upperIx, double v, std::vector<double> &vVect, std::vector<double> &oVect);

    static const std::vector<AircraftState>::const_iterator findCrossingState(
            const std::vector<AircraftState> &aircraftstates, const std::pair<Units::Length, Units::Length> &xyLoc);

    static const Units::Length calculateEuclideanDistance(const std::pair<Units::Length, Units::Length> &xyLoc1,
                                                          const std::pair<Units::Length, Units::Length> &xyLoc2);

private:
    // Method to check index validity.
    static bool indexValid(int upperIx, double v, std::vector<double> &vVect);

    static log4cplus::Logger logger;

};

#endif
