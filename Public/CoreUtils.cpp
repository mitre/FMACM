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

#include <stdexcept>

#include "public/CoreUtils.h"
#include "public/Scenario.h"
#include "public/AircraftCalculations.h"
#include "public/SimulationTime.h"

using namespace std;


log4cplus::Logger CoreUtils::logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("CoreUtils"));

const int CoreUtils::UNINITIALIZED_AIRCRAFT_ID = Scenario::AIRCRAFT_ID_NOT_IN_MAP; // link to Core definition

int CoreUtils::nearestIndex(int startIx, double v, vector<double> &vVect)
{

    // Computes an upper index for a value within a vector of
    // values of the same kind in ascending values.  Search starts
    // on a pre-initialized upper index and depending on the input
    // values, the index is adjusted either upward or downwards.
    //
    // **NOTE 27 Sep 13:In the IM_* classes::update, the distance values
    // passed into this function start a high negative and go to positive.
    // To get correct results for these, they should be sent as -distance.
    //
    // startIx:upper index to start at.
    // v:value to search for.
    // vVector:vector of values.
    //
    // returns upper index bounding v within vVect if v lies between
    //           one index and another.  If v > vVect[vVect.size()-1]
    //         vVect.size() is returned.  If v < vVect[0] 0 is returned.

    int ix;

    if (v <= vVect[0])
    {
        // Below lowest value.

        ix = 0;
    }
    else if (v > vVect[vVect.size() - 2])
    {
        // Higher than highest value.

        ix = vVect.size() - 1;
    }
    else if (v < vVect[startIx])
    {
        // Adjust downward.

        ix = startIx;

        while (!CoreUtils::indexValid(ix,v,vVect) && (ix > 0))
            ix--;
    }
    else
    {
        // Adjust upward.

        ix = startIx;

        while (!CoreUtils::indexValid(ix,v,vVect) && (ix < (int) vVect.size()))
            ix++;
    }

    if (!CoreUtils::indexValid(ix,v,vVect))
    {
        LOG4CPLUS_WARN(CoreUtils::logger,"Invalid index computed " << endl);
    }

    return ix;
}


double CoreUtils::interpolate(int upperIx, double v, std::vector<double> &vVect, std::vector<double> &oVect)
{
    // Interpolates using a value to compute an output value.
    //
    // **NOTE 27 Sep 13:In the IM_* classes::update, the distance values
    // passed into this function start a high negative and go to positive.
    // To get correct results for these, they should be sent as -distance.
    //
    // upperIx:high index into vectors.
    // v:value to be interpolated with.
    // vVect:vector with data the same type as val, (ie:if val is a distance, vVect contains distances.
    //         this is the data we are interpolating from).
    // oVect:vector with data that we want to compute to, (ie:will be velocities if we want velocity output).
    //
    // returns interpolated output value.

    if (upperIx < 1 || upperIx >= vVect.size()) {
        char msg[200];
        sprintf(msg, "upperIx (%d) is not between 1 and %d", upperIx, static_cast<int>(vVect.size() - 1) );
        LOG4CPLUS_FATAL(logger, msg);
        throw out_of_range(msg);
    }

    double v2 = vVect[upperIx];
    double v1 = vVect[upperIx-1];
    double o2 = oVect[upperIx];
    double o1 = oVect[upperIx-1];

    if ((v-v1) * (v-v2) > 0) {
        char msg[200];
        sprintf(msg, "v (%lf) is not between %lf and %lf.", v, v1, v2);
        LOG4CPLUS_FATAL(logger, msg);
        throw domain_error(msg);
    }

    return ((o2-o1)/(v2-v1))*(v-v1)+o1;
}


bool CoreUtils::indexValid(int upperIx, double v, vector<double> &vVect)
{
    // Method to check index validity.
    //
    // upperIx:high index of value.
    // v:value.
    // vVect:corresponding vector of values for index.
    //
    // returns whether index valid or not.
    bool valid;

    if (upperIx == vVect.size() - 1)
    {       // High index: v > top vVect.

        valid = (v > vVect[vVect.size() - 2]);
    }
    else if (upperIx == 0)
    {
        // Low index: v < low vVect.

        valid = (v <= vVect[0]);
    }
    else
    {
        // Somewhere in the middle: vVect[ix-1] <= v <= vVect[ix]

        // Note:The v <= vVect[ix] case should only occur for ix == size()-1.
        // Even where it does, the interpolation should be correct.

        valid = ((vVect[upperIx - 1] <= v) &&
                 (v <= vVect[upperIx]));
    }

    return valid;
}

const Units::Length CoreUtils::calculateEuclideanDistance(const std::pair<Units::Length, Units::Length> &xyLoc1,
                                                          const std::pair<Units::Length, Units::Length> &xyLoc2)
{
    Units::Length xdiff = xyLoc1.first - xyLoc2.first;
    Units::Length ydiff = xyLoc1.second - xyLoc2.second;
    Units::Length eucldist = sqrt((xdiff*xdiff) + (ydiff*ydiff));
    return eucldist;
}

const double CoreUtils::limit(double value,
                              double low_limit,
                              double high_limit)
{
    return (value < low_limit ? low_limit : (value > high_limit ? high_limit : value));
}

const int CoreUtils::sign(double value)
{
    return (((value) == (0)) ?	0: (((value) > (0)) ? (1) : (-1)));
}
