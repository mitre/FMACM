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

#include "public/MergePointMetric.h"
#include "public/AircraftCalculations.h"
#include <stdexcept>
// #include <AchievePointCalcs.h>

using namespace std;
log4cplus::Logger MergePointMetric::logger = log4cplus::Logger::getInstance("MergePointMetric");

MergePointMetric::MergePointMetric(void) {
    mReportMetrics = false;
    mMergePointName = "";
    mIMDist = mMergeDist = Units::NauticalMilesLength(1000000000.0);
    mTargY = mTargX = mIMY = mIMX = 0;
    mMergePointY = mMergePointX = Units::ZERO_LENGTH;
}


MergePointMetric::~MergePointMetric(void) {
}

void MergePointMetric::determineMergePoint(const AircraftIntent &imintent, const AircraftIntent &targintent) {

    // Determines and sets the merge point between the IM and target aircraft.
    // The merge point is defined as the first waypoint in the route where the
    // remainder of the IM and target route match
    // If no merge point is found, this class will not report any metrics.
    //
    // imintent:intent of IM aircraft containing the list of waypoints.
    // targintent:intent of target aircraft containing the list of waypoints.

    std::pair<const int, const int> indices = imintent.findCommonWaypoint(targintent);
    const int index = indices.first;

    if (index == -1) {
        LOG4CPLUS_DEBUG(logger, "No merge point found for im acid " << imintent.getId() <<
                        " and target acid " << targintent.getId() << ". No merge metrics will be reported.");
        mReportMetrics = false; // do not report metrics...no merge point to report relative to
        mMergePointName = "";
    } else {
        mMergePointName = imintent.getWaypointName(index);
        mMergePointX = imintent.getWaypointX(index);
        mMergePointY = imintent.getWaypointY(index);
        mReportMetrics = true;
    }

}


void MergePointMetric::update(double imXNew, double imYNew, double targXNew, double targYNew) {

    // Replaces the current IM and target position if the new IM position is closer to the
    // merge point than the current IM point.  Distances are computed in nmi.
    //
    // imXNew,imYNew:the new IM position.
    // targXNew,targYNew:the  new target position.

    if (!mReportMetrics)
        return; // nothing to do

    if (newPointCloser(imXNew, imYNew)) {
        // Replace the current information with the new information.

        mIMX = imXNew;
        mIMY = imYNew;
        mIMDist = AircraftCalculations::ptToPtDist(
                mMergePointX,
                mMergePointY,
                Units::FeetLength(mIMX),
                Units::FeetLength(mIMY));

        mTargX = targXNew;
        mTargY = targYNew;
        mMergeDist = AircraftCalculations::ptToPtDist(
                Units::FeetLength(mIMX),
                Units::FeetLength(mIMY),
                Units::FeetLength(mTargX),
                Units::FeetLength(mTargY));
    }
}


string MergePointMetric::getMergePoint() {

    // Gets merge point.
    //
    // returns name of waypoint which is the merge point.

    return mMergePointName;
}


Units::Length MergePointMetric::getDist() {

    // Gets computed distance when IM aircraft was at merge point.
    //
    // returns distance in nmi.

    return mMergeDist;
}


bool MergePointMetric::newPointCloser(double x, double y) {

    // Checks if newest IM position closer to waypoint than the stored IM position.
    //
    // x,y:new IM position in feet.
    //
    // returns true if the new closer to the merge point.
    //         else false.

    return (AircraftCalculations::ptToPtDist(
            mMergePointX, mMergePointY,
            Units::FeetLength(x), Units::FeetLength(y))
            < mIMDist);
}


bool MergePointMetric::mergePointFound() {

    // Determines whether merge point already found.
    //
    // returns true if merge point found.
    //         else false

    return (mMergePointName.length() > 0);
}

void MergePointMetric::determineMetricsLocation(const AircraftIntent &imintent, const AircraftIntent &targintent, const string &waypointName) {

    bool foundWaypointIm = false;
    for (int i = 0; i < imintent.getNumberOfWaypoints(); ++i) {
        foundWaypointIm = imintent.getWaypointName(i).compare(waypointName) == 0;
        if (foundWaypointIm) {
            mMergePointName = imintent.getWaypointName(i);
            mMergePointX = imintent.getWaypointX(i);
            mMergePointY = imintent.getWaypointY(i);
            mReportMetrics = true;
            break;
        }
    }

    if (!foundWaypointIm) {
        LOG4CPLUS_DEBUG(logger, "Cannot find waypoint in aircraft intents. Merge Point Metrics will not be reported.");
        mReportMetrics = false;
        mMergePointName = "";
    }

}

bool MergePointMetric::willReportMetrics() const {
    return mReportMetrics;
}
