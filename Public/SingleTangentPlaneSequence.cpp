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

#include "public/SingleTangentPlaneSequence.h"
using namespace std;

std::list<Waypoint> SingleTangentPlaneSequence::mMasterWaypointSequence = {};  // initialize as empty
log4cplus::Logger SingleTangentPlaneSequence::logger = log4cplus::Logger::getInstance("SingleTangentPlaneSequence");

SingleTangentPlaneSequence::SingleTangentPlaneSequence(list<Waypoint> &waypoint_list) {
    initialize(waypoint_list);
}

void SingleTangentPlaneSequence::initialize(std::list<Waypoint> &waypoint_list) {
    if (mMasterWaypointSequence.empty()) {
        LOG4CPLUS_DEBUG(logger, "initializing the first time");
        mMasterWaypointSequence = waypoint_list;
    }

    TangentPlaneSequence::initialize(mMasterWaypointSequence);

    return;
}

void SingleTangentPlaneSequence::clearStaticMembers() {
    mMasterWaypointSequence.clear();
}

void SingleTangentPlaneSequence::dump(std::ofstream& file_out) const
{
    std::list<Waypoint>::iterator wpt_iter;
    for (wpt_iter = mMasterWaypointSequence.begin(); wpt_iter != mMasterWaypointSequence.end(); ++wpt_iter) {
        file_out << "------" << std::endl;
        file_out << wpt_iter->getName() << std::endl;
        file_out << "Arc Radius: " << Units::MetersLength(wpt_iter->getRfTurnArcRadius()).value() << std::endl;
        file_out << "Center Lat: " << Units::DegreesAngle(wpt_iter->getRfTurnCenterLatitude()).value() << std::endl;
        file_out << "Center Lon: " << Units::DegreesAngle(wpt_iter->getRfTurnCenterLongitude()).value() << std::endl;
        file_out << "Mach: " << wpt_iter->getMach() << std::endl;
        file_out << "IAS: " << Units::FeetPerSecondSpeed(wpt_iter->getNominalIas()).value() << std::endl;
        file_out << "decent angle deg: " << Units::DegreesAngle(wpt_iter->getDescentAngle()).value() << std::endl;
        file_out << "descent rate kt/s: " << Units::KnotsPerSecondAcceleration(wpt_iter->getDescentRate()).value() << std::endl;
        file_out << "Waypoint lat: " << Units::RadiansAngle(wpt_iter->getLatitude()).value() << std::endl;
        file_out << "Waypoint lon: " << Units::RadiansAngle(wpt_iter->getLongitude()).value() << std::endl;
        file_out << "Waypoint alt: " << Units::MetersLength(wpt_iter->getAltitude()).value() << std::endl;
        file_out << "alt hi: " << Units::MetersLength(wpt_iter->getAltitudeConstraintHigh()).value() << std::endl;
        file_out << "alt low: " << Units::MetersLength(wpt_iter->getAltitudeConstraintLow()).value() << std::endl;
        file_out << "speed hi: " << Units::MetersPerSecondSpeed(wpt_iter->getSpeedConstraintHigh()).value() << std::endl;
        file_out << "speed low: " << Units::MetersPerSecondSpeed(wpt_iter->getSpeedConstraintLow()).value() << std::endl;
    }
}
