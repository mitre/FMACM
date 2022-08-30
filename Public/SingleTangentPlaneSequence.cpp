// ****************************************************************************
// NOTICE
//
// This work was produced for the U.S. Government under Contract 693KA8-22-C-00001 
// and is subject to Federal Aviation Administration Acquisition Management System 
// Clause 3.5-13, Rights In Data-General, Alt. III and Alt. IV (Oct. 1996).
//
// The contents of this document reflect the views of the author and The MITRE 
// Corporation and do not necessarily reflect the views of the Federal Aviation 
// Administration (FAA) or the Department of Transportation (DOT). Neither the FAA 
// nor the DOT makes any warranty or guarantee, expressed or implied, concerning 
// the content or accuracy of these views.
//
// For further information, please contact The MITRE Corporation, Contracts Management 
// Office, 7515 Colshire Drive, McLean, VA 22102-7539, (703) 983-6000.
//
// 2022 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#include "public/CoreUtils.h"
#include "public/SingleTangentPlaneSequence.h"

using namespace std;

std::list<Waypoint> SingleTangentPlaneSequence::m_master_waypoint_sequence = {};  // initialize as empty
log4cplus::Logger SingleTangentPlaneSequence::logger = log4cplus::Logger::getInstance("SingleTangentPlaneSequence");

SingleTangentPlaneSequence::SingleTangentPlaneSequence(const list<Waypoint> &waypoint_list) {
   initialize(waypoint_list);
}

void SingleTangentPlaneSequence::initialize(const std::list<Waypoint> &waypoint_list) {
   if (m_master_waypoint_sequence.empty()) {
      m_master_waypoint_sequence = waypoint_list;
   }

   TangentPlaneSequence::initialize(m_master_waypoint_sequence);

   return;
}

void SingleTangentPlaneSequence::clearStaticMembers() {
   m_master_waypoint_sequence.clear();
}

void SingleTangentPlaneSequence::dump(std::ofstream &file_out) const {
   std::list<Waypoint>::iterator wpt_iter;
   for (wpt_iter = m_master_waypoint_sequence.begin(); wpt_iter != m_master_waypoint_sequence.end(); ++wpt_iter) {
      file_out << "------" << std::endl;
      file_out << wpt_iter->GetName() << std::endl;
      file_out << "Arc Radius: " << Units::MetersLength(wpt_iter->GetRfTurnArcRadius()).value() << std::endl;
      file_out << "Center Lat: " << Units::DegreesAngle(wpt_iter->GetRfTurnCenterLatitude()).value() << std::endl;
      file_out << "Center Lon: " << Units::DegreesAngle(wpt_iter->GetRfTurnCenterLongitude()).value() << std::endl;
      file_out << "Mach: " << wpt_iter->GetMach() << std::endl;
      file_out << "IAS: " << Units::FeetPerSecondSpeed(wpt_iter->GetNominalIas()).value() << std::endl;
      file_out << "Waypoint lat: " << Units::RadiansAngle(wpt_iter->GetLatitude()).value() << std::endl;
      file_out << "Waypoint lon: " << Units::RadiansAngle(wpt_iter->GetLongitude()).value() << std::endl;
      file_out << "Waypoint alt: " << Units::MetersLength(wpt_iter->GetAltitude()).value() << std::endl;
      file_out << "alt hi: " << Units::MetersLength(wpt_iter->GetAltitudeConstraintHigh()).value() << std::endl;
      file_out << "alt low: " << Units::MetersLength(wpt_iter->GetAltitudeConstraintLow()).value() << std::endl;
      file_out << "speed hi: " << Units::MetersPerSecondSpeed(wpt_iter->GetSpeedConstraintHigh()).value() << std::endl;
      file_out << "speed low: " << Units::MetersPerSecondSpeed(wpt_iter->GetSpeedConstraintLow()).value() << std::endl;
   }
}
