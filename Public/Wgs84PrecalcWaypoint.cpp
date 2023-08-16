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
// 2023 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#include "public/Wgs84PrecalcWaypoint.h"

using namespace aaesim;

Wgs84PrecalcWaypoint::Wgs84PrecalcWaypoint() {
   m_name = "noname";
   m_leg_type = AircraftIntent::Arinc424LegType::UNSET;
   m_leg_length = Units::zero();
   m_enu_course_in_angle = Units::DegreesAngle(0.0);
   m_enu_course_out_angle = Units::DegreesAngle(0.0);

   m_position = LatitudeLongitudePoint(Units::SignedRadiansAngle(0.0), Units::SignedRadiansAngle(0.0));

   m_rf_leg_center = LatitudeLongitudePoint(Units::SignedRadiansAngle(0.0), Units::SignedRadiansAngle(0.0));
   m_radius_rf_leg = Units::MetersLength(0);

   // m_precalc_constraints initialized by constructor

   m_bank_angle = Units::RadiansAngle(0);
   m_ground_speed = Units::MetersPerSecondSpeed(0.0);
}

bool Wgs84PrecalcWaypoint::operator==(const Wgs84PrecalcWaypoint &obj) const {

   bool match = (m_leg_length == obj.m_leg_length);
   match = match && (m_name == obj.m_name);
   match = match && (m_bank_angle == obj.m_bank_angle);
   match = match && (m_enu_course_in_angle == obj.m_enu_course_in_angle);
   match = match && (m_enu_course_out_angle == obj.m_enu_course_out_angle);
   match = match && (m_ground_speed == obj.m_ground_speed);
   match = match && (m_precalc_constraints == obj.m_precalc_constraints);
   match = match && (m_radius_rf_leg == obj.m_radius_rf_leg);
   match = match && (m_rf_leg_center == obj.m_rf_leg_center);
   match = match && (m_position == obj.m_position);
   match = match && (m_position == obj.m_position);
   match = match && (m_leg_type == obj.m_leg_type);

   return match;
}
