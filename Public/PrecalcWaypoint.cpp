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
// (c) 2026 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#include "public/PrecalcWaypoint.h"

bool PrecalcWaypoint::operator==(const PrecalcWaypoint &obj) const {
   bool match = (m_leg_length == obj.m_leg_length);
   match = match && (m_course_angle == obj.m_course_angle);
   match = match && (m_x_pos_meters == obj.m_x_pos_meters);
   match = match && (m_y_pos_meters == obj.m_y_pos_meters);
   match = match && (m_precalc_constraints == obj.m_precalc_constraints);
   match = match && (m_rf_leg_center_x == obj.m_rf_leg_center_x);
   match = match && (m_rf_leg_center_y == obj.m_rf_leg_center_y);
   match = match && (m_radius_rf_leg == obj.m_radius_rf_leg);

   return match;
}
