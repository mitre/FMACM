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

#include "public/PrecalcWaypoint.h"

PrecalcWaypoint::PrecalcWaypoint() {
   m_name = "noname";
   m_leg_length = Units::zero();
   m_course_angle = Units::DegreesAngle(0.0);

   m_x_pos_meters = Units::MetersLength(0);
   m_y_pos_meters = Units::MetersLength(0);

   m_rf_leg_center_x = Units::MetersLength(0);
   m_rf_leg_center_y = Units::MetersLength(0);
   m_radius_rf_leg = Units::MetersLength(0);

   m_precalc_constraints.constraint_along_path_distance = Units::MetersLength(0.0);  // distance constraints
   m_precalc_constraints.constraint_altHi = Units::MetersLength(0.0);                // altitude max constraints
   m_precalc_constraints.constraint_altLow = Units::MetersLength(0.0);               // altitude min constraints

   m_bank_angle = Units::RadiansAngle(0);
   m_ground_speed = Units::MetersPerSecondSpeed(0.0);

   m_loaded = false;
}

PrecalcWaypoint::~PrecalcWaypoint() = default;

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

bool PrecalcWaypoint::load(DecodedStream *input) {
   set_stream(input);

   Units::NauticalMilesLength leg_length;
   bool f = load_datum(leg_length);
   if (!f) {
      LoggingLoadable::report_error("could not load leg length");
   }
   m_leg_length = leg_length;

   Units::DegreesAngle course_angle1;
   f = load_datum(course_angle1);
   if (!f) {
      LoggingLoadable::report_error("could not load m_path_course angle");
   }
   m_course_angle = course_angle1;

   Units::NauticalMilesLength path_distance;
   f = load_datum(path_distance);
   if (!f) {
      LoggingLoadable::report_error("could not load distance constraint");
   }
   m_precalc_constraints.constraint_along_path_distance = path_distance;

   Units::FeetLength in_altitude_high;
   f = load_datum(in_altitude_high);
   if (!f) {
      LoggingLoadable::report_error("could not load max altitude constraint");
   }
   m_precalc_constraints.constraint_altHi = in_altitude_high;

   Units::FeetLength in_altitude_low;
   f = load_datum(in_altitude_low);
   if (!f) {
      LoggingLoadable::report_error("could not load min altitude constraint");
   }
   m_precalc_constraints.constraint_altLow = in_altitude_low;

   m_loaded = true;

   return m_loaded;
}
