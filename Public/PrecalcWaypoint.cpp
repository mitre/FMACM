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
// Copyright 2019 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#include "public/PrecalcWaypoint.h"


PrecalcWaypoint::PrecalcWaypoint() {
   m_name = "noname";
   m_leg_length_meters = 0.0;
   m_course_angle = Units::DegreesAngle(0.0);

   m_x_pos_meters = 0.0;
   m_y_pos_meters = 0.0;

   m_center_x_for_rf_leg = 0.0;
   m_center_y_for_rf_leg = 0.0;
   m_radius_rf_leg_meters = 0.0;

   m_precalc_constraints.constraint_dist = 0.0; // distance constraints
   m_precalc_constraints.constraint_altHi = 0.0; // altitude max constraints
   m_precalc_constraints.constraint_altLow = 0.0; // altitude min constraints

   m_bank_angle = Units::RadiansAngle(0);
   m_ground_speed = Units::MetersPerSecondSpeed(0.0);

   m_loaded = false;
}

bool PrecalcWaypoint::operator==(const PrecalcWaypoint &obj) const {
   bool match = (this->m_leg_length_meters == obj.m_leg_length_meters);
   match = match && (this->m_course_angle == obj.m_course_angle);
   match = match && (this->m_x_pos_meters == obj.m_x_pos_meters);
   match = match && (this->m_y_pos_meters == obj.m_y_pos_meters);
   match = match && (this->m_precalc_constraints == obj.m_precalc_constraints);
   match = match && (this->m_center_x_for_rf_leg == obj.m_center_x_for_rf_leg);
   match = match && (this->m_center_y_for_rf_leg == obj.m_center_y_for_rf_leg);
   match = match && (this->m_radius_rf_leg_meters == obj.m_radius_rf_leg_meters);

   return match;
}

bool PrecalcWaypoint::load(DecodedStream *input) {
   set_stream(input);

   bool f = load_datum(m_leg_length_meters); // loads in Nautical Miles
   if (!f) {
      LoggingLoadable::report_error("could not load leg length");
   }
   m_leg_length_meters *= NAUTICAL_MILES_TO_METERS; // converts nautical miles to meters

   // Historically, course_angle was loaded in degrees but then
   // converted in place to radians.
   Units::DegreesAngle course_angle1;
   f = load_datum(course_angle1);
   if (!f) {
      LoggingLoadable::report_error("could not load m_path_course angle");
   }
   m_course_angle = course_angle1;

   f = load_datum(m_precalc_constraints.constraint_dist);
   if (!f) {
      LoggingLoadable::report_error("could not load distance constraint");
   }
   m_precalc_constraints.constraint_dist *= NAUTICAL_MILES_TO_METERS;

   f = load_datum(m_precalc_constraints.constraint_altHi);
   if (!f) {
      LoggingLoadable::report_error("could not load max altitude constraint");
   }
   m_precalc_constraints.constraint_altHi *= FEET_TO_METERS;

   f = load_datum(m_precalc_constraints.constraint_altLow);
   if (!f) {
      LoggingLoadable::report_error("could not load min altitude constraint");
   }
   m_precalc_constraints.constraint_altLow *= FEET_TO_METERS;

   m_loaded = true;

   return m_loaded;
}

