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

#include "public/PrecalcWaypoint.h"


PrecalcWaypoint::PrecalcWaypoint(void) {
   name = "noname";
   leg_length = 0.0; // in nautical miles
   course_angle = Units::DegreesAngle(0.0); // in degrees

   x_pos = 0.0; // in nautical miles (.h file says meters)
   y_pos = 0.0; // in nautical miles (.h file says meters)

   x_cp = 0.0;
   y_cp = 0.0;
   radius_cp = 0.0;

   constraints.constraint_dist = 0.0; // distance constraints
   constraints.constraint_altHi = 0.0; // altitude max constraints
   constraints.constraint_altLow = 0.0; // altitude min constraints

   bankAngle = Units::RadiansAngle(0);
   groundspeed = Units::MetersPerSecondSpeed(0.0);

   loaded = false;
}

PrecalcWaypoint::~PrecalcWaypoint(void) {
}

// equals operator
bool PrecalcWaypoint::operator==(const PrecalcWaypoint &obj) const {
   bool match = (this->leg_length == obj.leg_length);
   match = match && (this->course_angle == obj.course_angle);
   match = match && (this->x_pos == obj.x_pos);
   match = match && (this->y_pos == obj.y_pos);
   match = match && (this->constraints == obj.constraints);
   match = match && (this->x_cp == obj.x_cp);
   match = match && (this->y_cp == obj.y_cp);
   match = match && (this->radius_cp == obj.radius_cp);

   return match;
}

// load method to read in the Dynamics values
bool PrecalcWaypoint::load(DecodedStream *input) {
   set_stream(input);

   bool f = load_datum(leg_length); // loads in Nautical Miles
   if (!f) {
      LoggingLoadable::report_error("could not load leg length");
   }
   leg_length *= NM_M; // converts nautical miles to meters

   // Historically, course_angle was loaded in degrees but then
   // converted in place to radians.
   Units::DegreesAngle course_angle1;
   f = load_datum(course_angle1);
   if (!f) {
      LoggingLoadable::report_error("could not load course angle");
   }
   course_angle = course_angle1;

   f = load_datum(constraints.constraint_dist);
   if (!f) {
      LoggingLoadable::report_error("could not load distance constraint");
   }
   constraints.constraint_dist *= NM_M;

   f = load_datum(constraints.constraint_altHi);
   if (!f) {
      LoggingLoadable::report_error("could not load max altitude constraint");
   }
   constraints.constraint_altHi *= FT_M;

   f = load_datum(constraints.constraint_altLow);
   if (!f) {
      LoggingLoadable::report_error("could not load min altitude constraint");
   }
   constraints.constraint_altLow *= FT_M;

   loaded = true;

   return loaded;
}

// method to check if the model loaded properly
bool PrecalcWaypoint::is_loaded() {
   return loaded;
}
