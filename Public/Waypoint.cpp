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
// Copyright 2020 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#include "public/Waypoint.h"

const Units::FeetLength Waypoint::MAX_ALTITUDE_CONSTRAINT(50000);
const Units::FeetLength Waypoint::MIN_ALTITUDE_CONSTRAINT(0);
const Units::FeetLength Waypoint::UNDEFINED_ALTITUDE_CONSTRAINT(-10000);

const Units::KnotsSpeed Waypoint::MAX_SPEED_CONSTRAINT(1000);
const Units::KnotsSpeed Waypoint::MIN_SPEED_CONSTRAINT(0);
const Units::KnotsSpeed Waypoint::UNDEFINED_SPEED_CONSTRAINT(-999);

Waypoint::Waypoint() {
   m_name = "";
   m_latitude = Units::ZERO_ANGLE;
   m_longitude = Units::ZERO_ANGLE;
   m_descent_angle = Units::ZERO_ANGLE;
   m_altitude = Units::ZERO_LENGTH;
   m_nominal_ias = Units::ZERO_SPEED;
   m_mach = 0;
   m_descent_rate = Units::Acceleration(0);

   m_altitude_constraint_high = Units::ZERO_LENGTH;
   m_altitude_constraint_low = Units::ZERO_LENGTH;
   m_speed_constraint_high = Units::ZERO_SPEED;
   m_speed_constraint_low = Units::ZERO_SPEED;

   m_rf_turn_center_latitude = Units::ZERO_ANGLE;
   m_rf_turn_center_longitude = Units::ZERO_ANGLE;
   m_rf_turn_arc_radius = Units::ZERO_LENGTH;
}

Waypoint::Waypoint(const std::string &name_in,
                   Units::Angle latitude_in,
                   Units::Angle longitude_in,
                   Units::Length altitude_constraint_in,
                   Units::Length altitude_constraint_2_in,
                   Units::Speed speed_constraint_in,
                   Units::Length altitude_in,
                   Units::Speed nominal_ias_in) {
   m_name = name_in;
   m_latitude = latitude_in;
   m_longitude = longitude_in;
   m_altitude = altitude_in;
   m_nominal_ias = nominal_ias_in;

   if (std::isnan(Units::MetersLength(altitude_constraint_in).value())) {
      altitude_constraint_in = UNDEFINED_ALTITUDE_CONSTRAINT;
   }
   if (std::isnan(Units::MetersLength(altitude_constraint_2_in).value())) {
      altitude_constraint_2_in = UNDEFINED_ALTITUDE_CONSTRAINT;
   }
   if (std::isnan(Units::KnotsSpeed(speed_constraint_in).value())) {
      speed_constraint_in = UNDEFINED_SPEED_CONSTRAINT;
   }

   ProcessAltitudeConstraints(altitude_constraint_in,
                              altitude_constraint_2_in);

   ProcessSpeedConstraints(speed_constraint_in);

   m_rf_turn_center_latitude = Units::ZERO_ANGLE;
   m_rf_turn_center_longitude = Units::ZERO_ANGLE;
   m_rf_turn_arc_radius = Units::ZERO_LENGTH;

   m_mach = 0;
   m_descent_rate = Units::Acceleration(0);
   m_descent_angle = Units::ZERO_ANGLE;
}

Waypoint::~Waypoint() {
}

void Waypoint::ProcessAltitudeConstraints(Units::Length altitude_in,
                                          Units::Length altitude_2_in) {
   if (altitude_in == UNDEFINED_ALTITUDE_CONSTRAINT && altitude_2_in == UNDEFINED_ALTITUDE_CONSTRAINT) {
      m_altitude_constraint_high = MAX_ALTITUDE_CONSTRAINT;
      m_altitude_constraint_low = MIN_ALTITUDE_CONSTRAINT;
   } else if (altitude_in == UNDEFINED_ALTITUDE_CONSTRAINT || altitude_2_in == UNDEFINED_ALTITUDE_CONSTRAINT) {
      if (altitude_in > altitude_2_in) {
         m_altitude_constraint_high = altitude_in;
      } else {
         m_altitude_constraint_high = altitude_2_in;
      }
      m_altitude_constraint_low = MIN_ALTITUDE_CONSTRAINT;
   } else {
      if (altitude_in > altitude_2_in) {
         m_altitude_constraint_high = altitude_in;
         m_altitude_constraint_low = altitude_2_in;

      } else {
         m_altitude_constraint_high = altitude_2_in;
         m_altitude_constraint_low = altitude_in;
      }
   }
}

void Waypoint::ProcessSpeedConstraints(Units::Speed speed_in) {
   if (speed_in == UNDEFINED_SPEED_CONSTRAINT) {
      m_speed_constraint_high = MAX_SPEED_CONSTRAINT;
      m_speed_constraint_low = MIN_SPEED_CONSTRAINT;
   } else {
      m_speed_constraint_high = speed_in;
      m_speed_constraint_low = speed_in;
   }
}

bool Waypoint::load(DecodedStream *input) {
   set_stream(input);

   bool f = load_datum(m_name);

   if (!f) {
      LoggingLoadable::report_error("could not load waypoint_name");
   }

   //----------------------------------------------------

   //double lat_in_deg;

   f = loadAngleDegrees(m_latitude);

   if (!f) {
      LoggingLoadable::report_error("could not load waypoint_Latitude");
   }

   // waypoint_Lat = lat_in_deg * DTORAD;

   //----------------------------------------------------

   //double lon_in_deg;

   f = loadAngleDegrees(m_longitude);

   if (!f) {
      LoggingLoadable::report_error("could not load waypoint_Longitude");
   }

   //waypoint_Lon = lon_in_deg * DTORAD;

   //----------------------------------------------------
   f = loadLengthFeet(m_altitude);

   if (!f) {
      LoggingLoadable::report_error("could not load waypoint_altitude");
   }

   //----------------------------------------------------
   f = loadAngleDegrees(m_descent_angle);

   if (!f) {
      LoggingLoadable::report_error("could not load waypoint_decent_angle");
   }

   //----------------------------------------------------

   //double nominal_IAS_in_knot;

   f = loadSpeedKnots(m_nominal_ias);


   if (!f) {
      LoggingLoadable::report_error("could not load waypoint_nominal_IAS");
   }

   //nominal_IAS_at_waypoint = nominal_IAS_in_knot * KT2FPS;

   //----------------------------------------------------
   f = load_datum(m_mach);

   if (!f) {
      LoggingLoadable::report_error("could not load waypoint_MACH");
   }

   //----------------------------------------------------
   f = loadAccelerationKnotsPerSecond(m_descent_rate);

   if (!f) {
      LoggingLoadable::report_error("could not load waypoint_decent_rate");
   }

   f = loadLengthFeet(m_altitude_constraint_high);

   if (!f) {
      input->push_back();
      m_altitude_constraint_high = Units::FeetLength(50000);
   }

   // altHi *= FT_M;

   f = loadLengthFeet(m_altitude_constraint_low);

   if (!f) {
      input->push_back();
      m_altitude_constraint_low = Units::FeetLength(0);
   }

   //altLow *= FT_M;

   f = loadSpeedKnots(m_speed_constraint_high);

   if (!f) {
      input->push_back();
      m_speed_constraint_high = Units::KnotsSpeed(1000);
   }

   //speedHi *= KTS_MPS;

   f = loadSpeedKnots(m_speed_constraint_low);

   if (!f) {
      input->push_back();
      m_speed_constraint_low = Units::KnotsSpeed(0);
   }

   //speedLow *= KTS_MPS;

   //----------------------------------------------------
   // double raius_in_nm
   f = loadLengthNM(m_rf_turn_arc_radius);

   if (!f) {
      input->push_back();
      m_rf_turn_arc_radius = Units::NauticalMilesLength(0);
   }

   //----------------------------------------------------
   //double lat_in_deg;

   f = loadAngleDegrees(m_rf_turn_center_latitude);

   if (!f) {
      input->push_back();
      m_rf_turn_center_latitude = Units::RadiansAngle(0);
   }

   //----------------------------------------------------
   //double lon_in_deg;

   f = loadAngleDegrees(m_rf_turn_center_longitude);

   if (!f) {
      input->push_back();
      m_rf_turn_center_longitude = Units::RadiansAngle(0);
   }

   return true;
}


std::ostream &operator<<(std::ostream &out,
                         const Waypoint &waypoint) {
   out << waypoint.GetName() << " ";
   out << Units::DegreesAngle(waypoint.GetLatitude()).value() << " ";
   out << Units::DegreesAngle(waypoint.GetLongitude()).value() << " ";
   out << Units::FeetLength(waypoint.GetAltitude()).value() << " ";
   out << Units::DegreesAngle(waypoint.GetDescentAngle()).value() << " ";
   out << Units::KnotsSpeed(waypoint.GetNominalIas()).value() << " ";
   out << waypoint.GetMach() << " ";
   out << Units::KnotsPerSecondAcceleration(waypoint.GetDescentRate()).value() << " ";
   out << Units::FeetLength(waypoint.GetAltitudeConstraintHigh()).value() << " ";
   out << Units::FeetLength(waypoint.GetAltitudeConstraintLow()).value() << " ";
   out << Units::KnotsSpeed(waypoint.GetSpeedConstraintHigh()).value() << " ";
   out << Units::KnotsSpeed(waypoint.GetSpeedConstraintLow()).value() << " ";
   out << Units::NauticalMilesLength(waypoint.GetRfTurnArcRadius()).value() << " ";
   out << Units::DegreesAngle(waypoint.GetRfTurnCenterLatitude()).value() << " ";
   out << Units::DegreesAngle(waypoint.GetRfTurnCenterLongitude()).value() << " ";
   out << std::endl;
   return out;
}

std::ostream &operator<<(std::ostream &out,
                         const std::list<Waypoint> &waypoints) {
   for (std::list<Waypoint>::const_iterator i = waypoints.begin(); i != waypoints.end(); ++i) {
      out << *i;
   }
   return out;
}
