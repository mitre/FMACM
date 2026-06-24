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

#include "public/Waypoint.h"

#include <list>
#include <ostream>
#include <string>

const Units::FeetLength Waypoint::MAX_ALTITUDE_CONSTRAINT(50000);
const Units::FeetLength Waypoint::MIN_ALTITUDE_CONSTRAINT(0);
const Units::KnotsSpeed Waypoint::MAX_SPEED_CONSTRAINT(1000);
const Units::KnotsSpeed Waypoint::MIN_SPEED_CONSTRAINT(0);

Waypoint::Waypoint(const std::string &name, Units::Angle latitude, Units::Angle longitude,
                   Units::Length altitude_constraint_upper, Units::Length altitude_constraint_lower,
                   Units::Speed speed_constraint, Units::Length nominal_altitude, Units::Speed nominal_ias,
                   const std::string &arinc424_leg_type)
   : m_name(name),
     m_latitude(latitude),
     m_longitude(longitude),
     m_altitude(nominal_altitude),
     m_nominal_ias(nominal_ias),
     m_altitude_constraint_high(altitude_constraint_upper),
     m_altitude_constraint_low(altitude_constraint_lower),
     m_speed_constraint_high(speed_constraint),
     m_speed_constraint_low(MIN_SPEED_CONSTRAINT),
     m_rf_turn_center_latitude(Units::ZERO_ANGLE),
     m_rf_turn_center_longitude(Units::ZERO_ANGLE),
     m_rf_turn_arc_radius(Units::ZERO_LENGTH),
     m_arinc424_leg_type(arinc424_leg_type) {}

std::ostream &operator<<(std::ostream &out, const Waypoint &waypoint) {
   out << waypoint.GetName() << " ";
   out << Units::DegreesAngle(waypoint.GetLatitude()).value() << " ";
   out << Units::DegreesAngle(waypoint.GetLongitude()).value() << " ";
   out << Units::FeetLength(waypoint.GetAltitude()).value() << " ";
   out << Units::KnotsSpeed(waypoint.GetNominalIas()).value() << " ";
   out << Units::FeetLength(waypoint.GetAltitudeConstraintHigh()).value() << " ";
   out << Units::FeetLength(waypoint.GetAltitudeConstraintLow()).value() << " ";
   out << Units::KnotsSpeed(waypoint.GetSpeedConstraintHigh()).value() << " ";
   out << Units::KnotsSpeed(waypoint.GetSpeedConstraintLow()).value() << " ";
   out << Units::NauticalMilesLength(waypoint.GetRfTurnArcRadius()).value() << " ";
   out << Units::DegreesAngle(waypoint.GetRfTurnCenterLatitude()).value() << " ";
   out << Units::DegreesAngle(waypoint.GetRfTurnCenterLongitude()).value() << " ";
   out << waypoint.GetArinc424LegType() << " ";
   out << std::endl;
   return out;
}

std::ostream &operator<<(std::ostream &out, const std::list<Waypoint> &waypoints) {
   for (std::list<Waypoint>::const_iterator i = waypoints.begin(); i != waypoints.end(); ++i) {
      out << *i;
   }
   return out;
}
