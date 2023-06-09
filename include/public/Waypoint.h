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

#pragma once

#include "public/LoggingLoadable.h"

class Waypoint : public LoggingLoadable {
  public:
   static const Units::FeetLength MAX_ALTITUDE_CONSTRAINT;
   static const Units::FeetLength MIN_ALTITUDE_CONSTRAINT;
   static const Units::FeetLength UNDEFINED_ALTITUDE_CONSTRAINT;
   static const Units::KnotsSpeed MAX_SPEED_CONSTRAINT;
   static const Units::KnotsSpeed MIN_SPEED_CONSTRAINT;
   static const Units::KnotsSpeed UNDEFINED_SPEED_CONSTRAINT;

   Waypoint();

   Waypoint(const std::string &name, Units::Angle latitude, Units::Angle longitude,
            Units::Length altitude_constraint_upper = UNDEFINED_ALTITUDE_CONSTRAINT,
            Units::Length altitude_constraint_lower = UNDEFINED_ALTITUDE_CONSTRAINT,
            Units::Speed speed_constraint = UNDEFINED_SPEED_CONSTRAINT,
            Units::Length nominal_altitude = Units::ZERO_LENGTH, Units::Speed nominal_ias = Units::ZERO_SPEED,
            std::string arinc424_leg_type = "");

   bool load(DecodedStream *input);

   void ProcessAltitudeConstraints(Units::Length altitude_upper, Units::Length altitude_lower);

   void ProcessSpeedConstraints(Units::Speed speed);

   const std::string &GetName() const;

   void SetName(const std::string name);

   Units::Angle GetLatitude() const;

   void SetLatitude(const Units::Angle &latitude);

   Units::Angle GetLongitude() const;

   void SetLongitude(const Units::Angle &longitude);

   void SetWaypointLatLon(const Units::Angle &latitude, const Units::Angle &longitude);

   Units::Length GetAltitude() const;

   void SetAltitude(const Units::Length &nominal_altitude);

   void SetNominalIas(const Units::Speed &nominal_ias);

   Units::Speed GetNominalIas() const;

   void SetMach(double mach);

   double GetMach() const;

   void SetAltitudeConstraintHigh(const Units::Length &altitude_high);

   Units::Length GetAltitudeConstraintHigh() const;

   void SetAltitudeConstraintLow(const Units::Length &altitude_low);

   Units::Length GetAltitudeConstraintLow() const;

   void SetSpeedConstraintHigh(const Units::Speed &speed_high);

   Units::Speed GetSpeedConstraintHigh() const;

   void SetSpeedConstraintLow(const Units::Speed &speed_low);

   Units::Speed GetSpeedConstraintLow() const;

   void SetRfTurnCenterLatitude(const Units::Angle &rf_turn_center_latitude);

   Units::Angle GetRfTurnCenterLatitude() const;

   void SetRfTurnCenterLongitude(const Units::Angle &rf_turn_center_longitude);

   Units::Angle GetRfTurnCenterLongitude() const;

   void SetRfTurnArcRadius(const Units::Length &rf_turn_radius);

   Units::Length GetRfTurnArcRadius() const;

   std::string GetArinc424LegType() const;

  private:
   std::string m_name;
   Units::Angle m_latitude;
   Units::Angle m_longitude;
   Units::Length m_altitude;
   Units::Speed m_nominal_ias;
   double m_mach;
   Units::Length m_altitude_constraint_high;
   Units::Length m_altitude_constraint_low;
   Units::Speed m_speed_constraint_high;
   Units::Speed m_speed_constraint_low;
   Units::Angle m_rf_turn_center_latitude;
   Units::Angle m_rf_turn_center_longitude;
   Units::Length m_rf_turn_arc_radius;
   std::string m_arinc424_leg_type;
};

inline const std::string &Waypoint::GetName() const { return m_name; }

inline void Waypoint::SetName(const std::string name) { m_name.assign(name); }

inline Units::Angle Waypoint::GetLatitude() const { return m_latitude; }

inline void Waypoint::SetLatitude(const Units::Angle &latitude) { m_latitude = latitude; }

inline Units::Angle Waypoint::GetLongitude() const { return m_longitude; }

inline void Waypoint::SetLongitude(const Units::Angle &longitude) { m_longitude = longitude; }

inline void Waypoint::SetWaypointLatLon(const Units::Angle &latitude, const Units::Angle &longitude) {
   SetLatitude(latitude);
   SetLongitude(longitude);
}

inline Units::Length Waypoint::GetAltitude() const { return m_altitude; }

inline void Waypoint::SetAltitude(const Units::Length &nominal_altitude) { m_altitude = nominal_altitude; }

inline Units::Speed Waypoint::GetNominalIas() const { return m_nominal_ias; }

inline void Waypoint::SetNominalIas(const Units::Speed &nominal_ias) { m_nominal_ias = nominal_ias; }

inline double Waypoint::GetMach() const { return m_mach; }

inline void Waypoint::SetMach(double mach) { m_mach = mach; }

inline void Waypoint::SetAltitudeConstraintHigh(const Units::Length &altitude_high) {
   m_altitude_constraint_high = altitude_high;
}

inline Units::Length Waypoint::GetAltitudeConstraintHigh() const { return m_altitude_constraint_high; }

inline void Waypoint::SetAltitudeConstraintLow(const Units::Length &altitude_low) {
   m_altitude_constraint_low = altitude_low;
}

inline Units::Length Waypoint::GetAltitudeConstraintLow() const { return m_altitude_constraint_low; }

inline void Waypoint::SetSpeedConstraintHigh(const Units::Speed &speed_high) { m_speed_constraint_high = speed_high; }

inline Units::Speed Waypoint::GetSpeedConstraintHigh() const { return m_speed_constraint_high; }

inline void Waypoint::SetSpeedConstraintLow(const Units::Speed &speed_low) { m_speed_constraint_low = speed_low; }

inline Units::Speed Waypoint::GetSpeedConstraintLow() const { return m_speed_constraint_low; }

inline Units::Angle Waypoint::GetRfTurnCenterLatitude() const { return m_rf_turn_center_latitude; }

inline void Waypoint::SetRfTurnCenterLatitude(const Units::Angle &rf_turn_center_latitude) {
   m_rf_turn_center_latitude = rf_turn_center_latitude;
}

inline Units::Angle Waypoint::GetRfTurnCenterLongitude() const { return m_rf_turn_center_longitude; }

inline void Waypoint::SetRfTurnCenterLongitude(const Units::Angle &rf_turn_center_longitude) {
   m_rf_turn_center_longitude = rf_turn_center_longitude;
}

inline Units::Length Waypoint::GetRfTurnArcRadius() const { return m_rf_turn_arc_radius; }

inline void Waypoint::SetRfTurnArcRadius(const Units::Length &rf_turn_radius) { m_rf_turn_arc_radius = rf_turn_radius; }

inline std::string Waypoint::GetArinc424LegType() const { return m_arinc424_leg_type; }

std::ostream &operator<<(std::ostream &out, const Waypoint &waypoint);

std::ostream &operator<<(std::ostream &out, const std::list<Waypoint> &waypoints);
