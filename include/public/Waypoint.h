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

#pragma once

#include "public/LoggingLoadable.h"

class Waypoint : public LoggingLoadable
{
public:

   static const Units::FeetLength MAX_ALTITUDE_CONSTRAINT;
   static const Units::FeetLength MIN_ALTITUDE_CONSTRAINT;
   static const Units::FeetLength UNDEFINED_ALTITUDE_CONSTRAINT;
   static const Units::KnotsSpeed MAX_SPEED_CONSTRAINT;
   static const Units::KnotsSpeed MIN_SPEED_CONSTRAINT;
   static const Units::KnotsSpeed UNDEFINED_SPEED_CONSTRAINT;

   enum AltitudeConstraintType
   {
      ABOVE = 100,
      BELOW,
      AT,
      BETWEEN,
      NONE,
      INVALID
   };

   Waypoint();

   Waypoint(const std::string &name_in,
            Units::Angle latitude_in,
            Units::Angle longitude_in,
            Units::Length altitude_constraint_in = UNDEFINED_ALTITUDE_CONSTRAINT,
            Units::Length altitude_constraint_2_in = UNDEFINED_ALTITUDE_CONSTRAINT,
            Units::Speed speed_constraint_in = UNDEFINED_SPEED_CONSTRAINT,
            Units::Length altitude_in = Units::ZERO_LENGTH,
            Units::Speed nominal_ias_in = Units::ZERO_SPEED);

   virtual ~Waypoint();

   bool load(DecodedStream *input);

   void ProcessAltitudeConstraints(Units::Length altitude_in,
                                   Units::Length altitude_2_in);

   void ProcessSpeedConstraints(Units::Speed speed_in);

   const std::string &GetName() const;

   void SetName(const std::string name_in);

   Units::Angle GetLatitude() const;

   void SetLatitude(const Units::Angle &latitude_in);

   Units::Angle GetLongitude() const;

   void SetLongitude(const Units::Angle &longitude_in);

   void SetWaypointLatLon(const Units::Angle &latitude_in,
                          const Units::Angle &longitude_in);

   Units::Length GetAltitude() const;

   void SetAltitude(const Units::Length &altitude_in);

   Units::Angle GetDescentAngle() const;

   void SetNominalIas(const Units::Speed &nominal_ias_in);

   Units::Speed GetNominalIas() const;

   void SetMach(double mach);

   double GetMach() const;

   Units::Acceleration GetDescentRate() const;

   void SetAltitudeConstraintHigh(const Units::Length &altitude_high_in);

   Units::Length GetAltitudeConstraintHigh() const;

   void SetAltitudeConstraintLow(const Units::Length &altitude_low_in);

   Units::Length GetAltitudeConstraintLow() const;

   void SetSpeedConstraintHigh(const Units::Speed &speed_high_in);

   Units::Speed GetSpeedConstraintHigh() const;

   void SetSpeedConstraintLow(const Units::Speed &speed_low_in);

   Units::Speed GetSpeedConstraintLow() const;

   void SetRfTurnCenterLatitude(const Units::Angle &rf_turn_center_latitude_in);

   Units::Angle GetRfTurnCenterLatitude() const;

   void SetRfTurnCenterLongitude(const Units::Angle &rf_turn_center_longitude_in);

   Units::Angle GetRfTurnCenterLongitude() const;

   void SetRfTurnArcRadius(const Units::Length &rf_turn_radius_in);

   Units::Length GetRfTurnArcRadius() const;


private:
   std::string m_name;
   Units::Angle m_latitude;
   Units::Angle m_longitude;
   Units::Angle m_descent_angle;
   Units::Length m_altitude;
   Units::Speed m_nominal_ias;
   Units::Acceleration m_descent_rate;
   double m_mach;


   // previously public data members which store the altitude and speed constraints
   // Getters are public.
   Units::Length m_altitude_constraint_high;
   Units::Length m_altitude_constraint_low;
   Units::Speed m_speed_constraint_high;
   Units::Speed m_speed_constraint_low;

   // Added data members for RF legs
   Units::Angle m_rf_turn_center_latitude;
   Units::Angle m_rf_turn_center_longitude;
   Units::Length m_rf_turn_arc_radius;
};

inline const std::string &Waypoint::GetName() const {
   return m_name;
}

inline void Waypoint::SetName(const std::string name_in) {
   m_name.assign(name_in);
}

inline Units::Angle Waypoint::GetLatitude() const {
   return m_latitude;
}

inline void Waypoint::SetLatitude(const Units::Angle &latitude_in) {
   m_latitude = latitude_in;
}

inline Units::Angle Waypoint::GetLongitude() const {
   return m_longitude;
}

inline void Waypoint::SetLongitude(const Units::Angle &longitude_in) {
   m_longitude = longitude_in;
}

inline void Waypoint::SetWaypointLatLon(const Units::Angle &latitude_in,
                                        const Units::Angle &longitude_in) {
   SetLatitude(latitude_in);
   SetLongitude(longitude_in);
}

inline Units::Length Waypoint::GetAltitude() const {
   return m_altitude;
}

inline void Waypoint::SetAltitude(const Units::Length &altitude_in) {
   m_altitude = altitude_in;
}

inline Units::Angle Waypoint::GetDescentAngle() const {
   return m_descent_angle;
}

inline Units::Speed Waypoint::GetNominalIas() const {
   return m_nominal_ias;
}

inline void Waypoint::SetNominalIas(const Units::Speed &nominal_ias_in) {
   m_nominal_ias = nominal_ias_in;
}

inline double Waypoint::GetMach() const {
   return m_mach;
}

inline void Waypoint::SetMach(double mach) {
   m_mach = mach;
}

inline Units::Acceleration Waypoint::GetDescentRate() const {
   return m_descent_rate;
}

inline void Waypoint::SetAltitudeConstraintHigh(const Units::Length &altitude_high_in) {
   m_altitude_constraint_high = altitude_high_in;
}

inline Units::Length Waypoint::GetAltitudeConstraintHigh() const {
   return m_altitude_constraint_high;
}

inline void Waypoint::SetAltitudeConstraintLow(const Units::Length &altitude_low_in) {
   m_altitude_constraint_low = altitude_low_in;
}

inline Units::Length Waypoint::GetAltitudeConstraintLow() const {
   return m_altitude_constraint_low;
}

inline void Waypoint::SetSpeedConstraintHigh(const Units::Speed &speed_high_in) {
   m_speed_constraint_high = speed_high_in;
}

inline Units::Speed Waypoint::GetSpeedConstraintHigh() const {
   return m_speed_constraint_high;
}

inline void Waypoint::SetSpeedConstraintLow(const Units::Speed &speed_low_in) {
   m_speed_constraint_low = speed_low_in;
}

inline Units::Speed Waypoint::GetSpeedConstraintLow() const {
   return m_speed_constraint_low;
}

inline Units::Angle Waypoint::GetRfTurnCenterLatitude() const {
   return m_rf_turn_center_latitude;
}

inline void Waypoint::SetRfTurnCenterLatitude(const Units::Angle &rf_turn_center_latitude_in) {
   m_rf_turn_center_latitude = rf_turn_center_latitude_in;
}

inline Units::Angle Waypoint::GetRfTurnCenterLongitude() const {
   return m_rf_turn_center_longitude;
}

inline void Waypoint::SetRfTurnCenterLongitude(const Units::Angle &rf_turn_center_longitude_in) {
   m_rf_turn_center_longitude = rf_turn_center_longitude_in;
}

inline Units::Length Waypoint::GetRfTurnArcRadius() const {
   return m_rf_turn_arc_radius;
}

inline void Waypoint::SetRfTurnArcRadius(const Units::Length &rf_turn_radius_in) {
   m_rf_turn_arc_radius = rf_turn_radius_in;
}

std::ostream &operator<<(std::ostream &out,
                         const Waypoint &waypoint);

std::ostream &operator<<(std::ostream &out,
                         const std::list<Waypoint> &waypoints);

