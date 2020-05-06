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

#include "public/PrecalcWaypoint.h"
#include "public/AircraftSpeed.h"

class Guidance
{
public:
   Guidance();

   virtual ~Guidance();

   void SetValid(bool value);

   const bool IsValid() const;

   const AircraftSpeed& GetSelectedSpeed() const;

   void SetSelectedSpeed(const AircraftSpeed& selected_speed);

   int GetIasCommandIntegerKnots() const;

   PrecalcConstraint m_active_precalc_constraints;

   Units::Speed m_ias_command;
   Units::Speed m_ground_speed;
   Units::Speed m_vertical_speed;
   Units::Length m_reference_altitude;
   Units::Length m_cross_track_error;
   Units::Angle m_reference_bank_angle;
   Units::Angle m_track_angle; // measured from east counter-clockwise

   bool m_use_cross_track;

private:
   bool m_valid;
   AircraftSpeed m_selected_speed;
};

inline void Guidance::SetValid(bool value) {
   m_valid = value;
}

inline const bool Guidance::IsValid() const {
   return m_valid;
}

inline const AircraftSpeed& Guidance::GetSelectedSpeed() const {
   return m_selected_speed;
}

inline void Guidance::SetSelectedSpeed(const AircraftSpeed& selected_speed) {
   m_selected_speed = selected_speed;
}

