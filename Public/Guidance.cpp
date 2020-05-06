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

#include "public/Guidance.h"
#include <cmath>

Guidance::Guidance() {
   m_ias_command = Units::ZERO_SPEED;
   m_ground_speed = Units::ZERO_SPEED;
   m_vertical_speed = Units::ZERO_SPEED;
   m_reference_altitude = Units::ZERO_LENGTH;
   m_cross_track_error = Units::ZERO_LENGTH;
   m_track_angle = Units::ZERO_ANGLE;
   m_reference_bank_angle = Units::ZERO_ANGLE;

   m_use_cross_track = false;
   m_valid = false;
}

Guidance::~Guidance() {
}

int Guidance::GetIasCommandIntegerKnots() const {
   double result = round(Units::KnotsSpeed(m_ias_command).value());
   return (int) result;
}
