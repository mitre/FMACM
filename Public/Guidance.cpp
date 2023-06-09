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

#include "public/Guidance.h"
#include <cmath>

using namespace aaesim::open_source;

Guidance::Guidance() {
   m_ias_command = Units::ZERO_SPEED;
   m_ground_speed = Units::ZERO_SPEED;
   m_vertical_speed = Units::ZERO_SPEED;
   m_reference_altitude = Units::ZERO_LENGTH;
   m_cross_track_error = Units::ZERO_LENGTH;
   m_enu_track_angle = Units::ZERO_ANGLE;
   m_reference_bank_angle = Units::ZERO_ANGLE;
   m_mach_command = 0;

   m_use_cross_track = false;
   m_valid = false;
}

Guidance::~Guidance() {}

int Guidance::GetIasCommandIntegerKnots() const {
   double result = round(Units::KnotsSpeed(m_ias_command).value());
   return (int)result;
}
