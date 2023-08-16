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

#include <stdexcept>
#include "public/AircraftSpeed.h"

using namespace aaesim::open_source;

AircraftSpeed::AircraftSpeed() { SetSpeed(UNSPECIFIED_SPEED, INT16_MIN); }

AircraftSpeed::AircraftSpeed(const SpeedValueType type, const double value) { SetSpeed(type, value); }

AircraftSpeed::AircraftSpeed(const SpeedValueType type, const Units::Speed speed) {
   if (type == MACH_SPEED) {
      throw std::logic_error("Cannot construct MACH_SPEED AircraftSpeed using traditional speed units.");
   }
   SetSpeed(type, Units::KnotsSpeed(speed).value());
}

AircraftSpeed::~AircraftSpeed() {}

SpeedValueType AircraftSpeed::GetSpeedType() const { return m_speed_type; }

void AircraftSpeed::SetSpeed(const SpeedValueType type, const double value) {
   m_speed_type = type;
   m_value = value;
}

double AircraftSpeed::GetValue() const { return m_value; }
