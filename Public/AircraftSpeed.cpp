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

#include <stdexcept>
#include "public/AircraftSpeed.h"

AircraftSpeed::AircraftSpeed() {
   SetSpeed(UNSPECIFIED_SPEED, 0);
}

AircraftSpeed::AircraftSpeed(const speed_type type, const double value) {
   SetSpeed(type, value);
}

/** Convenience constructor for using Units::Speed IAS, stores speed as knots */
AircraftSpeed::AircraftSpeed(const speed_type type, const Units::Speed speed) {
   if (type == MACH_SPEED) {
      throw std::logic_error("Cannot construct MACH_SPEED AircraftSpeed using traditional speed units.");
   }
   SetSpeed(type, Units::KnotsSpeed(speed).value());
}

AircraftSpeed::~AircraftSpeed() {
}

speed_type AircraftSpeed::GetSpeedType() const {
   return m_speed_type;
}

void AircraftSpeed::SetSpeed(const speed_type type, const double value) {
   // TODO range sanity checks -- perhaps allow MACH 0.1 to 10.0, IAS 20 to 2000
   m_speed_type = type;
   m_value = value;
}

double AircraftSpeed::GetValue() const {
   return m_value;
}
