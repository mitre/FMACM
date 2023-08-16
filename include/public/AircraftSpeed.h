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

#pragma once

#include "scalar/Speed.h"
#include "utility/BoundedValue.h"

enum SpeedValueType {
   UNSPECIFIED_SPEED,
   INDICATED_AIR_SPEED,
   MACH_SPEED,
};

namespace aaesim {
namespace open_source {
class AircraftSpeed {
  public:
   static AircraftSpeed OfMach(const BoundedValue<double, 0, 1> mach_value);
   static AircraftSpeed OfIndicatedAirspeed(const Units::Speed ias);

   AircraftSpeed();
   virtual ~AircraftSpeed();
   SpeedValueType GetSpeedType() const;
   double GetValue() const;

  private:
   AircraftSpeed(const SpeedValueType type, const double value);
   AircraftSpeed(const SpeedValueType type, const Units::Speed value);
   void SetSpeed(const SpeedValueType type, const double value);
   SpeedValueType m_speed_type;
   double m_value;
};

inline AircraftSpeed AircraftSpeed::OfMach(const BoundedValue<double, 0, 1> mach_value) {
   return AircraftSpeed(MACH_SPEED, mach_value);
}

inline AircraftSpeed AircraftSpeed::OfIndicatedAirspeed(const Units::Speed ias) {
   return AircraftSpeed(INDICATED_AIR_SPEED, ias);
}
}  // namespace open_source
}  // namespace aaesim