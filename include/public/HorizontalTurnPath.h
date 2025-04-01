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

#include <scalar/Speed.h>
#include <scalar/UnsignedAngle.h>
#include <scalar/SignedAngle.h>
#include <scalar/Length.h>

namespace aaesim::open_source {

class HorizontalPath;  // avoid recursive class defs

class HorizontalTurnPath final {
  public:
   enum TURN_TYPE { UNKNOWN, PERFORMANCE, RADIUS_FIXED };
   static std::string GetTurnTypeAsString(TURN_TYPE tt) {
      switch (tt) {
         case PERFORMANCE:
            return "PERFORMANCE";
         case RADIUS_FIXED:
            return "RADIUS_FIXED";
         case UNKNOWN:
         default:
            return "UNKNOWN";
      }
   };
   enum TURN_DIRECTION { NO_TURN, LEFT_TURN, RIGHT_TURN };

   HorizontalTurnPath() = default;

   ~HorizontalTurnPath() = default;

   TURN_DIRECTION GetTurnDirection(const HorizontalPath &p0, const HorizontalPath &p1) const;

   double x_position_meters{0};
   double y_position_meters{0};
   Units::UnsignedRadiansAngle q_start{0};
   Units::UnsignedRadiansAngle q_end{0};
   Units::MetersLength radius{0};
   Units::UnsignedRadiansAngle bankAngle{0};
   Units::MetersPerSecondSpeed groundspeed{0};
   TURN_TYPE turn_type{UNKNOWN};
};
}  // namespace aaesim::open_source
