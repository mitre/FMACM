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

   HorizontalTurnPath();

   ~HorizontalTurnPath();

   bool operator==(const HorizontalTurnPath &that) const;

   TURN_DIRECTION GetTurnDirection(const HorizontalPath &p0, const HorizontalPath &p1) const;

   double x_position_meters;
   double y_position_meters;
   Units::UnsignedRadiansAngle q_start;
   Units::UnsignedRadiansAngle q_end;
   Units::MetersLength radius;
   Units::UnsignedRadiansAngle bankAngle;
   Units::MetersPerSecondSpeed groundspeed;
   TURN_TYPE turn_type;
};
