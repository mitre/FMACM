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

namespace aaesim {
namespace open_source {

struct TurnAnticipation {

   double distance;     // turn anticipation in meters
   double bankAngle;    // bank angle in radians
   double maxAngle;     // max bank angle
   double radius;       // radius of turn
   double groundspeed;  // meters per second

   TurnAnticipation() : distance(0), bankAngle(0), maxAngle(0), radius(0), groundspeed(0){};

   TurnAnticipation(double pDist, double pBank, double pMaxBank, double pRadius, double pgs)
      : distance(pDist), bankAngle(pBank), maxAngle(pMaxBank), radius(pRadius), groundspeed(pgs){};
};
}  // namespace open_source
}  // namespace aaesim