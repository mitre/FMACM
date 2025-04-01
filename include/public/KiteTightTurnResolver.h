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

#include "public/EuclideanTightTurnResolver.h"

namespace aaesim::open_source {
class KiteTightTurnResolver final : public EuclideanTightTurnResolver {
  public:
   KiteTightTurnResolver() = default;
   ~KiteTightTurnResolver() = default;
   void ResolveTightTurnGeometry(const double courseChange1, const double courseChange2, const double legLength,
                                 double &radius, double &turnDist) override {
      CalculateUsingKite(courseChange1, courseChange2, legLength, radius, turnDist);
   }

  private:
   static void CalculateUsingKite(const double courseChange1, const double courseChange2, const double legLength,
                                  double &radius, double &turnDist) {
      // Calculates the trajectory for a turn when the leg length is less than the required turn distance.
      // courseChange1 is first turn (change in m_path_course at the firat waypoint) in degrees.
      // courseChange2 is second turn (change in m_path_course at the second waypoint)in degrees.
      // legLength is the distance between the first waypoint and the second waypoint.
      // Calculates the inscribed turn that is tangent to the m_path_course into the firat waypoint,
      // tangent to the line between waypoint 1 and 2, and tangent to the m_path_course out of the
      // second waypoint.

      // varaibles for Kite algorithm
      double c, d;  // tangent segments for Kite algorithm

      /*
      //           A
      //          / \
      //         /   \
      //        B  O  C
      //         \ E /
      //          \ /F
      //           D
      // E is center of inscribed circle
      // F is tangent point of inscribed circle and segment_type CD
       */

      Units::Angle A, C, D;               // Kite angles
      double AC, OC, CD;                  // Kite side distances
      Units::Angle ECD, ACE, ACO, OCE;    // Angles for inscribed circle points
      double BC, OE, OA, AE, AD, ED, EF;  // distances for inscribed circle calculations
      // radius;  // equal to EF

      A = Units::ToUnsigned(Units::DegreesAngle(180) - Units::RadiansAngle(fabs(courseChange1)));
      C = Units::ToUnsigned(Units::DegreesAngle(180) - Units::RadiansAngle(fabs(courseChange2)));
      // B = C;  // unused
      D = Units::DegreesAngle(360) - (A + C + C);
      // test for D positive
      if (Units::DegreesAngle(D).value() < 0.0)  // turns do not form a kite
      {
         LOG4CPLUS_INFO(m_logger, "Kite called for tight turn that cannot form a kite, Using half leg-length instead.");
         radius = -1;
         turnDist = 0;
         return;
      }

      // find kite side distances in meters
      AC = legLength;
      // AB = AC; // unused
      OC = AC * sin(A / 2);
      CD = OC / sin(D / 2);
      // BD = CD;  // unused

      // find radius of inscribed circle
      ECD = C / 2;
      ACE = ECD;
      ACO = Units::DegreesAngle(90) - A / 2;
      OCE = ACE - ACO;
      BC = OC * 2;
      OE = BC * tan(OCE) / 2;
      OA = AC * cos(A / 2);
      AE = OA + OE;
      AD = AC * cos(A / 2) + CD * cos(D / 2);
      ED = AD - AE;
      EF = ED * sin(D / 2);
      radius = EF;

      // find tangent segments
      // segment_type a is the turn anticipation at angle A.
      // segment_type b = c is the turn anticipation at angle C
      // segment_type d is the turn anticipation at Angle D, which does not exist as a way point
      d = ED * cos(D / 2);
      c = CD - d;
      // b= c;
      // a = AC - c;
      turnDist = AC - c;
   }
};
}  // namespace aaesim::open_source
