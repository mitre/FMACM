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
class LawOfSinesResolver final : public EuclideanTightTurnResolver {
  public:
   LawOfSinesResolver() = default;
   ~LawOfSinesResolver() = default;
   void ResolveTightTurnGeometry(const double courseChange1, const double courseChange2, const double legLength,
                                 double &radius, double &turnDist) override {
      CalculateUsingLawOfSines(courseChange1, courseChange2, legLength, radius, turnDist);
   }

  private:
   static void CalculateUsingLawOfSines(const double courseChange1, const double courseChange2, const double legLength,
                                        double &radius, double &turnDist) {
      // Calculates the trajectory for a turn when the leg length is less than the required turn distance.
      // courseChange1 is first turn (change in m_path_course at the first waypoint).
      // courseChange2 is second turn (change in m_path_course at the second waypoint).
      // legLength is the distance between the first waypoint and the second waypoint.
      // Calculates the inscribed turn that is tangent to the m_path_course into the first waypoint,
      // tangent to the line between waypoint 1 and 2, and tangent to the m_path_course out of the
      // second waypoint.
      // output parameters:
      //   radius - the radius of the inscribed circle
      //   turnDist - the turn anticipation distance for the first turn
      const auto A = Units::ToUnsigned(Units::DegreesAngle(180) - Units::RadiansAngle(fabs(courseChange1)));
      const auto B = Units::ToUnsigned(Units::DegreesAngle(180) - Units::RadiansAngle(fabs(courseChange2)));
      const auto AOB = Units::ToUnsigned(Units::DegreesAngle(180) - A / 2 - B / 2);
      const auto AO = legLength * sin(B / 2) / sin(AOB);
      const auto AOC = Units::ToUnsigned(Units::DegreesAngle(90) - A / 2);
      radius = AO * sin(A / 2);
      const auto AC = AO * sin(AOC);
      turnDist = AC;
   }
};
}  // namespace aaesim::open_source
