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

#include "PublicUtils.h"

using namespace std;

std::vector<HorizontalPath> aaesim::test::utils::PublicUtils::CreateStraightHorizontalPath(Quadrant quadrant) {
   // Generate hpath
   static const Units::MetersLength unity = Units::MetersLength(1);
   double course_radians = INFINITY;
   Units::MetersLength x_sign(0), y_sign(0);

   switch (quadrant) {
      case Quadrant::FIRST:
         x_sign = unity;
         y_sign = unity;
         course_radians = M_PI / 4;
         break;
      case Quadrant ::SECOND:
         x_sign = -unity;
         y_sign = unity;
         course_radians = 3 * M_PI / 4;
         break;
      case Quadrant ::THIRD:
         x_sign = -unity;
         y_sign = -unity;
         course_radians = -3 * M_PI / 4;
         break;
      case Quadrant ::FOURTH:
         x_sign = unity;
         y_sign = -unity;
         course_radians = -M_PI / 4;
         break;
      default:
         break;
   }

   vector<HorizontalPath> horizontal_traj;
   HorizontalPath hp0, hp1, hp2;
   hp0.m_segment_type = HorizontalPath::SegmentType::STRAIGHT;
   hp0.SetXYPositionMeters(0 * unity.value(),
         0 * unity.value());  // meter
   hp0.m_path_length_cumulative_meters = 0;
   hp0.m_path_course = course_radians;
   horizontal_traj.push_back(hp0);
   hp1.m_segment_type = HorizontalPath::SegmentType::STRAIGHT;
   hp1.SetXYPositionMeters(1 * unity.value() * x_sign.value(),
         1 * unity.value() * y_sign.value());
   hp1.m_path_length_cumulative_meters = hp0.m_path_length_cumulative_meters + sqrt(2);
   hp1.m_path_course = course_radians;
   horizontal_traj.push_back(hp1);
   hp2.m_segment_type = HorizontalPath::SegmentType::STRAIGHT;
   hp2.SetXYPositionMeters(2 * unity.value() * x_sign.value(),
         2 * unity.value() * y_sign.value());
   hp2.m_path_length_cumulative_meters = hp1.m_path_length_cumulative_meters + sqrt(2);
   hp2.m_path_course = course_radians;
   horizontal_traj.push_back(hp2);

   return horizontal_traj;
}
