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

#include <vector>

#include "public/LineOnEllipsoid.h"
#include "public/LatitudeLongitudePoint.h"
#include "avionics/Wgs84HorizontalPathSegment.h"

namespace aaesim::avionics {
inline static const Units::NauticalMilesLength EXTENSION_LENGTH{10};

struct HorizontalPathExtender {

   static std::vector<Wgs84HorizontalPathSegment> ExtendHorizontalTrajectory(
         const std::vector<Wgs84HorizontalPathSegment> &horizontal_path) {

      // add one more straight segment to end
      const LatitudeLongitudePoint current_end_point = horizontal_path[0].GetShapeOnEllipsoid()->GetStartPoint();
      const LatitudeLongitudePoint new_end_point = current_end_point.ProjectDistanceAlongCourse(
            EXTENSION_LENGTH,
            horizontal_path[0].GetShapeOnEllipsoid()->GetForwardCourseEnuAtStartPoint() + Units::PI_RADIANS_ANGLE);
      const LineOnEllipsoid first_extension_line = LineOnEllipsoid::CreateFromPoints(new_end_point, current_end_point);
      const Wgs84HorizontalPathSegment starting_shape{first_extension_line, Units::ZERO_LENGTH};
      std::vector<Wgs84HorizontalPathSegment> extended_trajectory = {starting_shape};

      // Copy over all the segments in the incoming vector
      Units::Length cumulative_length = first_extension_line.GetShapeLength();
      for (Wgs84HorizontalPathSegment segment : horizontal_path) {
         if (segment.GetCumulativePathLengthIncludingThisShape() == segment.GetCumulativePathLength()) {
            // Drop segments of zero length
            continue;
         }
         Wgs84HorizontalPathSegment segment_copy{segment.GetShapeOnEllipsoid(),
                                                 segment.GetSegmentType(),
                                                 cumulative_length,
                                                 segment.GetTurnInfo().GetBankAngle(),
                                                 segment.GetTurnInfo().GetGroundspeed(),
                                                 segment.GetTurnInfo().GetTurnType()};
         extended_trajectory.push_back(segment_copy);
         cumulative_length = segment_copy.GetCumulativePathLengthIncludingThisShape();
      }

      // Add one more extension line to the vector
      const LatitudeLongitudePoint last_start_point = extended_trajectory.back().GetShapeOnEllipsoid()->GetEndPoint();
      const LatitudeLongitudePoint last_end_point = last_start_point.ProjectDistanceAlongCourse(
            EXTENSION_LENGTH, extended_trajectory.back().GetShapeOnEllipsoid()->GetForwardCourseEnuAtStartPoint());
      const LineOnEllipsoid final_extension_line = LineOnEllipsoid::CreateFromPoints(last_start_point, last_end_point);
      const Wgs84HorizontalPathSegment end_shape{final_extension_line, cumulative_length};
      extended_trajectory.push_back(end_shape);
      return extended_trajectory;
   }
};
}  // namespace aaesim::avionics