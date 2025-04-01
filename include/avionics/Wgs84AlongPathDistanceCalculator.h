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
#include <log4cplus/logger.h>
#include "avionics/Wgs84HorizontalPathTracker.h"

namespace aaesim {
/**
 * Calculates distance along a horizontal path and ensures that the progression of
 */
class Wgs84AlongPathDistanceCalculator : public Wgs84HorizontalPathTracker {

  public:
   Wgs84AlongPathDistanceCalculator() = default;

   Wgs84AlongPathDistanceCalculator(
         const std::vector<Wgs84HorizontalPathSegment> &horizontal_path,
         aaesim::open_source::TrajectoryIndexProgressionDirection expected_index_progression);

   virtual ~Wgs84AlongPathDistanceCalculator() = default;

   /**
    * For a given position on or near the horizontal path, calculate a distance along path of the horizontal trajectory.
    *
    * This method returns multiple artifacts of the calculation. For simple calls, use a same-named method that only
    * returns distance.
    *
    */
   bool CalculateAlongPathDistanceFromPosition(const LatitudeLongitudePoint &point_near_path,
                                               Units::Length &distance_along_path,
                                               Units::SignedAngle &forward_course_enu_along_path,
                                               LatitudeLongitudePoint &point_on_path, Units::Length &distance_to_path);

   /**
    * @brief Same as other public method, but this one does not return course. This is here for convenience of callers
    * that do not want the course returned.
    */
   bool CalculateAlongPathDistanceFromPosition(const LatitudeLongitudePoint &point_near_path,
                                               Units::Length &distance_along_path);

   bool CalculateAlongPathDistanceFromPosition(const Units::Length x, Units::Length y,
                                               Units::Length &distance_along_path) {
      return false;
   };

   void UpdateHorizontalTrajectory(const std::vector<Wgs84HorizontalPathSegment> &horizontal_trajectory) override;

  private:
   static log4cplus::Logger m_logger;
   bool m_is_first_call;
};
}  // namespace aaesim