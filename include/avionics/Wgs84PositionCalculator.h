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

#include "avionics/Wgs84HorizontalPathTracker.h"

namespace aaesim {
/**
 * @brief Track a defined horizontal path and calculate positions on that path. When "forward course" is returned, it is
 * defined as the course in direction of flight.
 */
class Wgs84PositionCalculator : public Wgs84HorizontalPathTracker {

  public:
   Wgs84PositionCalculator(const std::vector<Wgs84HorizontalPathSegment> &horizontal_path,
                           aaesim::open_source::TrajectoryIndexProgressionDirection expected_index_progression);

   virtual ~Wgs84PositionCalculator() = default;

   /**
    * @brief Calculate a location and forward course on the horizontal path at a specific distance along the path.
    */
   bool CalculatePositionAtAlongPathDistance(Units::Length distance_along_path,
                                             LatitudeLongitudePoint &point_at_distance_along_path,
                                             Units::SignedAngle &forward_course_enu);

   /**
    * @brief The course at index zero of the horizontal path.
    */
   Units::UnsignedAngle GetCourseAtPathEnd() const;

   /**
    * @brief The course at index (size-1) of the horizontal path.
    */
   Units::UnsignedAngle GetCourseAtPathStart() const;

   Wgs84PositionCalculator() = default;

  private:
   static log4cplus::Logger m_logger;

  protected:
   Units::SignedAngle m_course_enu_at_path_end;
   Units::SignedAngle m_course_enu_at_path_start;

   bool CalculatePositionAndForwardCourse(
         const Units::Length &distance_along_extended_path,
         std::vector<Wgs84HorizontalPathSegment>::size_type starting_trajectory_index,
         LatitudeLongitudePoint &point_at_distance_along_path,
         Units::SignedAngle &forward_course_enu_at_distance_along_path,
         std::vector<Wgs84HorizontalPathSegment>::size_type &resolved_trajectory_index) const;
};

inline Units::UnsignedAngle Wgs84PositionCalculator::GetCourseAtPathEnd() const { return m_course_enu_at_path_end; }

inline Units::UnsignedAngle Wgs84PositionCalculator::GetCourseAtPathStart() const { return m_course_enu_at_path_start; }
}  // namespace aaesim