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

#include "avionics/Wgs84PositionCalculator.h"

/**
 * @brief Calculates the forward course along a horizontal path. "Forward" is defined as the direction of flight.
 */
class Wgs84DirectionOfFlightCourseCalculator : public aaesim::Wgs84PositionCalculator {

  public:
   Wgs84DirectionOfFlightCourseCalculator();
   Wgs84DirectionOfFlightCourseCalculator(
         const std::vector<aaesim::Wgs84HorizontalPathSegment> &horizontal_path,
         aaesim::open_source::TrajectoryIndexProgressionDirection expected_index_progression);
   virtual ~Wgs84DirectionOfFlightCourseCalculator() = default;

   /**
    * @brief Calculate the course on the horizontal path at a specific distance along the path.
    */
   bool CalculateCourseAtAlongPathDistance(const Units::Length &distance_along_path,
                                           Units::SignedAngle &forward_course_enu_at_distance_along_path);

   /**
    * @brief The course at index zero of the horizontal path.
    */
   Units::UnsignedAngle GetForwardCourseEnuAtPathEnd() const;

   /**
    * @brief The course at index (size-1) of the horizontal path.
    */
   Units::UnsignedAngle GetForwardCourseEnuAtPathStart() const;

  private:
   static log4cplus::Logger m_logger;
};

inline Units::UnsignedAngle Wgs84DirectionOfFlightCourseCalculator::GetForwardCourseEnuAtPathEnd() const {
   // the base class holds the path direction which is opposite direction of flight
   return m_course_enu_at_path_end + Units::PI_RADIANS_ANGLE;
}

inline Units::UnsignedAngle Wgs84DirectionOfFlightCourseCalculator::GetForwardCourseEnuAtPathStart() const {
   // the base class holds the path direction which is opposite direction of flight
   return m_course_enu_at_path_start + Units::PI_RADIANS_ANGLE;
}
