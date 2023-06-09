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
// 2022 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#pragma once

#include <public/HorizontalPathTracker.h>

/**
 * @brief Calculates the forward course along a horizontal path. "Forward" is defined as the direction of flight.
 */
class DirectionOfFlightCourseCalculator : public HorizontalPathTracker {

  public:
   DirectionOfFlightCourseCalculator();
   DirectionOfFlightCourseCalculator(const std::vector<HorizontalPath> &horizontal_path,
                                     TrajectoryIndexProgressionDirection expected_index_progression);
   virtual ~DirectionOfFlightCourseCalculator();

   /**
    * @brief Calculate the course on the horizontal path at a specific distance along the path.
    */
   bool CalculateCourseAtAlongPathDistance(const Units::Length &distance_along_path,
                                           Units::UnsignedAngle &forward_course);

   /**
    * @brief The course at index zero of the horizontal path.
    */
   Units::UnsignedAngle GetCourseAtPathEnd() const;

   /**
    * @brief The course at index (size-1) of the horizontal path.
    */
   Units::UnsignedAngle GetCourseAtPathStart() const;

  private:
   static log4cplus::Logger m_logger;
   Units::UnsignedAngle m_end_course;
   Units::UnsignedAngle m_start_course;

  protected:
   bool CalculateForwardCourse(const Units::Length &distance_along_path,
                               const std::vector<HorizontalPath> &horizontal_trajectory,
                               const std::vector<HorizontalPath>::size_type starting_trajectory_index,
                               Units::UnsignedAngle &forward_course, Units::Angle &turn_theta,
                               Units::Length &turn_radius,
                               std::vector<HorizontalPath>::size_type &resolved_trajectory_index);
};

inline Units::UnsignedAngle DirectionOfFlightCourseCalculator::GetCourseAtPathEnd() const { return m_end_course; }

inline Units::UnsignedAngle DirectionOfFlightCourseCalculator::GetCourseAtPathStart() const { return m_start_course; }
