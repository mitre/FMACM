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

#include <public/PositionCalculator.h>
#include <log4cplus/loggingmacros.h>
#include <public/AircraftCalculations.h>
#include <public/CoreUtils.h>
#include "public/PositionCalculator.h"

log4cplus::Logger PositionCalculator::m_logger = log4cplus::Logger::getInstance("PositionCalculator");

PositionCalculator::PositionCalculator(const std::vector<HorizontalPath> &horizontal_path,
                                       TrajectoryIndexProgressionDirection expected_index_progression) : DirectionOfFlightCourseCalculator(horizontal_path, expected_index_progression) {
}


PositionCalculator::PositionCalculator() : DirectionOfFlightCourseCalculator() {

}

PositionCalculator::~PositionCalculator() = default;

bool PositionCalculator::CalculatePositionFromAlongPathDistance(const Units::Length &distance_along_path,
                                                                Units::Length &position_x,
                                                                Units::Length &position_y,
                                                                Units::UnsignedAngle &course) {
   std::vector<HorizontalPath>::size_type resolved_index;
   bool return_value = CalculatePosition(distance_along_path + EXTENSION_LENGTH, m_extended_horizontal_trajectory, m_current_index, position_x, position_y, course, resolved_index);

   // Check for end of route is based on passed in distance_along_path
   switch (m_index_progression_direction) {
      case TrajectoryIndexProgressionDirection::DECREMENTING:
         if (!m_is_passed_end_of_route) {
            m_is_passed_end_of_route = distance_along_path < Units::zero();
         }
         break;

      case TrajectoryIndexProgressionDirection::INCREMENTING:
         if (m_is_passed_end_of_route) {
            m_is_passed_end_of_route = distance_along_path < Units::zero();
         }
         break;

      case TrajectoryIndexProgressionDirection::UNDEFINED:
         m_is_passed_end_of_route = distance_along_path < Units::zero();
         break;

      default:
         break;
   }

   // Verify that resolved_index has not become discontinuous and is progressing appropriately
   const bool found_index_is_valid = return_value && ValidateIndexProgression(resolved_index);
   if (found_index_is_valid) {
      // resolved_index looks correct. Update class member.
      UpdateCurrentIndex(resolved_index);

   } else if (distance_along_path + EXTENSION_LENGTH > Units::MetersLength(m_extended_horizontal_trajectory.back().m_path_length_cumulative_meters)) {
      // distance_along_path is very large so off the back of the path. The old code allowed this situation to quietly
      // happen. For now, it helps a lot to allow this. But, we should consider this deprecated behavior and throw in
      // the future.
      char msg[300];
      std::sprintf(msg, "Very long distance_along_path encountered. Too long for path. Allowing for now: %f", Units::MetersLength(distance_along_path).value());
      LOG4CPLUS_ERROR(m_logger, msg);

   } else {
      // resolved_index looks incorrect. Throw.
      char msg[300];
      std::sprintf(msg, "Invalid index progression encountered from CalculatePositionFromDistanceAlongPath(), current_index %lu, resolved_index %lu", m_current_index,  resolved_index);
      LOG4CPLUS_FATAL(m_logger, msg);
      throw std::logic_error(msg);
   }

   return return_value;
}

bool PositionCalculator::CalculatePosition(const Units::Length &distance_along_path,
                                           const std::vector<HorizontalPath> &horizontal_trajectory,
                                           const std::vector<HorizontalPath>::size_type starting_trajectory_index,
                                           Units::Length &x_position,
                                           Units::Length &y_position,
                                           Units::UnsignedAngle &course,
                                           std::vector<HorizontalPath>::size_type &resolved_trajectory_index) {

   Units::Angle turn_theta;
   Units::Length turn_radius;
   const bool found = CalculateForwardCourse(distance_along_path, horizontal_trajectory, starting_trajectory_index,
                                             course, turn_theta, turn_radius, resolved_trajectory_index);

   if (found) {
      // calculate position based on if it's a straight or turning path
      if (horizontal_trajectory[resolved_trajectory_index].m_segment_type == HorizontalPath::SegmentType::STRAIGHT) {
         const Units::Angle crs = Units::RadiansAngle(horizontal_trajectory[resolved_trajectory_index].m_path_course);

         // calculate output values
         x_position = Units::MetersLength(horizontal_trajectory[resolved_trajectory_index].GetXPositionMeters()) +
                 ((distance_along_path - Units::MetersLength(horizontal_trajectory[resolved_trajectory_index].m_path_length_cumulative_meters)) * cos(crs));
         y_position = Units::MetersLength(horizontal_trajectory[resolved_trajectory_index].GetYPositionMeters()) +
                 ((distance_along_path - Units::MetersLength(horizontal_trajectory[resolved_trajectory_index].m_path_length_cumulative_meters)) * sin(crs));
      } else if (horizontal_trajectory[resolved_trajectory_index].m_segment_type == HorizontalPath::SegmentType::TURN) {
         if ((distance_along_path - Units::MetersLength(horizontal_trajectory[resolved_trajectory_index].m_path_length_cumulative_meters)) < Units::MetersLength(3)) {
            x_position = Units::MetersLength(horizontal_trajectory[resolved_trajectory_index].GetXPositionMeters());
            y_position = Units::MetersLength(horizontal_trajectory[resolved_trajectory_index].GetYPositionMeters());
         } else {
            const Units::Length radius = turn_radius;
            const Units::Angle theta = turn_theta;
            x_position = Units::MetersLength(horizontal_trajectory[resolved_trajectory_index].m_turn_info.x_position_meters) + radius * cos(theta);
            y_position = Units::MetersLength(horizontal_trajectory[resolved_trajectory_index].m_turn_info.y_position_meters) + radius * sin(theta);
         }
      }
   }

   return found;

}

