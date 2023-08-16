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

#include "public/DirectionOfFlightCourseCalculator.h"

#include "public/AircraftCalculations.h"
#include "public/CoreUtils.h"

log4cplus::Logger DirectionOfFlightCourseCalculator::m_logger =
      log4cplus::Logger::getInstance("DirectionOfFlightCourseCalculator");

DirectionOfFlightCourseCalculator::DirectionOfFlightCourseCalculator() {}

DirectionOfFlightCourseCalculator::DirectionOfFlightCourseCalculator(
      const std::vector<HorizontalPath> &horizontal_path,
      TrajectoryIndexProgressionDirection expected_index_progression)
   : HorizontalPathTracker(horizontal_path, expected_index_progression) {

   m_end_course = Units::RadiansAngle(horizontal_path.front().m_path_course) + Units::PI_RADIANS_ANGLE;
   m_start_course = Units::RadiansAngle(horizontal_path.back().m_path_course) + Units::PI_RADIANS_ANGLE;
}

DirectionOfFlightCourseCalculator::~DirectionOfFlightCourseCalculator() = default;

bool DirectionOfFlightCourseCalculator::CalculateCourseAtAlongPathDistance(const Units::Length &distance_along_path,
                                                                           Units::UnsignedAngle &forward_course) {
   std::vector<HorizontalPath>::size_type resolved_index;
   Units::Angle ignored_turn_theta;
   Units::Length ignored_turn_radius;
   bool return_value = CalculateForwardCourse(distance_along_path + EXTENSION_LENGTH, m_extended_horizontal_trajectory,
                                              m_current_index, forward_course, ignored_turn_theta, ignored_turn_radius,
                                              resolved_index);

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

   } else if (distance_along_path + EXTENSION_LENGTH >
              Units::MetersLength(m_extended_horizontal_trajectory.back().m_path_length_cumulative_meters)) {
      // distance_along_path is very large so off the back of the path. The old code allowed this situation to quietly
      // happen. For now, it helps a lot to allow this. But, we should consider this deprecated behavior and throw in
      // the future.
      char msg[300];
      std::sprintf(msg, "Very long distance_along_path encountered. Too long for path. Allowing for now: %f",
                   Units::MetersLength(distance_along_path).value());
      LOG4CPLUS_ERROR(m_logger, msg);

   } else {
      // resolved_index looks incorrect. Throw.
      char msg[300];
      std::sprintf(msg,
                   "Invalid index progression encountered from CalculateCourseAtAlongPathDistance(), current_index "
                   "%lu, resolved_index %lu",
                   m_current_index, resolved_index);
      LOG4CPLUS_FATAL(m_logger, msg);
      throw std::logic_error(msg);
   }

   return return_value;
}

bool DirectionOfFlightCourseCalculator::CalculateForwardCourse(
      const Units::Length &distance_along_path, const std::vector<HorizontalPath> &horizontal_trajectory,
      const std::vector<HorizontalPath>::size_type starting_trajectory_index, Units::UnsignedAngle &forward_course,
      Units::Angle &turn_theta, Units::Length &turn_radius,
      std::vector<HorizontalPath>::size_type &resolved_trajectory_index) {
   std::vector<HorizontalPath>::size_type index = 0;  // stores the index last position < current distance
   bool found = false;
   const double distance_to_find_meters = Units::MetersLength(distance_along_path).value();

   // loop to find the distance
   static const Units::MetersLength on_node_tol(1e-10);
   const std::vector<HorizontalPath>::size_type decremented_starting_index =
         starting_trajectory_index < 1 ? 0 : starting_trajectory_index - 1;
   for (std::vector<HorizontalPath>::size_type loop = decremented_starting_index;
        loop < horizontal_trajectory.size() && !found; ++loop) {
      if (std::abs(distance_to_find_meters - horizontal_trajectory[loop].m_path_length_cumulative_meters) <
          on_node_tol.value()) {
         found = true;
         index = loop;
      } else if (distance_to_find_meters < horizontal_trajectory[loop].m_path_length_cumulative_meters) {
         found = true;
         if (loop > 0) {  // this if prevents loop from being decremented if at zero
            index = loop - 1;
         } else {
            index = 0;
         }
      }
   }

   // See AAES-639 for explanation. In FAS scenario, distance_along_path is different from total
   // m_path_length_cumulative_meters in the 9th decimal place.
   if (!found && (distance_to_find_meters <
                  (horizontal_trajectory[horizontal_trajectory.size() - 1].m_path_length_cumulative_meters + 0.0001))) {
      found = true;
      index = horizontal_trajectory.size() - 1;
   }

   turn_radius = Units::zero();
   turn_theta = Units::zero();
   if (found) {
      // calculate position based on if it's a straight or turning path
      if (horizontal_trajectory[index].m_segment_type == HorizontalPath::SegmentType::STRAIGHT) {
         const Units::Angle crs = Units::RadiansAngle(
               horizontal_trajectory[index].m_path_course);  // get the forward_course for the given index

         // calculate output values
         forward_course = AircraftCalculations::Convert0to2Pi(crs + Units::PI_RADIANS_ANGLE);
      } else if (horizontal_trajectory[index].m_segment_type == HorizontalPath::SegmentType::TURN) {
         if ((distance_along_path - Units::MetersLength(horizontal_trajectory[index].m_path_length_cumulative_meters)) <
             Units::MetersLength(3)) {
            forward_course =
                  Units::UnsignedRadiansAngle(horizontal_trajectory[index].m_path_course) + Units::PI_RADIANS_ANGLE;
         } else {
            turn_radius = Units::MetersLength(horizontal_trajectory[index].m_turn_info.radius);
            Units::UnsignedAngle start = Units::UnsignedRadiansAngle(horizontal_trajectory[index].m_turn_info.q_start);
            Units::UnsignedAngle end = Units::UnsignedRadiansAngle(horizontal_trajectory[index].m_turn_info.q_end);

            // calculate forward_course change between the start and end of turn
            Units::SignedRadiansAngle course_change = AircraftCalculations::ConvertPitoPi(end - start);

            // calculate difference in distance
            Units::Angle delta = Units::RadiansAngle(
                  (distance_along_path -
                   Units::MetersLength(horizontal_trajectory[index].m_path_length_cumulative_meters)) /
                  turn_radius);

            // calculate the theta of the turn
            turn_theta = start + delta * CoreUtils::SignOfValue(course_change.value());
            forward_course = AircraftCalculations::Convert0to2Pi(
                  turn_theta - Units::PI_RADIANS_ANGLE / 2.0 * CoreUtils::SignOfValue(course_change.value()));
         }
      }
      resolved_trajectory_index = index;
   } else {
      resolved_trajectory_index = -1;
   }

   return found;
}
