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

#include <algorithm>
#include <public/AlongPathDistanceCalculator.h>
#include <public/AircraftCalculations.h>
#include "public/AlongPathDistanceCalculator.h"

log4cplus::Logger AlongPathDistanceCalculator::m_logger = log4cplus::Logger::getInstance("AlongPathDistanceCalculator");
Units::Length AlongPathDistanceCalculator::CROSS_TRACK_TOLERANCE = Units::NauticalMilesLength(2.5);
Units::Length AlongPathDistanceCalculator::EXTENDED_CROSS_TRACK_TOLERANCE = Units::NauticalMilesLength(20.0);
Units::Length AlongPathDistanceCalculator::CAPTURE_CROSS_TRACK_TOLERANCE = Units::NauticalMilesLength(20.0);

AlongPathDistanceCalculator::AlongPathDistanceCalculator(const std::vector<HorizontalPath> &horizontal_path,
                             TrajectoryIndexProgressionDirection expected_index_progression) : HorizontalPathTracker(horizontal_path, expected_index_progression) {

   m_is_first_call = true;
   m_cross_track_tolerance = CROSS_TRACK_TOLERANCE;
}

AlongPathDistanceCalculator::AlongPathDistanceCalculator(const std::vector<HorizontalPath> &horizontal_path,
                             TrajectoryIndexProgressionDirection expected_index_progression,
                             bool use_large_cross_track_tolerance) : HorizontalPathTracker(horizontal_path, expected_index_progression) {
   m_is_first_call = true;
   if (use_large_cross_track_tolerance)
      m_cross_track_tolerance = EXTENDED_CROSS_TRACK_TOLERANCE;
   else
      m_cross_track_tolerance = CROSS_TRACK_TOLERANCE;
}

AlongPathDistanceCalculator::AlongPathDistanceCalculator(const std::vector<HorizontalPath> &horizontal_path,
                            TrajectoryIndexProgressionDirection expected_index_progression,
                            Units::Length specified_cross_track_tolerance) : HorizontalPathTracker(horizontal_path, expected_index_progression) {
   m_is_first_call = true;
   m_cross_track_tolerance = specified_cross_track_tolerance;
}

AlongPathDistanceCalculator AlongPathDistanceCalculator::CreateForCaptureClearance(const std::vector<HorizontalPath> &horizontal_path)   {
   AlongPathDistanceCalculator result(horizontal_path, /*TrajectoryIndexProgressionDirection::*/UNDEFINED,
           CAPTURE_CROSS_TRACK_TOLERANCE);
   return result;
}

AlongPathDistanceCalculator::~AlongPathDistanceCalculator() = default;

bool AlongPathDistanceCalculator::CalculateAlongPathDistanceFromPosition(const Units::Length position_x,
                                                                         const Units::Length position_y,
                                                                         Units::Length &distance_along_path,
                                                                         Units::UnsignedAngle &course) {
   Units::UnsignedAngle dummy_course = Units::ZERO_ANGLE;
   return CalculateAlongPathDistanceFromPosition(position_x, position_y, distance_along_path, course, dummy_course);
}

/*
 *  pt_to_pt_course is used for PERFORMANCE_TURN, only.  Otherwise, pt_to_pt_course is the same as course.
 *  This is used in IMTimeBasedAchieve to determine alignment when clearance type is CAPTURE.
 */
bool AlongPathDistanceCalculator::CalculateAlongPathDistanceFromPosition(const Units::Length position_x,
                                                                         const Units::Length position_y,
                                                                         Units::Length &distance_along_path,
                                                                         Units::UnsignedAngle &course,
                                                                         Units::UnsignedAngle &pt_to_pt_course) {

   std::vector<HorizontalPath>::size_type resolved_index;
   Units::Length calculated_distance_along_path;
   bool return_boolean = IsPositionOnNode(position_x, position_y, resolved_index);
   if (!return_boolean) {
      if (m_is_first_call)
         UpdateCurrentIndex(0);

      return_boolean = AircraftCalculations::CalculateDistanceAlongPathFromPosition(m_cross_track_tolerance, position_x, position_y,
                                                                                    m_extended_horizontal_trajectory,
                                                                                    m_current_index,
                                                                                    calculated_distance_along_path,
                                                                                    course,
                                                                                    resolved_index);
      HorizontalTurnPath::TURN_TYPE turn_type = m_extended_horizontal_trajectory[resolved_index].m_turn_info.turn_type;
      if (turn_type == HorizontalTurnPath::TURN_TYPE::PERFORMANCE) {
         Units::MetersLength half_turn_dist = Units::MetersLength((m_extended_horizontal_trajectory[resolved_index].m_path_length_cumulative_meters
                 + m_extended_horizontal_trajectory[resolved_index+1].m_path_length_cumulative_meters) / 2);
         if (calculated_distance_along_path > half_turn_dist) // first half of turn
            pt_to_pt_course = course;
         else
            pt_to_pt_course = Units::UnsignedRadiansAngle(Units::UnsignedRadiansAngle(m_extended_horizontal_trajectory[resolved_index].m_path_course) + Units::PI_RADIANS_ANGLE);
      } else {
         // for RADIUS_FIXED only use tangent == course
         // for UNKNOWN (straight segment) pt_to_pt == course
         pt_to_pt_course = course;
      }

   } else {
      calculated_distance_along_path = Units::MetersLength(m_extended_horizontal_trajectory[resolved_index].m_path_length_cumulative_meters);
      course = Units::RadiansAngle(m_extended_horizontal_trajectory[resolved_index].m_path_course) + Units::PI_RADIANS_ANGLE;
      pt_to_pt_course = course;
   }

   if (m_is_first_call) {
      UpdateCurrentIndex(resolved_index); // force validate to succeed
      m_is_first_call = false;
   }

   // Verify that resolved_index has not become discontinuous and is progressing appropriately
   const bool found_index_is_valid = return_boolean && ValidateIndexProgression(resolved_index);
   if (found_index_is_valid) {
      // resolved_index looks correct. Update class member.
      UpdateCurrentIndex(resolved_index);

      // decrement distance-to-go by the amount of the modified horizontal trajectory
      distance_along_path = calculated_distance_along_path - EXTENSION_LENGTH;

      m_is_passed_end_of_route = distance_along_path < Units::zero();
   } else {
      // resolved_index looks incorrect. Throw.
      char msg[300];
      std::sprintf(msg,
                   "Invalid index progression encountered from CalculatePositionFromDistanceAlongPath(), current_index %lu, resolved_index %lu",
                   m_current_index, resolved_index);
      LOG4CPLUS_FATAL(m_logger, msg);
      auto high_index = std::max(m_current_index, resolved_index) + 1;
      auto low_index = std::min(m_current_index, resolved_index);
      for (auto i = low_index; i <= high_index; i++) {
         LOG4CPLUS_TRACE(m_logger, "" << i << ": (" <<
               m_extended_horizontal_trajectory[i].GetXPositionMeters() << "," <<
               m_extended_horizontal_trajectory[i].GetYPositionMeters() << ")");

      }
      throw std::logic_error(msg);
   }

   return return_boolean;
}

AlongPathDistanceCalculator::AlongPathDistanceCalculator() : HorizontalPathTracker(), m_is_first_call(true) { }

bool AlongPathDistanceCalculator::CalculateAlongPathDistanceFromPosition(const Units::Length position_x,
                                                                         const Units::Length position_y,
                                                                         Units::Length &distance_along_path) {
   Units::UnsignedAngle ignored_course;
   return CalculateAlongPathDistanceFromPosition(position_x, position_y, distance_along_path, ignored_course);
}

void AlongPathDistanceCalculator::UpdateHorizontalTrajectory(const std::vector<HorizontalPath> &horizontal_trajectory) {
   HorizontalPathTracker::UpdateHorizontalTrajectory(horizontal_trajectory);
   m_is_first_call = true;
}

