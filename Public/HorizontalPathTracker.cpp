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

#include <public/HorizontalPathTracker.h>

aaesim::open_source::HorizontalPathTracker::HorizontalPathTracker(
      const std::vector<HorizontalPath> &horizontal_trajectory,
      TrajectoryIndexProgressionDirection expected_index_progression) {
   m_index_progression_direction = expected_index_progression;
   m_unmodified_horizontal_trajectory = horizontal_trajectory;
   m_extended_horizontal_trajectory = ExtendHorizontalTrajectory(horizontal_trajectory);
   if (expected_index_progression == TrajectoryIndexProgressionDirection::INCREMENTING) {
      m_is_passed_end_of_route = true;
   } else {
      m_is_passed_end_of_route = false;
   }
   InitializeStartingIndex();
}

std::vector<aaesim::open_source::HorizontalPath> aaesim::open_source::HorizontalPathTracker::ExtendHorizontalTrajectory(
      const std::vector<aaesim::open_source::HorizontalPath> &horizontal_trajectory) {

   // add one more straight segment to end
   std::vector<aaesim::open_source::HorizontalPath> extended_trajectory;
   Units::RadiansAngle crs(horizontal_trajectory[0].m_path_course);
   aaesim::open_source::HorizontalPath hp;
   hp.m_segment_type = aaesim::open_source::HorizontalPath::SegmentType::STRAIGHT;
   hp.SetXYPositionMeters(horizontal_trajectory[0].GetXPositionMeters() -
                                Units::MetersLength(EXTENSION_LENGTH).value() * Units::cos(crs),
                          horizontal_trajectory[0].GetYPositionMeters() -
                                Units::MetersLength(EXTENSION_LENGTH).value() * Units::sin(crs));  // meter
   hp.m_path_length_cumulative_meters = 0;
   hp.m_path_course = horizontal_trajectory[0].m_path_course;
   extended_trajectory.push_back(hp);

   // extend all lengths
   for (auto itr = horizontal_trajectory.begin(); itr < horizontal_trajectory.end(); ++itr) {
      aaesim::open_source::HorizontalPath element = itr.operator*();
      element.m_path_length_cumulative_meters += Units::MetersLength(EXTENSION_LENGTH).value();
      extended_trajectory.push_back(element);
   }

   // add one more straigt segment to the beginning
   Units::RadiansAngle crs_back(horizontal_trajectory.back().m_path_course);
   aaesim::open_source::HorizontalPath hp_beginning;
   hp_beginning.m_segment_type = aaesim::open_source::HorizontalPath::SegmentType::STRAIGHT;
   hp_beginning.SetXYPositionMeters(
         horizontal_trajectory.back().GetXPositionMeters() +
               Units::MetersLength(EXTENSION_LENGTH).value() * Units::cos(crs_back),
         horizontal_trajectory.back().GetYPositionMeters() +
               Units::MetersLength(EXTENSION_LENGTH).value() * Units::sin(crs_back));  // meter
   hp_beginning.m_path_length_cumulative_meters = horizontal_trajectory.back().m_path_length_cumulative_meters +
                                                  2 * Units::MetersLength(EXTENSION_LENGTH).value();
   hp_beginning.m_path_course = horizontal_trajectory.back().m_path_course;
   extended_trajectory.push_back(hp_beginning);

   return extended_trajectory;
}

void aaesim::open_source::HorizontalPathTracker::InitializeStartingIndex() {

   switch (m_index_progression_direction) {
      case TrajectoryIndexProgressionDirection::DECREMENTING:
         if (m_extended_horizontal_trajectory.size() > 1) {
            UpdateCurrentIndex(m_extended_horizontal_trajectory.size() - 2);
         } else {
            UpdateCurrentIndex(0);
         }
         break;

      case TrajectoryIndexProgressionDirection::UNDEFINED:
      case TrajectoryIndexProgressionDirection::INCREMENTING:
         UpdateCurrentIndex(0);
         break;

      default:
         // The code should never get here.
         UpdateCurrentIndex(INT32_MAX);
         break;
   }
}

bool aaesim::open_source::HorizontalPathTracker::ValidateIndexProgression(
      std::vector<aaesim::open_source::HorizontalPath>::size_type index_to_check) {
   bool is_progress_valid;
   switch (m_index_progression_direction) {
      case TrajectoryIndexProgressionDirection::DECREMENTING:
         is_progress_valid = m_current_index == index_to_check || m_current_index - 1 == index_to_check;
         break;

      case TrajectoryIndexProgressionDirection::UNDEFINED:
         is_progress_valid = true;
         break;

      case TrajectoryIndexProgressionDirection::INCREMENTING:
         is_progress_valid = m_current_index == index_to_check || m_current_index + 1 == index_to_check;
         break;

      default:
         is_progress_valid = false;
         break;
   }

   return is_progress_valid;
}

void aaesim::open_source::HorizontalPathTracker::UpdateHorizontalTrajectory(
      const std::vector<aaesim::open_source::HorizontalPath> &horizontal_trajectory) {
   const auto hp_to_find = m_extended_horizontal_trajectory[m_current_index];
   m_unmodified_horizontal_trajectory = horizontal_trajectory;
   m_extended_horizontal_trajectory = ExtendHorizontalTrajectory(horizontal_trajectory);

   auto find_result =
         std::find(m_extended_horizontal_trajectory.begin(), m_extended_horizontal_trajectory.end(), hp_to_find);
   if (find_result == m_extended_horizontal_trajectory.end()) {
      InitializeStartingIndex();
      return;
   }
   m_current_index = std::distance(m_extended_horizontal_trajectory.begin(), find_result);
}

bool aaesim::open_source::HorizontalPathTracker::IsPositionOnNode(
      const Units::Length position_x, const Units::Length position_y,
      std::vector<aaesim::open_source::HorizontalPath>::size_type &node_index) {
   bool is_on_node =
         Units::abs(Units::MetersLength(m_extended_horizontal_trajectory[m_current_index].GetXPositionMeters()) -
                    position_x) < ON_NODE_TOLERANCE &&
         Units::abs(Units::MetersLength(m_extended_horizontal_trajectory[m_current_index].GetYPositionMeters()) -
                    position_y) < ON_NODE_TOLERANCE;

   if (!is_on_node) {
      std::vector<aaesim::open_source::HorizontalPath>::size_type next_index;
      switch (m_index_progression_direction) {
         case TrajectoryIndexProgressionDirection::DECREMENTING:
            if (m_current_index > 0) {
               next_index = m_current_index - 1;
               auto x_diff =
                     Units::abs(Units::MetersLength(m_extended_horizontal_trajectory[next_index].GetXPositionMeters()) -
                                position_x);
               auto y_diff =
                     Units::abs(Units::MetersLength(m_extended_horizontal_trajectory[next_index].GetYPositionMeters()) -
                                position_y);
               is_on_node = x_diff < ON_NODE_TOLERANCE && y_diff < ON_NODE_TOLERANCE;
               if (is_on_node) node_index = next_index;
            }
            break;

         case TrajectoryIndexProgressionDirection::INCREMENTING:
            if (m_current_index < m_extended_horizontal_trajectory.size() - 1) {
               next_index = m_current_index + 1;
               auto x_diff =
                     Units::abs(Units::MetersLength(m_extended_horizontal_trajectory[next_index].GetXPositionMeters()) -
                                position_x);
               auto y_diff =
                     Units::abs(Units::MetersLength(m_extended_horizontal_trajectory[next_index].GetYPositionMeters()) -
                                position_y);
               is_on_node = x_diff < ON_NODE_TOLERANCE && y_diff < ON_NODE_TOLERANCE;
               if (is_on_node) node_index = next_index;
            }
            break;

         case TrajectoryIndexProgressionDirection::UNDEFINED:
            for (auto index = 0; index < m_extended_horizontal_trajectory.size(); ++index) {
               is_on_node =
                     Units::abs(Units::MetersLength(m_extended_horizontal_trajectory[index].GetXPositionMeters()) -
                                position_x) < ON_NODE_TOLERANCE &&
                     Units::abs(Units::MetersLength(m_extended_horizontal_trajectory[index].GetYPositionMeters()) -
                                position_y) < ON_NODE_TOLERANCE;
               if (is_on_node) {
                  node_index = index;
                  break;
               }
            }
            break;

         default:
            is_on_node = false;
            break;
      }
   } else {
      node_index = m_current_index;
   }

   return is_on_node;
}

bool aaesim::open_source::HorizontalPathTracker::IsDistanceAlongPathOnNode(
      const Units::Length distance_along_path,
      std::vector<aaesim::open_source::HorizontalPath>::size_type &node_index) {
   const Units::Length distance_to_check = distance_along_path + EXTENSION_LENGTH;
   bool is_on_node =
         Units::abs(
               Units::MetersLength(m_extended_horizontal_trajectory[m_current_index].m_path_length_cumulative_meters) -
               distance_to_check) < ON_NODE_TOLERANCE;

   if (!is_on_node) {
      std::vector<aaesim::open_source::HorizontalPath>::size_type next_index;
      switch (m_index_progression_direction) {
         case TrajectoryIndexProgressionDirection::DECREMENTING:
            next_index = m_current_index - 1;
            is_on_node =
                  Units::abs(Units::MetersLength(
                                   m_extended_horizontal_trajectory[next_index].m_path_length_cumulative_meters) -
                             distance_to_check) < ON_NODE_TOLERANCE;
            if (is_on_node) node_index = next_index;
            break;

         case TrajectoryIndexProgressionDirection::INCREMENTING:
            next_index = m_current_index + 1;
            is_on_node =
                  Units::abs(Units::MetersLength(
                                   m_extended_horizontal_trajectory[next_index].m_path_length_cumulative_meters) -
                             distance_to_check) < ON_NODE_TOLERANCE;
            if (is_on_node) node_index = next_index;
            break;

         case TrajectoryIndexProgressionDirection::UNDEFINED:
            for (int index = 0; index < m_extended_horizontal_trajectory.size(); ++index) {
               is_on_node = Units::abs(Units::MetersLength(
                                             m_extended_horizontal_trajectory[index].m_path_length_cumulative_meters) -
                                       distance_to_check) < ON_NODE_TOLERANCE;
               if (is_on_node) {
                  node_index = index;
                  break;
               }
            }
            break;

         default:
            is_on_node = false;
            break;
      }
   } else {
      node_index = m_current_index;
   }

   return is_on_node;
}
