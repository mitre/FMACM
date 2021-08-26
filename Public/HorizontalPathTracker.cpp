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

#include <public/HorizontalPathTracker.h>
#include <public/AircraftCalculations.h>

log4cplus::Logger HorizontalPathTracker::m_logger = log4cplus::Logger::getInstance("HorizontalPathTracker");
const Units::Length HorizontalPathTracker::EXTENSION_LENGTH = Units::NauticalMilesLength(1.0);
const Units::MetersLength HorizontalPathTracker::ON_NODE_TOLERANCE = Units::MetersLength(1e-10);

HorizontalPathTracker::HorizontalPathTracker(const std::vector<HorizontalPath> &horizontal_trajectory, TrajectoryIndexProgressionDirection expected_index_progression) {
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

HorizontalPathTracker::~HorizontalPathTracker() = default;

std::vector<HorizontalPath> HorizontalPathTracker::ExtendHorizontalTrajectory(const std::vector<HorizontalPath> &horizontal_trajectory) {

   // add one more straight segment to end
   std::vector<HorizontalPath> extended_trajectory;
   Units::RadiansAngle crs(horizontal_trajectory[0].m_path_course);
   HorizontalPath hp;
   hp.m_segment_type = HorizontalPath::SegmentType::STRAIGHT;
   hp.SetXYPositionMeters(horizontal_trajectory[0].GetXPositionMeters()
         - Units::MetersLength(EXTENSION_LENGTH).value()*Units::cos(crs),
         horizontal_trajectory[0].GetYPositionMeters()
         - Units::MetersLength(EXTENSION_LENGTH).value()*Units::sin(crs));  // meter
   hp.m_path_length_cumulative_meters = 0;
   hp.m_path_course = horizontal_trajectory[0].m_path_course;
   extended_trajectory.push_back(hp);

   // extend all lengths
   for (auto itr = horizontal_trajectory.begin(); itr < horizontal_trajectory.end(); ++itr) {
      HorizontalPath element = itr.operator*();
      element.m_path_length_cumulative_meters += Units::MetersLength(EXTENSION_LENGTH).value();
      extended_trajectory.push_back(element);
   }

   // add one more straigt segment to the beginning
   Units::RadiansAngle crs_back(horizontal_trajectory.back().m_path_course);
   HorizontalPath hp_beginning;
   hp_beginning.m_segment_type = HorizontalPath::SegmentType::STRAIGHT;
   hp_beginning.SetXYPositionMeters(horizontal_trajectory.back().GetXPositionMeters()
         + Units::MetersLength(EXTENSION_LENGTH).value()*Units::cos(crs_back),
         horizontal_trajectory.back().GetYPositionMeters()
         + Units::MetersLength(EXTENSION_LENGTH).value()*Units::sin(crs_back));  // meter
   hp_beginning.m_path_length_cumulative_meters = horizontal_trajectory.back().m_path_length_cumulative_meters + 2 * Units::MetersLength(EXTENSION_LENGTH).value();
   hp_beginning.m_path_course = horizontal_trajectory.back().m_path_course;
   extended_trajectory.push_back(hp_beginning);

   return extended_trajectory;
}

void HorizontalPathTracker::InitializeStartingIndex() {

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
         UpdateCurrentIndex(INFINITY);
         break;
   }
}

bool HorizontalPathTracker::ValidateIndexProgression(std::vector<HorizontalPath>::size_type index_to_check) {

   bool is_progress_valid;
   switch (m_index_progression_direction) {
      case TrajectoryIndexProgressionDirection::DECREMENTING:
         is_progress_valid = m_current_index == index_to_check ||
                             m_current_index - 1 == index_to_check;
         break;

      case TrajectoryIndexProgressionDirection::UNDEFINED:
         is_progress_valid = true;
         break;

      case TrajectoryIndexProgressionDirection::INCREMENTING:
         is_progress_valid = m_current_index == index_to_check ||
                             m_current_index + 1 == index_to_check;
         break;

      default:
         is_progress_valid = false;
         break;
   }

   return is_progress_valid;
}

HorizontalPathTracker::HorizontalPathTracker() {
   m_current_index = 0;
   m_extended_horizontal_trajectory = std::vector<HorizontalPath>();
   m_unmodified_horizontal_trajectory = std::vector<HorizontalPath>();
   m_is_passed_end_of_route = false;
   m_index_progression_direction = TrajectoryIndexProgressionDirection::UNDEFINED;
}

void HorizontalPathTracker::UpdateHorizontalTrajectory(const std::vector<HorizontalPath> &horizontal_trajectory) {
   m_unmodified_horizontal_trajectory = horizontal_trajectory;
   m_extended_horizontal_trajectory = ExtendHorizontalTrajectory(horizontal_trajectory);
}

bool HorizontalPathTracker::IsPositionOnNode(const Units::Length position_x,
                                             const Units::Length position_y,
                                             std::vector<HorizontalPath>::size_type &node_index) {
   bool is_on_node = Units::abs(Units::MetersLength(m_extended_horizontal_trajectory[m_current_index].GetXPositionMeters()) - position_x) < ON_NODE_TOLERANCE &&
         Units::abs(Units::MetersLength(m_extended_horizontal_trajectory[m_current_index].GetYPositionMeters()) - position_y) < ON_NODE_TOLERANCE;

   if (!is_on_node) {
      std::vector<HorizontalPath>::size_type next_index;
      switch (m_index_progression_direction) {
         case TrajectoryIndexProgressionDirection::DECREMENTING:
            if (m_current_index > 0) {
               next_index = m_current_index - 1;  // can go UB
               auto x_diff = Units::abs(Units::MetersLength(m_extended_horizontal_trajectory[next_index].GetXPositionMeters()) - position_x);
               auto y_diff = Units::abs(Units::MetersLength(m_extended_horizontal_trajectory[next_index].GetYPositionMeters()) - position_y);
               is_on_node = x_diff < ON_NODE_TOLERANCE && y_diff < ON_NODE_TOLERANCE;
               if (is_on_node) node_index = next_index;
            }
            break;

         case TrajectoryIndexProgressionDirection::INCREMENTING:
            if (m_current_index < m_extended_horizontal_trajectory.size()-1) {
               next_index = m_current_index + 1;
               auto x_diff = Units::abs(Units::MetersLength(m_extended_horizontal_trajectory[next_index].GetXPositionMeters()) - position_x);
               auto y_diff = Units::abs(Units::MetersLength(m_extended_horizontal_trajectory[next_index].GetYPositionMeters()) - position_y);
               is_on_node = x_diff < ON_NODE_TOLERANCE && y_diff < ON_NODE_TOLERANCE;
               if (is_on_node) node_index = next_index;
            }
            break;

         case TrajectoryIndexProgressionDirection::UNDEFINED:
            for (auto index = 0; index < m_extended_horizontal_trajectory.size(); ++index) {
               is_on_node = Units::abs(Units::MetersLength(m_extended_horizontal_trajectory[index].GetXPositionMeters()) - position_x) < ON_NODE_TOLERANCE &&
                            Units::abs(Units::MetersLength(m_extended_horizontal_trajectory[index].GetYPositionMeters()) - position_y) < ON_NODE_TOLERANCE;
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

bool HorizontalPathTracker::IsDistanceAlongPathOnNode(const Units::Length distance_along_path,
                                                      std::vector<HorizontalPath>::size_type &node_index) {
   const Units::Length distance_to_check = distance_along_path + EXTENSION_LENGTH;
   bool is_on_node = Units::abs(Units::MetersLength(m_extended_horizontal_trajectory[m_current_index].m_path_length_cumulative_meters) - distance_to_check) < ON_NODE_TOLERANCE;

   if (!is_on_node) {
      std::vector<HorizontalPath>::size_type next_index;
      switch (m_index_progression_direction) {
         case TrajectoryIndexProgressionDirection::DECREMENTING:
            next_index = m_current_index - 1;
            is_on_node = Units::abs(Units::MetersLength(m_extended_horizontal_trajectory[next_index].m_path_length_cumulative_meters) - distance_to_check) < ON_NODE_TOLERANCE;
            if (is_on_node) node_index = next_index;
            break;

         case TrajectoryIndexProgressionDirection::INCREMENTING:
            next_index = m_current_index + 1;
            is_on_node = Units::abs(Units::MetersLength(m_extended_horizontal_trajectory[next_index].m_path_length_cumulative_meters) - distance_to_check) < ON_NODE_TOLERANCE;
            if (is_on_node) node_index = next_index;
            break;

         case TrajectoryIndexProgressionDirection::UNDEFINED:
            for (int index = 0; index < m_extended_horizontal_trajectory.size(); ++index) {
               is_on_node = Units::abs(Units::MetersLength(m_extended_horizontal_trajectory[index].m_path_length_cumulative_meters) - distance_to_check) < ON_NODE_TOLERANCE;
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


