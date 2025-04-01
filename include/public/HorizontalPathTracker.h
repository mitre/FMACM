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

#include <public/HorizontalPath.h>
#include <vector>
#include <log4cplus/logger.h>

namespace aaesim::open_source {

/**
 * Use to define the direction of index travel along the horizontal path.
 */
enum TrajectoryIndexProgressionDirection {
   /**
    * No directionality known or expected.
    */
   UNDEFINED,
   /**
    * The trajectory index is expected to start from zero and increment during subsequent calls. Trajectroy prediction
    * operations tend to progress in this direction.
    */
   INCREMENTING,
   /**
    * The trajectory index is expected to start from the size of the vector and decrement during subsequent calls. This
    * direction is consistent with the aircraft direction of flight.
    */
   DECREMENTING
};

class HorizontalPathTracker {

  public:
   HorizontalPathTracker() = default;
   HorizontalPathTracker(const std::vector<HorizontalPath> &horizontal_trajectory,
                         TrajectoryIndexProgressionDirection expected_index_progression);
   virtual ~HorizontalPathTracker() = default;

   TrajectoryIndexProgressionDirection GetExpectedProgressionDirection() const;

   /**
    * Provides a boolean to indicate if passed end of route. Logic for passed end of route is implemented in child
    * classes.
    *
    * At end of route is a numerical definition with ambiguity. Therefore, this method will likely return false if _at_
    * the end of the route. But this must be decided by child class implementations.
    *
    * @return
    */
   bool IsPassedEndOfRoute() const;

   const std::vector<HorizontalPath>::size_type GetCurrentTrajectoryIndex() const;

   /**
    * @brief Update the horizontal trajectory without resetting the tracking behavior. To be used when the trajectory
    * has experienced a slight change and the tracking behavior should remain consistent. For large changes in the
    * horizontal trajectory, build a new object instead of calling this.
    */
   virtual void UpdateHorizontalTrajectory(const std::vector<HorizontalPath> &horizontal_trajectory);

   const std::vector<HorizontalPath> GetHorizontalPath() const;

   void UpdateCurrentIndex(std::vector<HorizontalPath>::size_type new_index);

   /**
    * Logic to initialize the starting index.
    */
   void InitializeStartingIndex();

   const HorizontalPath GetActivePathSegment() const;

  protected:
   inline static const Units::Length EXTENSION_LENGTH{Units::NauticalMilesLength(1.0)};
   std::vector<HorizontalPath>::size_type m_current_index{0};
   std::vector<HorizontalPath> m_extended_horizontal_trajectory{}, m_unmodified_horizontal_trajectory{};
   bool m_is_passed_end_of_route{false};
   TrajectoryIndexProgressionDirection m_index_progression_direction{TrajectoryIndexProgressionDirection::UNDEFINED};

   /**
    * @brief Subclasses can call this to verify that the index is progressing appropriately.
    */
   bool ValidateIndexProgression(std::vector<HorizontalPath>::size_type index_to_check);

   /**
    * @brief Extends a horizontal path vector in both directions so that the edges of the original path are more
    * conveniently handled mathematically.
    *
    * @param horizontal_trajectory the original vector that will be extended
    * @return a copy of the original vector, but extended with straight segments at both ends
    */
   std::vector<HorizontalPath> ExtendHorizontalTrajectory(const std::vector<HorizontalPath> &horizontal_trajectory);

   /**
    * @brief Checks the incoming location to determine if on a local horizontal path node.
    */
   bool IsPositionOnNode(const Units::Length position_x, const Units::Length position_y,
                         std::vector<HorizontalPath>::size_type &node_index);

   /**
    * @brief Checks the incoming distance to see if it is "close" to a horizontal path node.
    */
   bool IsDistanceAlongPathOnNode(const Units::Length distance_along_path,
                                  std::vector<HorizontalPath>::size_type &node_index);

  private:
   inline static log4cplus::Logger m_logger{log4cplus::Logger::getInstance("HorizontalPathTracker")};
   inline static const Units::MetersLength ON_NODE_TOLERANCE{Units::MetersLength(1e-10)};
};

inline bool HorizontalPathTracker::IsPassedEndOfRoute() const { return m_is_passed_end_of_route; }

inline const std::vector<HorizontalPath> HorizontalPathTracker::GetHorizontalPath() const {
   return m_unmodified_horizontal_trajectory;
}

inline const std::vector<HorizontalPath>::size_type HorizontalPathTracker::GetCurrentTrajectoryIndex() const {
   return m_current_index - 1;  // subtract one because caller doesn't know about m_extended_horizontal_trajectory
}

inline TrajectoryIndexProgressionDirection HorizontalPathTracker::GetExpectedProgressionDirection() const {
   return m_index_progression_direction;
}

inline void HorizontalPathTracker::UpdateCurrentIndex(std::vector<HorizontalPath>::size_type new_index) {
   m_current_index = new_index;
}

inline const HorizontalPath HorizontalPathTracker::GetActivePathSegment() const {
   return m_extended_horizontal_trajectory[m_current_index];
}

}  // namespace aaesim::open_source
