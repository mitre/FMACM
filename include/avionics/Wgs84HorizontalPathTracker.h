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

#include "public/HorizontalPathTracker.h"
#include <vector>
#include <log4cplus/logger.h>
#include "avionics/Wgs84HorizontalPathSegment.h"

namespace aaesim {

class Wgs84HorizontalPathTracker {

  public:
   static const Units::MetersLength ON_NODE_TOLERANCE;
   Wgs84HorizontalPathTracker(const std::vector<Wgs84HorizontalPathSegment> &horizontal_trajectory,
                              aaesim::open_source::TrajectoryIndexProgressionDirection expected_index_progression);

   virtual ~Wgs84HorizontalPathTracker() = default;

   aaesim::open_source::TrajectoryIndexProgressionDirection GetExpectedProgressionDirection() const;

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

   const std::vector<Wgs84HorizontalPathSegment>::size_type GetCurrentTrajectoryIndex() const;

   const std::vector<Wgs84HorizontalPathSegment>::size_type GetCurrentTrajectoryIndexUnsafe() const;

   /**
    * @brief Update the horizontal trajectory without resetting the tracking behavior. To be used when the trajectory
    * has experienced a slight change and the tracking behavior should remain consistent. For large changes in the
    * horizontal trajectory, build a new object instead of calling this.
    */
   virtual void UpdateHorizontalTrajectory(const std::vector<Wgs84HorizontalPathSegment> &horizontal_trajectory);

   const std::vector<Wgs84HorizontalPathSegment> GetHorizontalPath() const;

   /**
    * Force the current index (of the extended horizontal trajetory vector) to be a specific value.
    * @param new_index
    */
   void UpdateCurrentIndex(std::vector<Wgs84HorizontalPathSegment>::size_type new_index);

  protected:
   static const Units::Length EXTENSION_LENGTH;

   std::vector<Wgs84HorizontalPathSegment> m_extended_horizontal_trajectory, m_unmodified_horizontal_trajectory;

   /**
    * This current index is works for the extended horizontal path, m_extended_horizontal_trajectory. Do not use
    * to index into m_unmodified_horizontal_trajectory.
    */
   std::vector<Wgs84HorizontalPathSegment>::size_type m_current_index;

   /**
    * Generally false. Becomes true when the tracking calculation gets to the zeroth leg of
    * m_extended_horizontal_trajectory.
    */
   bool m_is_passed_end_of_route;

   aaesim::open_source::TrajectoryIndexProgressionDirection m_index_progression_direction;

   /**
    * @brief Subclasses can call this to verify that the index is progressing appropriately.
    */
   bool ValidateIndexProgression(std::vector<Wgs84HorizontalPathSegment>::size_type index_to_check);

   /**
    * @brief Extends a horizontal path vector in both directions so that the edges of the original path are more
    * conveniently handled mathematically.
    *
    * @param horizontal_trajectory the original vector that will be extended
    * @return a copy of the original vector, but extended with straight segments at both ends
    */
   std::vector<Wgs84HorizontalPathSegment> ExtendHorizontalTrajectory(
         const std::vector<Wgs84HorizontalPathSegment> &horizontal_trajectory);

   /**
    * @brief Checks the incoming location to determine if on a local horizontal path node.
    */
   bool IsPositionOnAnyEndNode(const aaesim::LatitudeLongitudePoint &latitude_longitude_point,
                               std::vector<Wgs84HorizontalPathSegment>::size_type &node_index) const;

   /**
    * Logic to initialize the starting index.
    */
   void InitializeStartingIndex();

   Wgs84HorizontalPathTracker();

   std::tuple<std::vector<Wgs84HorizontalPathSegment>::size_type, LatitudeLongitudePoint, Units::Length,
              Units::SignedAngle>
         FindIndexOfNearestShape(const LatitudeLongitudePoint &point_near_path);

   bool IsPositionOnBoundedEndNode(const aaesim::LatitudeLongitudePoint &latitude_longitude_point,
                                   std::vector<Wgs84HorizontalPathSegment>::size_type &node_index);

   void CheckIsPassedEndOfRoute(Units::Length distance_to_check);

  private:
   static const Units::MetersLength GAP_NODE_TOLERANCE;
   static log4cplus::Logger m_logger;
};

inline bool Wgs84HorizontalPathTracker::IsPassedEndOfRoute() const { return m_is_passed_end_of_route; }

inline const std::vector<Wgs84HorizontalPathSegment> Wgs84HorizontalPathTracker::GetHorizontalPath() const {
   return m_unmodified_horizontal_trajectory;
}

inline const std::vector<Wgs84HorizontalPathSegment>::size_type Wgs84HorizontalPathTracker::GetCurrentTrajectoryIndex()
      const {
   // The caller can't receive m_current_index because that will not index correctly into the
   // m_unmodified_horizontal_trajectory that they have. The logic below ensures the caller gets a value that indexes
   // into the horizontal vector they know about.
   std::vector<Wgs84HorizontalPathSegment>::size_type return_this;
   if (m_current_index == 0) {
      return_this = 0;
   } else if (m_current_index >= m_unmodified_horizontal_trajectory.size()) {
      return_this = m_unmodified_horizontal_trajectory.size() - 1;
   } else {
      return_this = m_current_index - 1;
   }
   return return_this;
}

inline const std::vector<Wgs84HorizontalPathSegment>::size_type
      Wgs84HorizontalPathTracker::GetCurrentTrajectoryIndexUnsafe() const {
   return m_current_index;
}

inline aaesim::open_source::TrajectoryIndexProgressionDirection
      Wgs84HorizontalPathTracker::GetExpectedProgressionDirection() const {
   return m_index_progression_direction;
}

inline void Wgs84HorizontalPathTracker::UpdateCurrentIndex(
      std::vector<Wgs84HorizontalPathSegment>::size_type new_index) {
   m_current_index = new_index;
}

};  // namespace aaesim
