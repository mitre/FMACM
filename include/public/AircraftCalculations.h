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

#include "public/Atmosphere.h"
#include "public/AircraftState.h"
#include "public/HorizontalPath.h"
#include <vector>
#include <scalar/UnsignedAngle.h>
#include <scalar/Length.h>
#include <scalar/Time.h>
#include <scalar/Speed.h>
#include <scalar/Area.h>

namespace aaesim::open_source {

class AircraftCalculations {

  public:
   /**
    * Get the position and course of an Aircraft based on current distance and precalculated Horizontal Trajectory
    *
    * @deprecated Do not write new code that calls this.
    * @see PositionCalculator
    * @param distance_to_go for the position desired
    * @param horizontal_trajectory on which distance_to_go is calculated
    * @param x_position, an output
    * @param y_position, an output
    * @param course, an output
    * @param traj_index, an output
    * @return true if calculation succeeded, false otherwise
    */
   static bool LegacyGetPositionFromPathLength(const Units::Length &distance_to_go,
                                               const std::vector<HorizontalPath> &horizontal_trajectory,
                                               Units::Length &x_position, Units::Length &y_position,
                                               Units::UnsignedAngle &course, int &traj_index);

   /**
    * Get the distance and course of the Aircraft based on the current position and precalculated Horizontal Trajectory
    *
    * @deprecated Do not write new code that calls this.
    * @see AlongPathDistanceCalculator
    * @param x position for distance calculation
    * @param y position for distance calculation
    * @param horizontal_trajectory on which position exists and distance is to be calculated
    * @param distance_along_path, an output
    * @param course, an output
    */
   static void LegacyGetPathLengthFromPosition(const Units::Length x, const Units::Length y,
                                               const std::vector<HorizontalPath> &horizontal_trajectory,
                                               Units::Length &distance_along_path, Units::Angle &course);

   /**
    * Get the distance and course of the Aircraft based on the current position and precalculated Horizontal Trajectory
    *
    * @param position_x for distance calculation
    * @param position_y for distance calculation
    * @param horizontal_trajectory that the position must be on and that will be used for distance calculation
    * @param starting_trajectory_index, allows the caller to keep track of indices and make calls more efficient
    * @param distance_along_path, an output
    * @param course, an output
    * @param resolved_trajectory_index, an output that indexes into horizontal_trajectory
    * @return true if calculation succeeded, false otherwise
    */
   static bool CalculateDistanceAlongPathFromPosition(
         const Units::Length position_x, const Units::Length position_y,
         const std::vector<HorizontalPath> &horizontal_trajectory,
         const std::vector<HorizontalPath>::size_type starting_trajectory_index, Units::Length &distance_along_path,
         Units::Angle &course, std::vector<HorizontalPath>::size_type &resolved_trajectory_index);

   /**
    * Intentionally shadows the same-named method. But this one exposes finer control over the cross track error
    * tolerance used in the algorithm. If you don't have a specific reason to control the tolerance, plesae use
    * the other method that sets the tolerance for you.
    *
    * @see AircraftCalculations::CalculateDistanceAlongPathFromPosition
    * @param cross_track_tolerance
    * @param position_x
    * @param position_y
    * @param horizontal_trajectory
    * @param starting_trajectory_index
    * @param distance_along_path
    * @param course
    * @param resolved_trajectory_index
    * @return
    */
   static bool CalculateDistanceAlongPathFromPosition(
         const Units::Length cross_track_tolerance, const Units::Length position_x, const Units::Length position_y,
         const std::vector<HorizontalPath> &horizontal_trajectory,
         const std::vector<HorizontalPath>::size_type starting_trajectory_index, Units::Length &distance_along_path,
         Units::Angle &course, std::vector<HorizontalPath>::size_type &resolved_trajectory_index);

   /**
    * @deprecated
    * @see CoreUtils::CalculateEuclideanDistance()
    */
   static Units::NauticalMilesLength PtToPtDist(Units::Length x0, Units::Length y0, Units::Length x1, Units::Length y1);

   /**
    * Use the dot product rule to calculate the angle between vectors. The angle will be on the
    * interval [-pi, pi].
    */
   static Units::SignedRadiansAngle ComputeAngleBetweenVectors(const Units::Length &xvertex,
                                                               const Units::Length &yvertex, const Units::Length &x1,
                                                               const Units::Length &y1, const Units::Length &x2,
                                                               const Units::Length &y2);

   /**
    * Cross-product calculation of (p1-vertex) X (p2-vertex)
    */
   static Units::Area ComputeCrossProduct(const Units::Length &xvertex, const Units::Length &yvertex,
                                          const Units::Length &x1, const Units::Length &y1, const Units::Length &x2,
                                          const Units::Length &y2);

  private:
   static log4cplus::Logger logger;

   struct PathDistance {
      std::vector<HorizontalPath>::size_type m_horizontal_path_index;
      Units::Length m_distance_to_path_node;
   };

   static std::vector<PathDistance> ComputePathDistances(const Units::Length x, const Units::Length y,
                                                         const std::vector<HorizontalPath>::size_type &starting_index,
                                                         const std::vector<HorizontalPath> &hTraj);

   static void CrossTrackError(const Units::Length position_enu_x, const Units::Length position_enu_y,
                               int current_trajectory_index, const std::vector<HorizontalPath> &horizontal_trajectory,
                               int &next_trajectory_index, Units::Length &cross_track_error);
};
}  // namespace aaesim::open_source
