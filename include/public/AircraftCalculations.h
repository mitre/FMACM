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
// Copyright 2019 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#pragma once

#include "public/Atmosphere.h"
#include "public/AircraftState.h"
#include "public/HorizontalPath.h"
#include <vector>
#include <UnsignedAngle.h>
#include <Length.h>
#include <Time.h>
#include <Speed.h>
#include <Area.h>

class AircraftCalculations
{

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
                                               Units::Length &x_position,
                                               Units::Length &y_position,
                                               Units::UnsignedAngle &course,
                                               int &traj_index);

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
   static void LegacyGetPathLengthFromPosition(const Units::Length x,
                                               const Units::Length y,
                                               const std::vector<HorizontalPath> &horizontal_trajectory,
                                               Units::Length &distance_along_path,
                                               Units::Angle &course);


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
   static bool CalculateDistanceAlongPathFromPosition(const Units::Length position_x,
                                                      const Units::Length position_y,
                                                      const std::vector<HorizontalPath> &horizontal_trajectory,
                                                      const std::vector<HorizontalPath>::size_type starting_trajectory_index,
                                                      Units::Length &distance_along_path,
                                                      Units::Angle &course,
                                                      std::vector<HorizontalPath>::size_type &resolved_trajectory_index);

   /**
    * Utility to translate the incoming angle to be on [0, 2pi]. See also Units::UnsignedAngle.
    */
   static Units::UnsignedRadiansAngle Convert0to2Pi(Units::Angle course_in);

   /**
    * Utility to translate the incoming angle to be on [-pi, pi]. See also Units::SignedAngle.
    */
   static Units::SignedRadiansAngle ConvertPitoPi(Units::Angle course_in);

   /**
    * @deprecated
    * @see CoreUtils::CalculateEuclideanDistance()
    */
   static Units::NauticalMilesLength PtToPtDist(Units::Length x0,
                                                Units::Length y0,
                                                Units::Length x1,
                                                Units::Length y1);

   /**
    * Compute ground speed from an aircraft state.
    *
    * Do not write new code that uses this.
    *
    * @deprecated
    */
   static Units::Speed GsAtACS(AircraftState acs);

   /**
    * Use the dot product rule to calculate the angle between vectors. The angle will be on the
    * interval [-pi, pi].
    */
   static Units::SignedRadiansAngle ComputeAngleBetweenVectors(const Units::Length &xvertex,
                                                               const Units::Length &yvertex,
                                                               const Units::Length &x1,
                                                               const Units::Length &y1,
                                                               const Units::Length &x2,
                                                               const Units::Length &y2);

   /**
    * Standard cross-product calculation.
    */
   static Units::Area ComputeCrossProduct(const Units::Length &xvertex,
                                          const Units::Length &yvertex,
                                          const Units::Length &x1,
                                          const Units::Length &y1,
                                          const Units::Length &x2,
                                          const Units::Length &y2);

private:
   static log4cplus::Logger logger;

   struct PathDistance
   {
      std::vector<HorizontalPath>::size_type m_horizontal_path_index;
      Units::Length m_distance_to_path_node;
   };

   static std::vector<PathDistance> ComputePathDistances(
         const Units::Length x,
         const Units::Length y,
         const std::vector<HorizontalPath>::size_type &starting_index,
         const std::vector<HorizontalPath> &hTraj);

   static void CrossTrackError(const Units::Length x,
                               const Units::Length y,
                               int trajIx,
                               const std::vector<HorizontalPath> hTraj,
                               int &nextTrajIx,
                               Units::Length &cte);

};
