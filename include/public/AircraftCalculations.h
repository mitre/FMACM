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

// Structure of distances in computing path length
// from position.  Data inserted from smallest to
// greatest.


class AircraftCalculations
{

public:

   /**
    * Get the position and course of an Aircraft based on current distance and precalculated Horizontal Trajectory
    *
    * @deprecated Do not write new code that calls this.
    * @see PositionCalculator
    * @param distance_to_go
    * @param horizontal_trajectory
    * @param x_position
    * @param y_position
    * @param course
    * @param traj_index
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
    * @param x
    * @param y
    * @param horizontal_trajectory
    * @param distance_along_path
    * @param course
    */
   static void LegacyGetPathLengthFromPosition(const Units::Length x,
                                               const Units::Length y,
                                               const std::vector<HorizontalPath> &horizontal_trajectory,
                                               Units::Length &distance_along_path,
                                               Units::Angle &course);


   /**
    * Get the distance and course of the Aircraft based on the current position and precalculated Horizontal Trajectory
    *
    * @param position_x
    * @param position_y
    * @param horizontal_trajectory
    * @param starting_trajectory_index
    * @param distance_along_path
    * @param course
    * @param resolved_trajectory_index
    * @return true if calculation succeeded, false otherwise
    */
   static bool CalculateDistanceAlongPathFromPosition(const Units::Length position_x,
                                                      const Units::Length position_y,
                                                      const std::vector<HorizontalPath> &horizontal_trajectory,
                                                      const std::vector<HorizontalPath>::size_type starting_trajectory_index,
                                                      Units::Length &distance_along_path,
                                                      Units::Angle &course,
                                                      std::vector<HorizontalPath>::size_type &resolved_trajectory_index);

   static Units::UnsignedRadiansAngle Convert0to2Pi(Units::Angle course_in);

   static Units::SignedRadiansAngle ConvertPitoPi(Units::Angle course_in);

   // method for calculating the Cas ESF
   static double ESFconstantCAS(const Units::Speed v_tas,
                                const Units::Length alt,
                                const Units::Temperature temp_in);

   // method to compute distance between two points given in feet.
   static Units::NauticalMilesLength PtToPtDist(Units::Length x0,
                                                Units::Length y0,
                                                Units::Length x1,
                                                Units::Length y1);

   // method to compute ground speed from an aircraft state.
   static Units::Speed GsAtACS(AircraftState acs);

   static Units::SignedRadiansAngle ComputeAngleBetweenVectors(const Units::Length &xvertex,
                                                               const Units::Length &yvertex,
                                                               const Units::Length &x1,
                                                               const Units::Length &y1,
                                                               const Units::Length &x2,
                                                               const Units::Length &y2);

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
