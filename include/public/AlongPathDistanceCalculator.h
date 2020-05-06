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

#pragma once

#include <vector>
#include <log4cplus/logger.h>
#include "HorizontalPath.h"
#include "HorizontalPathTracker.h"

/**
 * Calculates distance along a horizontal path and ensures that the progression of
 */
class AlongPathDistanceCalculator: public HorizontalPathTracker
{
public:
   AlongPathDistanceCalculator();
   AlongPathDistanceCalculator(const std::vector<HorizontalPath> &horizontal_path,
                               TrajectoryIndexProgressionDirection expected_index_progression);

   AlongPathDistanceCalculator(const std::vector<HorizontalPath> &horizontal_path,
                               TrajectoryIndexProgressionDirection expected_index_progression,
                               bool use_large_cross_track_tolerance);

   AlongPathDistanceCalculator(const std::vector<HorizontalPath> &horizontal_path,
                               TrajectoryIndexProgressionDirection expected_index_progression,
                               Units::Length specified_cross_track_tolerance);

   virtual ~AlongPathDistanceCalculator();

   /**
    * Calculate a distance along path of the horizontal trajectory for a given position.
    */
   bool CalculateAlongPathDistanceFromPosition(const Units::Length position_x,
                                               const Units::Length position_y,
                                               Units::Length &distance_along_path,
                                               Units::UnsignedAngle &course);

   /**
    * @brief Same as other public method, but this one does not return course. This is here for convenience of callers
    * that do not want the course returned.
    */
   bool CalculateAlongPathDistanceFromPosition(const Units::Length position_x,
                                               const Units::Length position_y,
                                               Units::Length &distance_along_path);

   /**
    * @brief Same as previous but includes the course into the next waypoint if in a turn.  Used for Capture IM-clearance.
    */
   bool CalculateAlongPathDistanceFromPosition(const Units::Length position_x,
                                               const Units::Length position_y,
                                               Units::Length &distance_along_path,
                                               Units::UnsignedAngle &course,
                                               Units::UnsignedAngle &pt_to_pt_course);

   void UpdateHorizontalTrajectory(const std::vector<HorizontalPath> &horizontal_trajectory) override;

private:
   static log4cplus::Logger m_logger;
   static Units::Length CROSS_TRACK_TOLERANCE, EXTENDED_CROSS_TRACK_TOLERANCE;
   bool m_is_first_call;
   Units::NauticalMilesLength m_cross_track_tolerance;
};


