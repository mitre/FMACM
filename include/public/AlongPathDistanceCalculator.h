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
   static AlongPathDistanceCalculator CreateForCaptureClearance(const std::vector<HorizontalPath> &horizontal_path);
   AlongPathDistanceCalculator();
   AlongPathDistanceCalculator(const std::vector<HorizontalPath> &horizontal_path,
                               TrajectoryIndexProgressionDirection expected_index_progression);

   AlongPathDistanceCalculator(const std::vector<HorizontalPath> &horizontal_path,
                               TrajectoryIndexProgressionDirection expected_index_progression,
                               bool use_large_cross_track_tolerance);

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
   static Units::Length CROSS_TRACK_TOLERANCE, EXTENDED_CROSS_TRACK_TOLERANCE, CAPTURE_CROSS_TRACK_TOLERANCE;
   bool m_is_first_call;
   Units::NauticalMilesLength m_cross_track_tolerance;
   // private constructor used to construct an AlongPathDistanceCalculator when clearance is CAPTURE
   AlongPathDistanceCalculator(const std::vector<HorizontalPath> &horizontal_path,
                               TrajectoryIndexProgressionDirection expected_index_progression,
                               Units::Length specified_cross_track_tolerance);
};


