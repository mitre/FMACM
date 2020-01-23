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
#include "DirectionOfFlightCourseCalculator.h"

/**
 * Calculates position (euclidean x,y) for a given horizontal path (previously defined in x/y) and a
 * distance along that path.
 */
class PositionCalculator: public DirectionOfFlightCourseCalculator
{
public:
   PositionCalculator();
   PositionCalculator(const std::vector<HorizontalPath> &horizontal_path, TrajectoryIndexProgressionDirection expected_index_progression);
   virtual ~PositionCalculator();

   /**
    * @brief Calculate a position and course along the horizontal trajectory from a provided distance along the path.
    */
   bool CalculatePositionFromAlongPathDistance(const Units::Length &distance_along_path,
                                               Units::Length &position_x,
                                               Units::Length &position_y,
                                               Units::UnsignedAngle &course);


private:
   static log4cplus::Logger m_logger;

   bool CalculatePosition(const Units::Length &distance_along_path,
                          const std::vector<HorizontalPath> &horizontal_trajectory,
                          const std::vector<HorizontalPath>::size_type starting_trajectory_index,
                          Units::Length &x_position,
                          Units::Length &y_position,
                          Units::UnsignedAngle &course,
                          std::vector<HorizontalPath>::size_type &resolved_trajectory_index);

};


