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
// Copyright 2018 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#pragma once

#include "Atmosphere.h"
#include "AircraftState.h"
#include "HorizontalPath.h"
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
   // method to get the position and course of an Aircraft based on current distance and precalculated Horizontal Trajectory
   static bool GetPosFromPathLength(const Units::Length &dist_in,
                                    const std::vector<HorizontalPath> &traj_in,
                                    Units::Length &x_out,
                                    Units::Length &y_out,
                                    Units::UnsignedAngle &course_out,
                                    int &traj_index);

   // method to get the distance and course of the Aircraft based on the current position and precalculated Horizontal Trajectory
   static void GetPathLengthFromPos(const Units::Length x,
                                    const Units::Length y,
                                    const std::vector<HorizontalPath> &hTraj,
                                    Units::Length &dist,
                                    Units::Angle &trk);

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

   struct mPathDistance
   {
      int mIx; // Index to point in horizontal trajectory.
      Units::Length mDist;
   };

   static std::vector<mPathDistance> ComputePathDistances(
         const Units::Length x,
         const Units::Length y,
         const std::vector<HorizontalPath> &hTraj);

   static void CrossTrackError(const Units::Length x,
                               const Units::Length y,
                               int trajIx,
                               const std::vector<HorizontalPath> hTraj,
                               int &nextTrajIx,
                               Units::Length &cte);

};
