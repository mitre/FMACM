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

#include <vector>
#include "public/HorizontalPath.h"
#include "public/Guidance.h"
#include "public/AircraftState.h"
#include "public/AircraftIntent.h"
#include "public/PrecalcWaypoint.h"
#include "loader/Loadable.h"
#include <Angle.h>
#include <Length.h>


class TrajectoryFromFile : Loadable
{

public:

   struct VerticalData
   {
      std::vector<double> mTimeToGo; // secs
      std::vector<double> mDistToGo; // meters
      std::vector<double> mAlt; // meters
      std::vector<double> mIas; // mps
      std::vector<double> mDotAlt; // mps
   } v_traj;

   TrajectoryFromFile(void);

   ~TrajectoryFromFile(void);

   bool load(DecodedStream *input);

   std::vector<HorizontalPath> getHorizontalData(void);

   VerticalData getVerticalData(void);

   Guidance update(AircraftState state,
                   Guidance guidance_in);

   void calculateWaypoints(AircraftIntent &intent);

   bool is_loaded(void);

   std::vector<HorizontalPath> h_traj;

   Units::Length mAltAtFAF;
   double mMassPercentile;

   std::vector<PrecalcWaypoint> waypoint_vector;


private:

   // Fields from vertical file in order.

   enum VerticalFields
   {
      TIME_TO_GO_SEC = 0,
      DISTANCE_TO_GO_VERT_M,
      ALTITUDE_M,
      IAS_MPS,
      DOT_ALTITUDE_MPS,
      NUM_VERTICAL_TRAJ_FIELDS
   };

   enum HorizontalFields
   {
      IX = 0,
      X_M,
      Y_M,
      DISTANCE_TO_GO_HORZ_M,
      SEGMENT_TYPE,
      COURSE_R,
      TURN_CENTER_X_M,
      TURN_CENTER_Y_M,
      ANGLE_AT_TURN_START_R,
      ANGLE_AT_TURN_END_R,
      TURN_RADIUS_M,
      LAT_D,
      LON_D,
      TURN_CENTER_LAT_D,
      TURN_CENTER_LON_D,
      NUM_HORIZONTAL_TRAJ_FIELDS
   };


   void readVerticalTrajectoryFile(void);

   void readHorizontalTrajectoryFile(void);

   std::string mVerticalTrajectoryFile;
   std::string mHorizontalTrajectoryFile;

   bool mLoaded;

};
