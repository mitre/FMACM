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

#include "public/AircraftState.h"
#include "public/InternalObserver.h"
#include "public/AircraftIntent.h"
#include "public/PrecalcConstraint.h"
#include "public/PrecalcWaypoint.h"
#include "public/HorizontalPath.h"

class TestFrameworkFMS
{
public:
   static Units::DegreesAngle MAX_BANK_ANGLE;

   TestFrameworkFMS();

   ~TestFrameworkFMS();

   void Update(const AircraftState &state,
               const std::vector<PrecalcWaypoint> &precalc_waypoints,
               const std::vector<HorizontalPath> &horizontal_trajectory);

   void Initialize(const std::vector<HorizontalPath> &horizontal_path);

   void CopyWaypointsFromIntent(const AircraftIntent &intent_in);

   bool IsFinished();

   AlongPathDistanceCalculator m_decrementing_distance_calculator;

   int m_number_of_waypoints;
   double m_psi[127];
   double m_waypoint_altitude[128];
   double m_waypoint_x[128];
   double m_waypoint_y[128];

private:

   int m_mode;
   double m_delta_track;
   double m_turn_radius;
   double m_range_start_turn;
   double m_previous_range_to_next_waypoint;
   double m_range_to_next_waypoint;
   double m_track[127];
   double m_length[127];
   int m_next_waypoint_ix;
   double m_nominal_ias_at_waypoint[128];
   double m_mach_at_waypoint[128];
   PrecalcConstraint m_constraints[128];
};


