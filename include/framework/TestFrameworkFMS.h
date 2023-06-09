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

#include "public/AircraftState.h"
#include "public/AircraftIntent.h"
#include "public/PrecalcConstraint.h"
#include "public/PrecalcWaypoint.h"
#include "public/HorizontalPath.h"

class TestFrameworkFMS {
  public:
   static Units::DegreesAngle MAX_BANK_ANGLE;

   TestFrameworkFMS();

   ~TestFrameworkFMS();

   void Update(const aaesim::open_source::AircraftState &state, const std::vector<PrecalcWaypoint> &precalc_waypoints,
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
