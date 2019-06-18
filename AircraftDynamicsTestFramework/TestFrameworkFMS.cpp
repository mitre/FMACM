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

#include <public/CoreUtils.h>
#include <public/AlongPathDistanceCalculator.h>
#include "framework/TestFrameworkFMS.h"
#include "math/CustomMath.h"
#include "public/AircraftCalculations.h"

Units::DegreesAngle TestFrameworkFMS::MAX_BANK_ANGLE(25.0);

TestFrameworkFMS::TestFrameworkFMS()
      : m_decrementing_distance_calculator(),
        m_number_of_waypoints(),
        m_psi(),
        m_waypoint_altitude(),
        m_waypoint_x(),
        m_waypoint_y(),
        m_mode(),
        m_delta_track(),
        m_turn_radius(),
        m_range_start_turn(),
        m_previous_range_to_next_waypoint(),
        m_range_to_next_waypoint(),
        m_track(),
        m_length(),
        m_next_waypoint_ix(),
        m_nominal_ias_at_waypoint(),
        m_mach_at_waypoint(),
        m_constraints() {
}

TestFrameworkFMS::~TestFrameworkFMS() = default;

void TestFrameworkFMS::Initialize(const std::vector<HorizontalPath> &horizontal_path) {
   m_decrementing_distance_calculator =
         AlongPathDistanceCalculator(horizontal_path, TrajectoryIndexProgressionDirection::DECREMENTING);

   int i;

   for (i = 1; i < m_number_of_waypoints; i++) {
      double dx = m_waypoint_x[i] - m_waypoint_x[i - 1];
      double dy = m_waypoint_y[i] - m_waypoint_y[i - 1];
      auto heading = static_cast<double>(atan3(dx, dy));

      m_track[i] = heading;
      m_psi[i] = static_cast<double>(atan2(dy, dx));
      m_length[i] = sqrt(dx * dx + dy * dy);
   }

   for (i = m_number_of_waypoints; i < 127; i++) {
      m_track[i] = -999.9;
      m_psi[i] = -999.9;
      m_length[i] = -999.0;
   }

   m_next_waypoint_ix = 1;
   m_mode = TRACKING;
   m_range_to_next_waypoint = 1.0e+10;
   m_previous_range_to_next_waypoint = 1.0e+10;
}

void TestFrameworkFMS::Update(const AircraftState &state,
                              const std::vector<PrecalcWaypoint> &precalc_waypoints,
                              const std::vector<HorizontalPath> &horizontal_trajectory) {
   double xWp = m_waypoint_x[m_next_waypoint_ix];
   double yWp = m_waypoint_y[m_next_waypoint_ix];

   double dx = xWp - state.m_x;
   double dy = yWp - state.m_y;

   m_previous_range_to_next_waypoint = m_range_to_next_waypoint;
   m_range_to_next_waypoint = sqrt(dx * dx + dy * dy);

   if (m_mode == TRACKING) {
      if (m_range_to_next_waypoint > m_previous_range_to_next_waypoint && m_range_to_next_waypoint < 4000.0) {
         m_next_waypoint_ix++;

         if (m_next_waypoint_ix > m_number_of_waypoints - 1) {
            return;
         }
         xWp = m_waypoint_x[m_next_waypoint_ix];
         yWp = m_waypoint_y[m_next_waypoint_ix];
         dx = xWp - state.m_x;
         dy = yWp - state.m_y;
         m_previous_range_to_next_waypoint = 1.0e+10;
         m_range_to_next_waypoint = sqrt(dx * dx + dy * dy);
      }

      if (m_next_waypoint_ix >= m_number_of_waypoints - 1) {
         m_delta_track = 0.;
      } else {
         m_delta_track = m_track[m_next_waypoint_ix + 1] - m_track[m_next_waypoint_ix];
      }

      double bank_angle_rad = CoreUtils::LimitOnInterval(m_delta_track,
                                                         Units::RadiansAngle(-MAX_BANK_ANGLE).value(),
                                                         Units::RadiansAngle(MAX_BANK_ANGLE).value());

      if (bank_angle_rad != 0.00) {
         double gs = Units::FeetPerSecondSpeed(state.GetGroundSpeed()).value();
         m_turn_radius = gs * gs /
                         (GRAVITY_METERS_PER_SECOND / FEET_TO_METERS * tan(bank_angle_rad));

         m_range_start_turn = fabs(m_turn_radius * tan(0.5 * m_delta_track));
      } else {
         m_turn_radius = 1.0e+10;
         m_range_start_turn = 0.00;
      }

      if (m_range_to_next_waypoint <= (m_range_start_turn)) {
         m_mode = TURNING;
         m_next_waypoint_ix++;

         if (m_next_waypoint_ix > m_number_of_waypoints - 1) {
            return;
         }

         xWp = m_waypoint_x[m_next_waypoint_ix];
         yWp = m_waypoint_y[m_next_waypoint_ix];
         dx = xWp - state.m_x;
         dy = yWp - state.m_y;
         m_previous_range_to_next_waypoint = 1.0e+10;
         m_range_to_next_waypoint = sqrt(dx * dx + dy * dy);
      }
   }

   double desired_course = m_track[m_next_waypoint_ix];

   // sin and cos swapped to account for heading instead of angle
   double cross_track_error = -1.0 * (-dy * sin(desired_course) + dx * cos(desired_course));

   double gs = Units::FeetPerSecondSpeed(state.GetGroundSpeed()).value();
   double xdot = gs * sin(state.GetHeadingCcwFromEastRadians()) * cos(state.m_gamma);
   double ydot = gs * cos(state.GetHeadingCcwFromEastRadians()) * cos(state.m_gamma);

   double course = atan3(xdot, ydot);

   double track_error = course - desired_course;

   if (track_error > M_PI) {
      while (track_error > M_PI) {
         track_error = track_error - 2.0 * M_PI;
      }
   }
   if (track_error < -M_PI) {
      while (track_error < -M_PI) {
         track_error = track_error + 2.0 * M_PI;
      }
   }

   if (m_mode == TURNING) {
      double DeltaGroundTrack = -1.0 * track_error;

      if (cross_track_error < 0.00 && DeltaGroundTrack < 10.00 * DEGREES_TO_RADIAN) {
         m_mode = TRACKING;
      }

      if (cross_track_error > 0.00 && DeltaGroundTrack > 10.00 * DEGREES_TO_RADIAN) {
         m_mode = TRACKING;
      }

      if (fabs(cross_track_error) < 1000.0 && fabs(track_error) < 5.0 * DEGREES_TO_RADIAN) {
         m_mode = TRACKING;
      }
   }

   Units::MetersLength current_distance_to_go;
   m_decrementing_distance_calculator.CalculateAlongPathDistanceFromPosition(Units::FeetLength(state.m_x),
                                                                             Units::FeetLength(state.m_y),
                                                                             current_distance_to_go);


   for (auto ix = 0; (ix < precalc_waypoints.size()); ix++) {
      if (current_distance_to_go.value() <= precalc_waypoints[ix].m_precalc_constraints.constraint_dist) {
         this->m_next_waypoint_ix = m_number_of_waypoints - ix - 1;
         break;
      }
   }
}

void TestFrameworkFMS::CopyWaypointsFromIntent(const AircraftIntent &intent_in) {
   m_number_of_waypoints = intent_in.GetNumberOfWaypoints();

   const AircraftIntent::RouteData &fms(intent_in.GetFms());
   for (int j = 0; j < m_number_of_waypoints; j++) {
      m_waypoint_x[j] = Units::FeetLength(fms.m_x[j]).value();
      m_waypoint_y[j] = Units::FeetLength(fms.m_y[j]).value();
      m_waypoint_altitude[j] = Units::FeetLength(fms.m_altitude[j]).value();
      m_nominal_ias_at_waypoint[j] = fms.m_nominal_ias[j].value();
      m_mach_at_waypoint[j] = fms.m_mach[j];
      m_constraints[j].constraint_altHi = fms.m_high_altitude_constraint[j].value();
      m_constraints[j].constraint_altLow = fms.m_low_altitude_constraint[j].value();
      m_constraints[j].constraint_speedHi = fms.m_high_speed_constraint[j].value();
      m_constraints[j].constraint_speedLow = fms.m_low_speed_constraint[j].value();
   }
}

bool TestFrameworkFMS::IsFinished() {
   return m_decrementing_distance_calculator.IsPassedEndOfRoute();
}
