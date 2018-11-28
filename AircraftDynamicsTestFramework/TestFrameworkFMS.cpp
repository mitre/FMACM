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

#include <public/CoreUtils.h>
#include "framework/TestFrameworkFMS.h"
#include "math/CustomMath.h"
#include "public/AircraftCalculations.h"

Units::DegreesAngle TestFrameworkFMS::MAX_BANK_ANGLE(25.0);

TestFrameworkFMS::TestFrameworkFMS() {
}

TestFrameworkFMS::~TestFrameworkFMS() {
}


// primary calculation method to update the FMS model
void TestFrameworkFMS::Update(AircraftState state,
      std::vector<PrecalcWaypoint> &precalc_waypoints,
      std::vector<HorizontalPath> &horizontal_trajectory) {

   double xWp, yWp, dx, dy;

   double DesiredCourse;
   double xdot, ydot;
   double Course;
   double TrackError;
   double CrossTrackError;

   //-------------------------------------------------------------------------------------------------
   //	Computes RANGE to next WP
   //	- - - - - - - - - - - - -
   xWp = m_waypoint_x[m_next_waypoint_ix];
   yWp = m_waypoint_y[m_next_waypoint_ix];

   dx = xWp - state.m_x;
   dy = yWp - state.m_y;

   m_previous_range_to_next_waypoint = m_range_to_next_waypoint;
   m_range_to_next_waypoint = sqrt(dx * dx + dy * dy);


   if (m_mode == TRACKING) {
      //Gwang 2010-08: This may give a false premature indication of turn because of the noisiness of Fms.RangeToNextWp.
      if (m_range_to_next_waypoint > m_previous_range_to_next_waypoint && m_range_to_next_waypoint < 4000.0) {
         m_next_waypoint_ix++;

         if (m_next_waypoint_ix > m_number_of_waypoints - 1)
            //The next waypoint (Fms.NextWp) is beyond the final waypoint (Fms.number_of_waypoints - 1)
            //which means that the final waypoint has just been passed.
         {
            return;
         }
         xWp = m_waypoint_x[m_next_waypoint_ix];
         yWp = m_waypoint_y[m_next_waypoint_ix];
         dx = xWp - state.m_x;
         dy = yWp - state.m_y;
         m_previous_range_to_next_waypoint = 1.0e+10;
         m_range_to_next_waypoint = sqrt(dx * dx + dy * dy);
      }


      //	Determine if the a/c changes course between two waypoints
      //	- - - - - - - - - - - - - - - - - -
      if (m_next_waypoint_ix >= m_number_of_waypoints - 1) {
         m_delta_track = 0.;
      } else {
         m_delta_track = m_track[m_next_waypoint_ix + 1] - m_track[m_next_waypoint_ix];
      }

      double BankAngle = CoreUtils::LimitOnInterval(m_delta_track,
                                                    Units::RadiansAngle(-MAX_BANK_ANGLE).value(),
                                                    Units::RadiansAngle(MAX_BANK_ANGLE).value());

      if (BankAngle != 0.00) {
         double gs = Units::FeetPerSecondSpeed(state.GetGroundSpeed()).value();
         m_turn_radius = gs * gs /
               (GRAVITY_METERS_PER_SECOND / FEET_TO_METERS * tan(BankAngle)); // v^2/(g*tan theta)

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

   //--------------------------------------------------------------------------------------------
   DesiredCourse = m_track[m_next_waypoint_ix];

   // sin and cos swapped to account for heading instead of angle
   CrossTrackError = -1.0 * (-dy * sin(DesiredCourse) + dx * cos(DesiredCourse));
   //gwang
   double gs = Units::FeetPerSecondSpeed(state.GetGroundSpeed()).value();
   xdot = gs * sin(state.GetHeading()) *
         cos(state.m_gamma); // + state.wind_x; // changed from cos to sin, since the value is a heading
   ydot = gs * cos(state.GetHeading()) *
         cos(state.m_gamma); // + state.wind_y; // changed from sin to cos, since the value is a heading
   //end gwang
   Course = atan3(xdot, ydot); // atan2(ydot, xdot); switched from Angle to Heading

   TrackError = Course - DesiredCourse;

   // fixes angles > +-180 degrees
   if (TrackError > M_PI) {
      while (TrackError > M_PI) {
         TrackError = TrackError - 2.0 * M_PI;
      }
   }
   if (TrackError < -M_PI) {
      while (TrackError < -M_PI) {
         TrackError = TrackError + 2.0 * M_PI;
      }
   }

   //--------------------------------------------------------------------------------------------

   if (m_mode == TURNING) {
      double DeltaGroundTrack = -1.0 * TrackError;

      if (CrossTrackError < 0.00 && DeltaGroundTrack < 10.00 * DEGREES_TO_RADIAN) {
         m_mode = TRACKING;
      }

      if (CrossTrackError > 0.00 && DeltaGroundTrack > 10.00 * DEGREES_TO_RADIAN) {
         m_mode = TRACKING;
      }

      if (fabs(CrossTrackError) < 1000.0 && fabs(TrackError) < 5.0 * DEGREES_TO_RADIAN) {
         m_mode = TRACKING;
      }
   }


   // Recompute NextWp

   Units::MetersLength currDist;
   Units::RadiansAngle tempCrs;
   AircraftCalculations::GetPathLengthFromPos(
         Units::FeetLength(state.m_x), Units::FeetLength(state.m_y),
         horizontal_trajectory, currDist, tempCrs);

   m_next_waypoint_ix = precalc_waypoints.size() - 1;

   for (int ix = 1; (ix < precalc_waypoints.size()); ix++) {

      if ((currDist.value() < precalc_waypoints[ix - 1].constraints.constraint_dist) &&
            (currDist.value() >= precalc_waypoints[ix].constraints.constraint_dist)) {
         m_next_waypoint_ix = ix;
         break;
      }

   }
}


// init method to initialize the FMS data
void TestFrameworkFMS::Init() {
   int i;
   double dx;
   double dy;
   double Heading;
   for (i = 1; i < m_number_of_waypoints; i++) {
      dx = m_waypoint_x[i] - m_waypoint_x[i - 1];
      dy = m_waypoint_y[i] - m_waypoint_y[i - 1];
      Heading = (double) atan3(dx, dy);// atan2(dy, dx); switched from Angle to Heading

      m_track[i] = Heading;
      m_psi[i] = (double) atan2(dy, dx); // psi measured from east counter-clockwise
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

void TestFrameworkFMS::CopyWaypointsFromIntent(AircraftIntent intent_in) {
   m_number_of_waypoints = intent_in.GetNumberOfWaypoints();

   const AircraftIntent::RouteData &fms(intent_in.GetFms());
   for (int j = 0; j < m_number_of_waypoints; j++) {
      m_waypoint_x[j] = Units::FeetLength(fms.xWp[j]).value();
      m_waypoint_y[j] = Units::FeetLength(fms.yWp[j]).value();
      m_waypoint_altitude[j] = Units::FeetLength(fms.AltWp[j]).value();
      m_nominal_ias_at_waypoint[j] = fms.nominal_IAS_at_waypoint[j].value();
      m_mach_at_waypoint[j] = fms.MACH_at_waypoint[j];
      m_constraints[j].constraint_altHi = fms.altHi[j].value();
      m_constraints[j].constraint_altLow = fms.altLow[j].value();
      m_constraints[j].constraint_speedHi = fms.speedHi[j].value();
      m_constraints[j].constraint_speedLow = fms.speedLow[j].value();
   }
}

// check to see if the FMS has reached the last waypoint
bool TestFrameworkFMS::IsFinished() {
   bool result = false;

   if (m_next_waypoint_ix > m_number_of_waypoints - 1) {
      result = true;
   }

   return result;
}

// get the psi of the given index if in range, returns -999.9 if out of range
double TestFrameworkFMS::GetPsi(int index) {
   double psi_result = -999.9;

   if (index >= 0 && index < m_number_of_waypoints) {
      psi_result = m_psi[index];
   }

   return psi_result;
}
