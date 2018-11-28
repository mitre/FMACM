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

#include "framework/TestFrameworkAircraft.h"
#include "math/CustomMath.h"
#include <time.h>
#include "public/Waypoint.h"
#include "public/AircraftCalculations.h"
#include "public/Guidance.h"
#include <iostream>
#include <stdio.h>

using namespace std;

//FILE *TestFrameworkAircraft::mGuidanceOut = fopen("testframework-guidance-output.csv","w");

TestFrameworkAircraft::TestFrameworkAircraft() {
   // Set some default for the parameters that this class will NOT load
   m_id = 0;
   m_start_time = 1;
   m_initial_altitude = Units::FeetLength(0);
   m_initial_ias = Units::KnotsSpeed(0);
   m_initial_mach = 0.78;
   //mFms = new TestFrameworkFMS();
}

TestFrameworkAircraft::~TestFrameworkAircraft() {
   // destructor stub
}

TestFrameworkAircraft::TestFrameworkAircraft(const TestFrameworkAircraft &in) {
   Copy(in);
}

TestFrameworkAircraft &TestFrameworkAircraft::operator=(const TestFrameworkAircraft &in) {
   if (this != &in) {
      Copy(in);
   }
   return *this;
}

// helper method for copying values safely, used by copy constructor and operator=
void TestFrameworkAircraft::Copy(const TestFrameworkAircraft &in) {
   m_start_time = in.m_start_time;
   m_id = in.m_id;
   m_truth_state_vector_old = in.m_truth_state_vector_old;
   m_is_finished = in.m_is_finished;
   m_aircraft_intent = in.m_aircraft_intent;
   m_dynamics = in.m_dynamics;
   m_target_intent_not_used = in.m_target_intent_not_used;
   m_precalc_traj = in.m_precalc_traj;
   m_airborne_app = in.m_airborne_app;
   m_fms = in.m_fms;
   m_aircraft_control = in.m_aircraft_control;
   m_initial_altitude = in.m_initial_altitude;
   m_initial_ias = in.m_initial_ias;
   m_initial_mach = in.m_initial_mach;
}

bool TestFrameworkAircraft::load(DecodedStream *input) {
   set_stream(input);

   //AircraftIntentFromFile aiFromFile;
   //register_loadable_with_brackets("state_model", &state_model, true );
   register_var("initial_altitude", &m_initial_altitude, true);
   register_var("initial_ias", &m_initial_ias, true);
   register_var("initial_mach", &m_initial_mach, true);
   register_loadable_with_brackets("aircraft_intent", &m_aircraft_intent, true);
   register_loadable_with_brackets("dynamics", &m_dynamics, false);
   register_loadable_with_brackets("precalc_traj_file", &m_precalc_traj, true);
   register_loadable_with_brackets("airborne_app", &m_airborne_app, true);

   bool isloaded = complete(); // read the stream
   if (isloaded) {
      //mAircraftIntent = &aiFromFile;
   }

   return isloaded;
}

void TestFrameworkAircraft::Initialize(Units::Length adsbReceptionRangeThreshold,
				       const WeatherTruth &weather) {
   m_dynamics.SetWeatherTruth(weather);

   //Initialize navigation_measurement_old with info in first waypoint and other info:
   InitTruthStateVectorOld();

   //This aircraft has not finished simulation yet:
   m_is_finished = false;
}

void TestFrameworkAircraft::InitTruthStateVectorOld() {
   m_truth_state_vector_old.m_id = m_id;
   //Initialize truth_state_vector_old with info in first waypoint:
   const AircraftIntent::RouteData &fms(m_aircraft_intent.GetFms());
   m_truth_state_vector_old.m_x = Units::FeetLength(fms.xWp[0]).value();
   m_truth_state_vector_old.m_y = Units::FeetLength(fms.yWp[0]).value();
   m_truth_state_vector_old.m_z = m_initial_altitude.value();

   //state_model.recomputePathLengthLeft(truth_state_vector_old.x*FT_M,truth_state_vector_old.y*FT_M);
   Units::UnsignedRadiansAngle dummyCourse; // not used, but required
   Units::MetersLength distToGo;
   AircraftCalculations::GetPathLengthFromPos(
         Units::FeetLength(m_truth_state_vector_old.m_x),
         Units::FeetLength(m_truth_state_vector_old.m_y),
         m_precalc_traj.m_horizontal_trajectory,
         distToGo, dummyCourse);
   m_truth_state_vector_old.m_distance_to_go = distToGo.value();

   // copy groundspeed components from ThreeDOFDynamics
   m_truth_state_vector_old.m_xd = m_dynamics.m_state.xd / FEET_TO_METERS;
   m_truth_state_vector_old.m_yd = m_dynamics.m_state.yd / FEET_TO_METERS;
   m_truth_state_vector_old.m_zd = 0.;

   m_truth_state_vector_old.m_Vwx = Units::MetersPerSecondSpeed(m_dynamics.GetWindVelocityX()).value();
   m_truth_state_vector_old.m_Vwy = Units::MetersPerSecondSpeed(m_dynamics.GetWindVelocityY()).value();

   //Initialize the time in truth_state_vector_old with this aircraft's start time:
   m_truth_state_vector_old.m_time = m_start_time;
}

bool TestFrameworkAircraft::Update(const SimulationTime &time) {
   /*
    *  In this very simple implementation, we only want to drive the
    *  aircraft dynamics and airborne application. All other objects
    *  are ignored.
    */


   //If finished, this aircraft does nothing.
   if (m_is_finished) {
      return true;
   }

   //It is not time for this aircraft to start yet:
   //This aircraft is at the initial waypoint at its start_time. Its simulation starts at
   //its (start_time + simulation_time_step).
   if (time.get_sim_cycle() <= m_start_time) {
      m_truth_state_vector_old.m_time = time.get_sim_cycle();
      //mTruthAircraftStates.push_back(truth_state_vector_old);
      return false;
   }

   Guidance current_guidance;
   AircraftState state_result;

   // update FMS data
   m_fms.Update(m_truth_state_vector_old,
               m_precalc_traj.m_waypoint,
               m_precalc_traj.m_horizontal_trajectory);

   // TODO ? if( state_model.precalc_traj.is_loaded() )
   {
      // LAW: use measured state to determine guidance
      current_guidance = m_precalc_traj.Update(m_truth_state_vector_old, current_guidance);
   }

   // if airborne application is loaded check get airborne application guidance
   if (m_airborne_app.IsLoaded()) {
      Guidance guidancefromairborneapplication;
      guidancefromairborneapplication = m_airborne_app.Update(
            time,
            m_dynamics,
            m_truth_state_vector_old,
            current_guidance);

      // check if guidance has valid data.
      if (guidancefromairborneapplication.is_valid() && guidancefromairborneapplication.m_im_speed_command_ias > 0.0) {
         // Use the guidance as modified by the airborne appliation
         current_guidance = guidancefromairborneapplication;
      }
   }

   //		fprintf(mGuidanceOut,"%f,%f,%f,%f,%f,%f,%i\n",time.get_current_simulation_time(),current_guidance.indicated_airspeed,current_guidance.reference_altitude,current_guidance.altitude_rate,current_guidance.psi,current_guidance.cross_track,current_guidance.use_cross_track);

   // run aircraft dynamics model to process the next state
   m_dynamics.SetFms(&m_fms);
   state_result = m_dynamics.Update(m_truth_state_vector_old, current_guidance);
   state_result.m_id = m_id;
   state_result.m_time = time.get_sim_cycle();

   //state_model.recomputePathLengthLeft(state_result.x*FT_M,state_result.y*FT_M);
   Units::MetersLength distToGo;
   Units::Angle dummyCourse;
   AircraftCalculations::GetPathLengthFromPos(
         Units::FeetLength(state_result.m_x),
         Units::FeetLength(state_result.m_y),
         m_precalc_traj.GetHorizontalData(),
         distToGo, dummyCourse);
   state_result.m_distance_to_go = distToGo.value();

   InternalObserver::getInstance()->storeStateModel(state_result,
                                                    m_dynamics.m_state.flapConfig, m_dynamics.m_state.speed_brake,
                                                    m_dynamics.m_state.v_cas.value());

   AircraftState report_state = state_result; // aircraft state reported to ADS-B (adds noise)

   //mIsFinished = state_model.is_finished();
   m_is_finished = (distToGo.value() < 0);

   // check if the model is finished
   if (m_is_finished == true) {
      m_truth_state_vector_old.m_time = CalculateEndTime(m_truth_state_vector_old);
      //			fclose(mGuidanceOut);
      //			cout << "done" << endl;
      return m_is_finished;
   }

   // update aircraft position and add to output list
   m_truth_state_vector_old = state_result;

   return m_is_finished;

}

double TestFrameworkAircraft::CalculateEndTime(AircraftState state_in) {
   // calculate the previous aircraft state
   AircraftState prev_state = state_in;
   prev_state.m_x -= state_in.m_xd;
   prev_state.m_y -= state_in.m_yd;
   prev_state.m_z -= state_in.m_zd;

   // get waypoint end coordinates
   AircraftState end_point;
   end_point.m_x = m_fms.m_waypoint_x[m_fms.m_number_of_waypoints - 1];
   end_point.m_y = m_fms.m_waypoint_y[m_fms.m_number_of_waypoints - 1];
   end_point.m_z = m_fms.m_waypoint_altitude[m_fms.m_number_of_waypoints - 1];

   // translate origin point to the previous aircraft state
   // translate end point
   state_in.m_x -= prev_state.m_x;
   state_in.m_y -= prev_state.m_y;
   state_in.m_z -= prev_state.m_z;
   // translate last waypoint
   end_point.m_x -= prev_state.m_x;
   end_point.m_y -= prev_state.m_y;
   end_point.m_z -= prev_state.m_z;
   // translate previous position to origin
   prev_state.m_x -= prev_state.m_x;
   prev_state.m_y -= prev_state.m_y;
   prev_state.m_z -= prev_state.m_z;

   // calculate the angle from the previous position to the final position
   double aircraft_angle = atan2(state_in.m_y, state_in.m_x);

   // calculate the angle from the previous position to the end waypoint
   double waypoint_angle = atan2(end_point.m_y, end_point.m_x);

   // calculate the angle of the nearest angle
   double nearest_angle = subtract_headings(aircraft_angle, waypoint_angle);

   // calculate the distance between the previous aircraft point and end waypoint
   double dist_waypoint = sqrt(pow(end_point.m_x, 2) + pow(end_point.m_y, 2));

   // calculate the distance between the previous aircraft point and the aircraft end point
   double dist_aircraft = sqrt(pow(state_in.m_x, 2) + pow(state_in.m_y, 2));

   // calculate distance to closest point between the previous and end point
   double dist_closest = dist_waypoint * cos(nearest_angle);

   // calculates the ratio of distance to the closest point compared to the distance to the aircraft end point
   double ratio = dist_closest / dist_aircraft;

   double end_time = state_in.m_time - 1.0 + ratio; // sets the end time as given time -1 + distance ratio

   return end_time;
}


void TestFrameworkAircraft::PostLoad(Units::Time simulation_time_step,
                                      int predicted_wind_opt,
                                      bool blend_wind) {

   // initialize the FMS information
   m_fms.CopyWaypointsFromIntent(m_aircraft_intent); // copy in waypoint data from aircraft intent
   m_fms.Init(); // initialize the FMS data

   if (m_precalc_traj.IsLoaded()) {
      m_precalc_traj.CalculateWaypoints(m_aircraft_intent);
   }


   // initialize eom dynamics
   const TrajectoryFromFile::VerticalData verticalTrajectoryFromFile = m_precalc_traj.GetVerticalData();
   const Units::MetersLength finalAltitude = Units::MetersLength(verticalTrajectoryFromFile.mAlt[0]);
   m_dynamics.SetFms(&m_fms);
   m_dynamics.Init(m_precalc_traj.m_mass_percentile,
                  finalAltitude,
                  m_initial_altitude,
                  m_initial_ias,   // FIXME should be TAS
                  m_initial_mach,
                  (double) m_start_time);
}
