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

#include "framework/TestFrameworkAircraft.h"
#include "math/CustomMath.h"
#include <time.h>
#include "public/Waypoint.h"
#include "public/AircraftCalculations.h"
#include "public/Guidance.h"
#include "public/SpeedOnPitchControl.h"
#include "public/SpeedOnThrustControl.h"
#include <iostream>
#include <stdio.h>

using namespace std;

log4cplus::Logger TestFrameworkAircraft::m_logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("IMAircraft"));

TestFrameworkAircraft::TestFrameworkAircraft() {
   m_id = 0;
   m_start_time = 1;
   m_initial_altitude = Units::FeetLength(0);
   m_initial_ias = Units::KnotsSpeed(0);
   m_initial_mach = 0.78;
   m_aircraft_control.reset(new AircraftControl());
   m_weather_truth = (new WeatherTruthByTime())->GetSharedPtr();
}

TestFrameworkAircraft::~TestFrameworkAircraft() = default;

bool TestFrameworkAircraft::load(DecodedStream *input) {
   set_stream(input);

   register_var("initial_altitude", &m_initial_altitude, true);
   register_var("initial_ias", &m_initial_ias, true);
   register_var("initial_mach", &m_initial_mach, true);

   // moved from TestFrameworkDynamics
   string speed_management_type, env_csv_file;
   register_var("ac_type", &m_ac_type, true);
   register_var("speed_management_type", &speed_management_type, true);
   register_var("env_csv_file", &env_csv_file, false);

   register_loadable_with_brackets("aircraft_intent", &m_aircraft_intent, true);
   register_loadable_with_brackets("dynamics", &m_dynamics, false);
   register_loadable_with_brackets("precalc_traj_file", &m_precalc_traj, true);
   register_loadable_with_brackets("airborne_app", &m_airborne_app, true);

   bool isloaded = complete(); // read the stream
   if (isloaded) {
      if (speed_management_type == "thrust") {
         m_aircraft_control.reset(new SpeedOnThrustControl());
      }
      else if (speed_management_type == "pitch") {
         m_aircraft_control.reset(new SpeedOnPitchControl(Units::KnotsSpeed(20.0), Units::FeetLength(500.0)));
      }
      else {
         throw runtime_error("Supported speed_management_type values are thrust and pitch");
      }
   }

   m_weather_truth->LoadEnvFile(env_csv_file);

   return isloaded;
}

void TestFrameworkAircraft::Initialize(Units::Length adsbReceptionRangeThreshold,
                                       const WeatherTruth &weather_truth) {

   // initial position and heading
   EarthModel::LocalPositionEnu initial_enu_position;
   initial_enu_position.x = m_aircraft_intent.GetFms().m_x[0];
   initial_enu_position.y = m_aircraft_intent.GetFms().m_y[0];
   initial_enu_position.z = m_initial_altitude;
   DirectionOfFlightCourseCalculator course_calculator = DirectionOfFlightCourseCalculator(m_precalc_traj.GetHorizontalTrajectory(), TrajectoryIndexProgressionDirection::UNDEFINED);
   Units::Angle initial_heading = course_calculator.GetCourseAtPathStart();

   Waypoint wp0(m_aircraft_intent.GetFms().m_name[0],
                m_aircraft_intent.GetFms().m_latitude[0],
                m_aircraft_intent.GetFms().m_longitude[0],
                Units::zero(), Units::zero(), Units::zero(), Units::zero(), Units::zero());
   list<Waypoint> waypoint_list;
   waypoint_list.push_back(wp0);
   shared_ptr<TangentPlaneSequence> tangent_plane_sequence(
         new TangentPlaneSequence(waypoint_list));

   // Use the first line in the env file for initialization
   m_weather_truth->SetWeatherFromTime(Units::zero());
   m_weather_truth->CalculateTemperatureOffset(m_initial_altitude);

   // initialize eom dynamics
   const TrajectoryFromFile::VerticalData verticalTrajectoryFromFile = m_precalc_traj.GetVerticalData();
   const Units::MetersLength finalAltitude = Units::MetersLength(verticalTrajectoryFromFile.m_altitude_meters[0]);
   Units::KnotsSpeed initial_tas = m_weather_truth->getAtmosphere()->CAS2TAS(m_initial_ias, m_initial_altitude);

   m_dynamics.Initialize(m_precalc_traj.GetMassPercentile(),
                         finalAltitude,
                         initial_tas,
                         tangent_plane_sequence, // only used for Wind
                         initial_enu_position,
                         initial_heading,
                         *m_weather_truth);

   m_bada_calculator.getAircraftParameters(m_ac_type, m_precalc_traj.GetMassPercentile());
   m_bada_calculator.setFlapSpeeds(m_ac_type);

   static const Units::DegreesAngle max_bank_angle(30);
   m_aircraft_control->Initialize(m_bada_calculator,
                                  finalAltitude,
                                  max_bank_angle,
                                  m_precalc_traj.GetPrecalcWaypoint().back());

   InitTruthStateVectorOld();
}

void TestFrameworkAircraft::InitTruthStateVectorOld() {
   m_truth_state_vector_old.m_id = m_id;

   const AircraftIntent::RouteData &fms(m_aircraft_intent.GetFms());
   m_truth_state_vector_old.m_x = Units::FeetLength(fms.m_x[0]).value();
   m_truth_state_vector_old.m_y = Units::FeetLength(fms.m_y[0]).value();
   m_truth_state_vector_old.m_z = m_initial_altitude.value();

   Units::MetersLength distance_to_go;
   m_fms.m_decrementing_distance_calculator.CalculateAlongPathDistanceFromPosition(Units::FeetLength(m_truth_state_vector_old.m_x),
                                                                                   Units::FeetLength(m_truth_state_vector_old.m_y),
                                                                                   distance_to_go);
   m_truth_state_vector_old.m_distance_to_go_meters = distance_to_go.value();

   // copy groundspeed components from ThreeDOFDynamics
   m_truth_state_vector_old.m_xd = Units::FeetPerSecondSpeed(m_dynamics.m_dynamics_state.xd).value();
   m_truth_state_vector_old.m_yd = Units::FeetPerSecondSpeed(m_dynamics.m_dynamics_state.yd).value();
   m_truth_state_vector_old.m_zd = 0.;

   pair<Units::Speed,Units::Speed> wind_components = m_dynamics.GetWindComponents();
   m_truth_state_vector_old.m_Vwx = Units::MetersPerSecondSpeed(wind_components.first).value();
   m_truth_state_vector_old.m_Vwy = Units::MetersPerSecondSpeed(wind_components.second).value();

   //Initialize the time in truth_state_vector_old with this aircraft's start time:
   m_truth_state_vector_old.m_time = m_start_time;
}

bool TestFrameworkAircraft::Update(const SimulationTime &time) {

   LOG4CPLUS_TRACE(m_logger,
                   "Simulating aircraft " << 0 << " for time " << time.getCurrentSimulationTimeAsString());

   if (m_fms.IsFinished()) {
      m_truth_state_vector_old.m_time = CalculateEndTime(m_truth_state_vector_old);
      return true;
   }

   if (time.get_sim_cycle() <= m_start_time) {
      m_truth_state_vector_old.m_time = time.get_sim_cycle();
      return false;
   }

   m_weather_truth->SetWeatherFromTime(time.get_current_simulation_time());

   m_fms.Update(m_truth_state_vector_old, m_precalc_traj.GetPrecalcWaypoint(),
                m_precalc_traj.GetHorizontalTrajectory());

   Guidance current_guidance = m_precalc_traj.Update(m_truth_state_vector_old);

   if (m_airborne_app.IsLoaded()) {
      Guidance speed_guidance_from_application = m_airborne_app.Update(time,
                                                                       m_dynamics,
                                                                       m_truth_state_vector_old,
                                                                       current_guidance);

      // check if guidance has valid data.
      if (speed_guidance_from_application.IsValid() &&
          speed_guidance_from_application.m_ias_command > Units::ZERO_SPEED) {
         current_guidance = speed_guidance_from_application;
      }
   }

   // run aircraft dynamics model to process the next state
   AircraftState state_result = m_dynamics.Update(m_truth_state_vector_old, current_guidance, m_aircraft_control);
   state_result.m_id = m_id;
   state_result.m_time = time.get_sim_cycle();
   Units::MetersLength distance_to_go;

   m_fms.m_decrementing_distance_calculator.CalculateAlongPathDistanceFromPosition(Units::FeetLength(state_result.m_x),
                                                                                   Units::FeetLength(state_result.m_y),
                                                                                   distance_to_go);
   state_result.m_distance_to_go_meters = distance_to_go.value();

   // update aircraft position and add to output list
   m_truth_state_vector_old = state_result;

   return false;
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
                                     bool blend_wind,
                                     WeatherTruth weather_truth) {

   // initialize the FMS information
   m_fms.CopyWaypointsFromIntent(m_aircraft_intent); // copy in waypoint data from aircraft intent
   m_fms.Initialize(m_precalc_traj.GetHorizontalTrajectory()); // initialize the FMS data

   if (m_precalc_traj.IsLoaded()) {
      m_precalc_traj.CalculateWaypoints(m_aircraft_intent);
   }

   m_dynamics.SetBadaAcType(m_ac_type);
}
