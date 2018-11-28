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

#include "framework/TrajectoryFromFile.h"
#include "public/AircraftCalculations.h"
#include "public/CoreUtils.h"
#include "utility/CsvParser.h"


TrajectoryFromFile::TrajectoryFromFile() {

   m_vertical_trajectory.mTimeToGo.clear();
   m_vertical_trajectory.mDistToGo.clear();
   m_vertical_trajectory.mAlt.clear();
   m_vertical_trajectory.mIas.clear();
   m_vertical_trajectory.mDotAlt.clear();

   m_horizontal_trajectory.clear();

   m_vertical_trajectory_file = "";
   m_horizontal_trajectory_file = "";

   m_mass_percentile = 0.5; // Dave Elliott says we can hard-code this for the tests
   m_altitude_at_final_approach_fix = Units::FeetLength(-50.0);

   m_loaded = false;
}

TrajectoryFromFile::~TrajectoryFromFile() {
}

bool TrajectoryFromFile::load(DecodedStream *input) {

   set_stream(input);

   register_var("vfp_csv_file", &m_vertical_trajectory_file);
   register_var("hfp_csv_file", &m_horizontal_trajectory_file);

   m_loaded = complete();

   if (m_loaded) {
      ReadVerticalTrajectoryFile();
      ReadHorizontalTrajectoryFile();
   }

   return m_loaded;

}


void TrajectoryFromFile::ReadVerticalTrajectoryFile() {

   std::ifstream file(m_vertical_trajectory_file.c_str());

   if (!file.is_open()) {
      std::cout << "Vertical trajectory file " << m_vertical_trajectory_file.c_str() << " not found" << std::endl;
      exit(-20);
   }

   int numhdrs = 0;

   for (CsvParser::CsvIterator csviter(file); csviter != CsvParser::CsvIterator(); ++csviter) {

      if (numhdrs < 1) {
         numhdrs++;
         continue;
      }

      if ((int) (*csviter).Size() != NUM_VERTICAL_TRAJ_FIELDS) {
         std::cout << "Bad number of fields found in " << m_vertical_trajectory_file.c_str()
                   << std::endl << "vertical trajectory file.  Found " << (int) (*csviter).Size()
                   << " fields expected " << (int) NUM_VERTICAL_TRAJ_FIELDS << " fields." << std::endl;
         exit(-21);
      }

      for (int vfield = TIME_TO_GO_SEC; vfield != NUM_VERTICAL_TRAJ_FIELDS; vfield++) {
         std::string fieldstr = (*csviter)[vfield];

         double val = atof(fieldstr.c_str());

         switch (static_cast<VerticalFields>(vfield)) {

            case TIME_TO_GO_SEC:
               m_vertical_trajectory.mTimeToGo.push_back(val);
               break;

            case DISTANCE_TO_GO_VERT_M:
               m_vertical_trajectory.mDistToGo.push_back(val);
               break;

            case ALTITUDE_M:
               m_vertical_trajectory.mAlt.push_back(val);
               break;

            case IAS_MPS:
               m_vertical_trajectory.mIas.push_back(val);
               break;

            case DOT_ALTITUDE_MPS:
               m_vertical_trajectory.mDotAlt.push_back(val);
               break;
         }
      }
   }

   file.close();

}


void TrajectoryFromFile::ReadHorizontalTrajectoryFile() {

   std::ifstream file(m_horizontal_trajectory_file.c_str());

   if (!file.is_open()) {
      std::cout << "Horizontal trajectory file " << m_horizontal_trajectory_file.c_str() << " not found" << std::endl;
      exit(-22);
   }

   int numhdrs = 0;

   for (CsvParser::CsvIterator csviter(file); csviter != CsvParser::CsvIterator(); ++csviter) {

      if (numhdrs < 2) {
         numhdrs++;
         continue;
      }

      if ((int) (*csviter).Size() != (int) NUM_HORIZONTAL_TRAJ_FIELDS) {
         std::cout << "Bad number of fields found in " << m_horizontal_trajectory_file.c_str()
                   << std::endl << "vertical trajectory file.  Found " << (int) (*csviter).Size()
                   << " fields expected " << (int) NUM_HORIZONTAL_TRAJ_FIELDS << " fields." << std::endl;
         exit(-38);
      }

      HorizontalPath htrajseg;

      for (int hfield = IX; hfield != (int) NUM_HORIZONTAL_TRAJ_FIELDS; hfield++) {
         std::string fieldstr = (*csviter)[hfield];

         double val = atof(fieldstr.c_str());

         switch (static_cast<HorizontalFields>(hfield)) {

            case IX:
               // unused
               break;

            case X_M:
               htrajseg.x = val;
               break;

            case Y_M:
               htrajseg.y = val;
               break;

            case DISTANCE_TO_GO_HORZ_M:
               htrajseg.L = val;
               break;

            case SEGMENT_TYPE:
               htrajseg.segment = fieldstr;
               break;

            case COURSE_R:
               htrajseg.course = val;
               break;

            case TURN_CENTER_X_M:
               htrajseg.turns.x_turn = val;
               break;

            case TURN_CENTER_Y_M:
               htrajseg.turns.y_turn = val;
               break;

            case ANGLE_AT_TURN_START_R:
               htrajseg.turns.q_start = Units::UnsignedRadiansAngle(val);
               break;

            case ANGLE_AT_TURN_END_R:
               htrajseg.turns.q_end = Units::UnsignedRadiansAngle(val);
               break;

            case TURN_RADIUS_M:
               htrajseg.turns.radius = Units::MetersLength(val);
               break;

            case LAT_D:
               // unused
               break;

            case LON_D:
               // unused
               break;

            case TURN_CENTER_LAT_D:
               // unused
               break;

            case TURN_CENTER_LON_D:
               // unused
               break;

         }
      }
      m_horizontal_trajectory.push_back(htrajseg);
   }

   file.close();

}


Guidance TrajectoryFromFile::Update(AircraftState state,
                                    Guidance guidance_in) {

   // Sets guidance information based on input guidance and aircraft state.
   // Guidance information set include psi, cross track information, reference altitude,
   // altitude rate, and indicated airspeed.
   //
   // state:Input aircraft state.
   // guidance_in:Input guidance.
   //
   // returns updated guidance.


   Guidance result = guidance_in;

   Units::MetersLength hdtg;
   Units::UnsignedAngle temp_course;

   double h_next;
   double v_next;
   double h_dot_next;


   int curr_index = 0;

   AircraftCalculations::GetPathLengthFromPos(
         Units::FeetLength(state.m_x),
         Units::FeetLength(state.m_y),
         m_horizontal_trajectory,
         hdtg,
         temp_course); // calls the distance to end helper method to calculate the distance to the end of the track

   // if the check if the distance left is <= the start of the precalculated descent distance

   if (hdtg.value() <= fabs(m_vertical_trajectory.mDistToGo.back())) {
      // Get index.

      curr_index = CoreUtils::FindNearestIndex(hdtg.value(), m_vertical_trajectory.mDistToGo);


      // Set _next values.

      if (curr_index == 0) {
         // Below lowest distance-take values at end of route.

         h_next = m_vertical_trajectory.mAlt[curr_index];
         v_next = m_vertical_trajectory.mIas[curr_index];
         h_dot_next = m_vertical_trajectory.mDotAlt[curr_index];

      } else if (curr_index == m_vertical_trajectory.mDistToGo.size()) {
         // Higher than furthest distance-take values at beginning of route.

         // NOTE:this should never occur because of if with hdtg above.

         h_next = m_vertical_trajectory.mAlt[(m_vertical_trajectory.mAlt.size() - 1)];
         v_next = m_vertical_trajectory.mIas[(m_vertical_trajectory.mIas.size() - 1)];
         h_dot_next = m_vertical_trajectory.mDotAlt[(m_vertical_trajectory.mDotAlt.size() - 1)];
      } else {
         // Interpolate values using distance.

         h_next = CoreUtils::LinearlyInterpolate(curr_index, hdtg.value(), m_vertical_trajectory.mDistToGo,
                                                 m_vertical_trajectory.mAlt);
         v_next = CoreUtils::LinearlyInterpolate(curr_index, hdtg.value(), m_vertical_trajectory.mDistToGo,
                                                 m_vertical_trajectory.mIas);
         h_dot_next = CoreUtils::LinearlyInterpolate(curr_index, hdtg.value(), m_vertical_trajectory.mDistToGo,
                                                     m_vertical_trajectory.mDotAlt);
      }


      // Set result

      result.reference_altitude = h_next / FEET_TO_METERS;
      result.altitude_rate = h_dot_next / FEET_TO_METERS;
      result.m_im_speed_command_ias = v_next / FEET_TO_METERS;
   }

   // Calculate Psi command and cross-track error if the aircraft is turning
   // get the distance based on aircraft position

   Units::MetersLength dist_out(0);
   Units::RadiansAngle course_out(0);
   AircraftCalculations::GetPathLengthFromPos(
         Units::FeetLength(state.m_x),
         Units::FeetLength(state.m_y),
         m_horizontal_trajectory, dist_out, course_out);


   // get aircraft position based on that calculated distance

   Units::MetersLength x_pos(0);
   Units::MetersLength y_pos(0);
   int traj_index = 0;
   AircraftCalculations::GetPosFromPathLength(dist_out, m_horizontal_trajectory, x_pos, y_pos, temp_course, traj_index);

   result.psi = course_out.value(); // set the course command result


   // calculate cross track as difference between actual and precalculated position

   double cross_track = sqrt(pow(state.m_x * FEET_TO_METERS - x_pos.value(), 2) + pow(state.m_y * FEET_TO_METERS - y_pos.value(), 2));


   // generate cross-track sign based on distance from turn center and change in course

   double center_dist = sqrt(pow(state.m_x * FEET_TO_METERS - m_horizontal_trajectory[traj_index].turns.x_turn, 2) +
                             pow(state.m_y * FEET_TO_METERS - m_horizontal_trajectory[traj_index].turns.y_turn, 2));


   // calculate the cross track error based on distance from center point and course change if turning

   if (m_horizontal_trajectory[traj_index].segment == "turn") {

      if (center_dist > Units::MetersLength(m_horizontal_trajectory[traj_index].turns.radius).value()) {

         if (course_out > state.GetHeadingInRadiansMathematical()) {
            result.cross_track = cross_track;
         } else {
            result.cross_track = -cross_track;
         }

      } else {
         if (course_out > state.GetHeadingInRadiansMathematical()) {
            result.cross_track = -cross_track;
         } else {
            result.cross_track = cross_track;
         }
      }
      result.use_cross_track = true;
   } else {

      // else do straight track trajectory cross-track calculation

      result.cross_track = -(state.m_y * FEET_TO_METERS - m_horizontal_trajectory[traj_index + 1].y) * cos(course_out) +
                           (state.m_x * FEET_TO_METERS - m_horizontal_trajectory[traj_index + 1].x) * sin(course_out);
   }

   result.use_cross_track = true;

   return result;
}


std::vector<HorizontalPath> TrajectoryFromFile::GetHorizontalData() {

   // Gets horizontal trajectory data.
   //
   // Returns horizontal trajectory data.

   return m_horizontal_trajectory;

}


TrajectoryFromFile::VerticalData TrajectoryFromFile::GetVerticalData() {

   // Gets vertical trajectory data.
   //
   // Returns vertical trajectory data.

   return m_vertical_trajectory;

}


bool TrajectoryFromFile::IsLoaded() {
   return m_loaded;
}


void TrajectoryFromFile::CalculateWaypoints(AircraftIntent &intent) {

   // Sets waypoints and altitude at FAF (final) waypoint based on intent.
   //
   // intent:aircraft intent.


   // set altitude at FAF.

   m_altitude_at_final_approach_fix = Units::MetersLength(intent.GetFms().AltWp[(intent.GetNumberOfWaypoints() - 1)]);


   // Set waypoints.

   m_waypoint.clear(); // empties the waypoint vector for Aircraft Intent Waypoints

   double prev_dist = 0;

   //loop to translate all of the intent waypoints into Precalc Waypoints, works from back to front since precalc starts from the endpoint
   for (int loop = intent.GetNumberOfWaypoints() - 1; loop > 0; loop--) {
      double delta_x = intent.GetFms().xWp[loop - 1].value() - intent.GetFms().xWp[loop].value();
      double delta_y = intent.GetFms().yWp[loop - 1].value() - intent.GetFms().yWp[loop].value();

      // calculate leg distance in meters
      double leg_length = sqrt(pow(delta_x, 2) + pow(delta_y, 2));

      // calculate Psi course
      double course = atan2(delta_y, delta_x);

      PrecalcWaypoint new_waypoint;
      new_waypoint.leg_length = leg_length;
      new_waypoint.course_angle = Units::RadiansAngle(course);
      new_waypoint.name = intent.GetFms().Name[loop];
      new_waypoint.x_pos = intent.GetFms().xWp[loop].value();
      new_waypoint.y_pos = intent.GetFms().yWp[loop].value();

      // add new constraints
      new_waypoint.constraints.constraint_dist = leg_length + prev_dist;
      new_waypoint.constraints.constraint_altHi = intent.GetFms().altHi[loop - 1].value();
      new_waypoint.constraints.constraint_altLow = intent.GetFms().altLow[loop - 1].value();
      new_waypoint.constraints.constraint_speedHi = intent.GetFms().speedHi[loop - 1].value();
      new_waypoint.constraints.constraint_speedLow = intent.GetFms().speedLow[loop - 1].value();

      prev_dist += leg_length;

      m_waypoint.push_back(new_waypoint);
   }

   // add the final waypoint
   PrecalcWaypoint new_waypoint;
   new_waypoint.name = intent.GetFms().Name[0];
   new_waypoint.leg_length = 0;
   new_waypoint.course_angle = m_waypoint.back().course_angle;//0;
   new_waypoint.x_pos = intent.GetFms().xWp[0].value();
   new_waypoint.y_pos = intent.GetFms().yWp[0].value();
   new_waypoint.constraints.constraint_dist = prev_dist;
   new_waypoint.constraints.constraint_altHi = intent.GetFms().altHi[0].value();
   new_waypoint.constraints.constraint_altLow = intent.GetFms().altLow[0].value();
   new_waypoint.constraints.constraint_speedHi = intent.GetFms().speedHi[0].value();
   new_waypoint.constraints.constraint_speedLow = intent.GetFms().speedLow[0].value();
   m_waypoint.push_back(new_waypoint);

}
