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
// Copyright 2015 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#include "TrajectoryFromFile.h"
#include "AircraftCalculations.h"
#include "IMUtils.h"
#include <CsvParser.h>
#include "micros.h"


TrajectoryFromFile::TrajectoryFromFile(void) {

  v_traj.mTimeToGo.clear();
  v_traj.mDistToGo.clear();
  v_traj.mAlt.clear();
  v_traj.mIas.clear();
  v_traj.mDotAlt.clear();

  h_traj.clear();

  mVerticalTrajectoryFile = "";
  mHorizontalTrajectoryFile = "";

  mMassPercentile = 0.5; // Dave Elliott says we can hard-code this for the tests
  mAltAtFAF = Units::FeetLength(-50.0);

  mLoaded = false;
}

TrajectoryFromFile::~TrajectoryFromFile(void) {}

bool TrajectoryFromFile::load(DecodedStream *input) {

  set_stream(input);

  register_var("vfp_csv_file", &mVerticalTrajectoryFile);
  register_var("hfp_csv_file", &mHorizontalTrajectoryFile);

  mLoaded = complete();

  if (mLoaded) {
    readVerticalTrajectoryFile();
    readHorizontalTrajectoryFile();
  }

  return mLoaded;

}


void TrajectoryFromFile::readVerticalTrajectoryFile(void) {
  
  std::ifstream file(mVerticalTrajectoryFile.c_str());

  if (!file.is_open()) {
    std::cout << "Vertical trajectory file " << mVerticalTrajectoryFile.c_str() << " not found" << std::endl;
    exit(-20);
  }

  int numhdrs = 0;

  for (CsvParser::CsvIterator csviter(file);csviter != CsvParser::CsvIterator();++csviter) {

    if (numhdrs <  1) {
      numhdrs++;
      continue;
    }

    if ((int) (*csviter).size() != NUM_VERTICAL_TRAJ_FIELDS) {
      std::cout << "Bad number of fields found in " << mVerticalTrajectoryFile.c_str()
		<< std::endl << "vertical trajectory file.  Found " << (int) (*csviter).size()
		<< " fields expected " << (int) NUM_VERTICAL_TRAJ_FIELDS << " fields." << std::endl;
      exit(-21);
    }

    for (int vfield = TIME_TO_GO_SEC; vfield != NUM_VERTICAL_TRAJ_FIELDS; vfield++) {
      std::string fieldstr = (*csviter)[vfield];

      double val = atof(fieldstr.c_str());

      switch (static_cast<VerticalFields>(vfield)) {

        case TIME_TO_GO_SEC:
	  v_traj.mTimeToGo.push_back(val);
	  break;

        case DISTANCE_TO_GO_VERT_M:
	  v_traj.mDistToGo.push_back(val);
	  break;

        case ALTITUDE_M:
	  v_traj.mAlt.push_back(val);
	  break;

        case IAS_MPS:
	  v_traj.mIas.push_back(val);
	  break;

        case DOT_ALTITUDE_MPS:
	  v_traj.mDotAlt.push_back(val);
	  break;
      }
    }
  }

  file.close();

}


void TrajectoryFromFile::readHorizontalTrajectoryFile(void) {

  std::ifstream file(mHorizontalTrajectoryFile.c_str());

  if (!file.is_open()) {
    std::cout << "Horizontal trajectory file " << mHorizontalTrajectoryFile.c_str() << " not found" << std::endl;
    exit(-22);
  }

  int numhdrs = 0;

  for (CsvParser::CsvIterator csviter(file);csviter != CsvParser::CsvIterator();++csviter) {

    if (numhdrs < 2) {
      numhdrs++;
      continue;
    }

    if ((int) (*csviter).size() != (int) NUM_HORIZONTAL_TRAJ_FIELDS) {
      std::cout << "Bad number of fields found in " << mHorizontalTrajectoryFile.c_str()
		<< std::endl << "vertical trajectory file.  Found " << (int) (*csviter).size()
		<< " fields expected " << (int) NUM_HORIZONTAL_TRAJ_FIELDS << " fields." << std::endl;
      exit(-38);
    }

    HorizontalTraj htrajseg;

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
	  htrajseg.turns.q_start = val;
	  break;

        case ANGLE_AT_TURN_END_R:
	  htrajseg.turns.q_end = val;
	  break;

        case TURN_RADIUS_M:
	  htrajseg.turns.radius = val;
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
    h_traj.push_back(htrajseg);
  }

  file.close();

}


Guidance TrajectoryFromFile::update(AircraftState state, Guidance guidance_in) {

  // Sets guidance information based on input guidance and aircraft state.
  // Guidance information set include psi, cross track information, reference altitude,
  // altitude rate, and indicated airspeed.
  //
  // state:Input aircraft state.
  // guidance_in:Input guidance.
  //
  // returns updated guidance.


  Guidance result = guidance_in;

  double hdtg;
  double temp_course;

  double h_next;
  double v_next;
  double h_dot_next;


  int curr_index = 0;

  AircraftCalculations::getPathLengthFromPos(state.x*FT_M, state.y*FT_M, h_traj, hdtg, temp_course); // calls the distance to end helper method to calculate the distance to the end of the track

  // if the check if the distance left is <= the start of the precalculated descent distance

  if( hdtg <= fabs(v_traj.mDistToGo.back()))
    {
      // Get index.

      curr_index = IMUtils::IM_index(curr_index,hdtg,v_traj.mDistToGo);


      // Set _next values.

      if (curr_index == 0)
	{
	  // Below lowest distance-take values at end of route.

	  h_next = v_traj.mAlt[curr_index];
	  v_next = v_traj.mIas[curr_index];
	  h_dot_next = v_traj.mDotAlt[curr_index];
	  
	}
      else if (curr_index == v_traj.mDistToGo.size())
	{
	  // Higher than furthest distance-take values at beginning of route.

	  // NOTE:this should never occur because of if with hdtg above.

	  h_next = v_traj.mAlt[(v_traj.mAlt.size()-1)];
	  v_next = v_traj.mIas[(v_traj.mIas.size()-1)];
	  h_dot_next = v_traj.mDotAlt[(v_traj.mDotAlt.size()-1)];
	}
      else
	{
	  // Interpolate values using distance.

	  h_next=IMUtils::interp(curr_index,hdtg, v_traj.mDistToGo, v_traj.mAlt);
	  v_next=IMUtils::interp(curr_index,hdtg, v_traj.mDistToGo, v_traj.mIas);
	  h_dot_next=IMUtils::interp(curr_index,hdtg, v_traj.mDistToGo, v_traj.mDotAlt);
	}


      // Set result

      result.reference_altitude = h_next / FT_M;
      result.altitude_rate = h_dot_next / FT_M;
      result.indicated_airspeed = v_next / FT_M;
    }
  
  // Calculate Psi command and cross-track error if the aircraft is turning
  // get the distance based on aircraft position

  double dist_out = 0;
  double course_out = 0;
  AircraftCalculations::getPathLengthFromPos(state.x*FT_M, state.y*FT_M, h_traj, dist_out, course_out); 


  // get aircraft position based on that calculated distance 

  double x_pos = 0;
  double y_pos = 0;
  int traj_index = 0;
  AircraftCalculations::getPosFromPathLength(dist_out, h_traj, x_pos, y_pos, temp_course, traj_index);

  course_out = AircraftCalculations::convert0to2Pi(course_out); // converts course to 0 to 2PI
  result.psi = course_out; // set the course command result


  // calculate cross track as difference between actual and precalculated position

  double cross_track = sqrt( SQR(state.x*FT_M - x_pos) + SQR(state.y*FT_M - y_pos) );
 

  // generate cross-track sign based on distance from turn center and change in course

  double center_dist = sqrt( SQR(state.x*FT_M - h_traj[traj_index].turns.x_turn) + SQR(state.y*FT_M - h_traj[traj_index].turns.y_turn) );


  // calculate the cross track error based on distance from center point and course change if turning

  if (h_traj[traj_index].segment == "turn") {

    if (center_dist > h_traj[traj_index].turns.radius) {

      if (course_out > state.get_heading_in_radians_mathmatical()) {
	result.cross_track = cross_track; 
      } else {
	result.cross_track = -cross_track;
      }

    } else {
      if (course_out > state.get_heading_in_radians_mathmatical()) {
	result.cross_track = -cross_track;
      } else {
	result.cross_track = cross_track; 
      }
    }
    result.use_cross_track = true;
  } else {

    // else do straight track trajectory cross-track calculation

    result.cross_track = -(state.y*FT_M - h_traj[traj_index+1].y)*cos(course_out) + (state.x*FT_M - h_traj[traj_index+1].x)*sin(course_out);
  }

  result.use_cross_track = true;

  return result;
}


std::vector<HorizontalTraj> TrajectoryFromFile::getHorizontalData(void) {

  // Gets horizontal trajectory data.
  //
  // Returns horizontal trajectory data.

  return this->h_traj;

}


TrajectoryFromFile::VerticalData TrajectoryFromFile::getVerticalData(void) {

  // Gets vertical trajectory data.
  //
  // Returns vertical trajectory data.

  return this->v_traj;

}


bool TrajectoryFromFile::is_loaded(void) {
  return mLoaded;
}


void TrajectoryFromFile::calculateWaypoints(AircraftIntent &intent) {

  // Sets waypoints and altitude at FAF (final) waypoint based on intent.
  //
  // intent:aircraft intent.


  // set altitude at FAF.

  mAltAtFAF = Units::MetersLength(intent.Fms.AltWp[(intent.number_of_waypoints-1)]);


  // Set waypoints.

  waypoint_vector.clear(); // empties the waypoint vector for Aircraft Intent Waypoints

  double prev_dist = 0;

  //loop to translate all of the intent waypoints into Precalc Waypoints, works from back to front since precalc starts from the endpoint
  for( int loop = intent.number_of_waypoints-1; loop > 0; loop--)
    {
      double delta_x = intent.Fms.xWp[loop-1] - intent.Fms.xWp[loop];
      double delta_y = intent.Fms.yWp[loop-1] - intent.Fms.yWp[loop];

      // calculate leg distance in meters
      double leg_length = sqrt(SQR(delta_x) + SQR(delta_y));
		
      // calculate Psi course
      double course = atan2( delta_y,  delta_x );

      PrecalcWaypoint new_waypoint;
      new_waypoint.leg_length = leg_length;
      new_waypoint.course_angle = course;
      new_waypoint.x_pos = intent.Fms.xWp[loop];
      new_waypoint.y_pos = intent.Fms.yWp[loop];

      // add new constraints
      new_waypoint.constraints.constraint_dist = leg_length + prev_dist;
      new_waypoint.constraints.constraint_altHi = intent.Fms.altHi[loop-1];
      new_waypoint.constraints.constraint_altLow = intent.Fms.altLow[loop-1];
      new_waypoint.constraints.constraint_speedHi = intent.Fms.speedHi[loop-1];
      new_waypoint.constraints.constraint_speedLow = intent.Fms.speedLow[loop-1];

      prev_dist += leg_length;

      waypoint_vector.push_back(new_waypoint);
    }

  // add the final waypoint
  PrecalcWaypoint new_waypoint;
  new_waypoint.leg_length = 0;
  new_waypoint.course_angle = waypoint_vector.back().course_angle;//0;
  new_waypoint.x_pos = intent.Fms.xWp[0];
  new_waypoint.y_pos = intent.Fms.yWp[0];
  new_waypoint.constraints.constraint_dist = prev_dist;
  new_waypoint.constraints.constraint_altHi = intent.Fms.altHi[0];
  new_waypoint.constraints.constraint_altLow = intent.Fms.altLow[0];
  new_waypoint.constraints.constraint_speedHi = intent.Fms.speedHi[0];
  new_waypoint.constraints.constraint_speedLow = intent.Fms.speedLow[0];
  waypoint_vector.push_back(new_waypoint);

}
