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

  Units::MetersLength hdtg;
  Units::UnsignedAngle temp_course;

  double h_next;
  double v_next;
  double h_dot_next;


  int curr_index = 0;

  AircraftCalculations::getPathLengthFromPos(
		  Units::FeetLength(state.x),
		  Units::FeetLength(state.y),
		  h_traj,
		  hdtg,
		  temp_course); // calls the distance to end helper method to calculate the distance to the end of the track

  // if the check if the distance left is <= the start of the precalculated descent distance

  if( hdtg.value() <= fabs(v_traj.mDistToGo.back()))
    {
      // Get index.

      curr_index = CoreUtils::nearestIndex(curr_index,hdtg.value(),v_traj.mDistToGo);


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

	  h_next=CoreUtils::interpolate(curr_index,hdtg.value(), v_traj.mDistToGo, v_traj.mAlt);
	  v_next=CoreUtils::interpolate(curr_index,hdtg.value(), v_traj.mDistToGo, v_traj.mIas);
	  h_dot_next=CoreUtils::interpolate(curr_index,hdtg.value(), v_traj.mDistToGo, v_traj.mDotAlt);
	}


      // Set result

      result.reference_altitude = h_next / FT_M;
      result.altitude_rate = h_dot_next / FT_M;
      result.m_im_speed_command_ias = v_next / FT_M;
    }
  
  // Calculate Psi command and cross-track error if the aircraft is turning
  // get the distance based on aircraft position

  Units::MetersLength dist_out(0);
  Units::RadiansAngle course_out(0);
  AircraftCalculations::getPathLengthFromPos(
		  Units::FeetLength(state.x),
		  Units::FeetLength(state.y),
		  h_traj, dist_out, course_out);


  // get aircraft position based on that calculated distance 

  Units::MetersLength x_pos(0);
  Units::MetersLength y_pos(0);
  int traj_index = 0;
  AircraftCalculations::getPosFromPathLength(dist_out, h_traj, x_pos, y_pos, temp_course, traj_index);

  result.psi = course_out.value(); // set the course command result


  // calculate cross track as difference between actual and precalculated position

  double cross_track = sqrt( pow(state.x*FT_M - x_pos.value(), 2) + pow(state.y*FT_M - y_pos.value(), 2) );
 

  // generate cross-track sign based on distance from turn center and change in course

  double center_dist = sqrt( pow(state.x*FT_M - h_traj[traj_index].turns.x_turn, 2) + pow(state.y*FT_M - h_traj[traj_index].turns.y_turn, 2) );


  // calculate the cross track error based on distance from center point and course change if turning

  if (h_traj[traj_index].segment == "turn") {

    if (center_dist > Units::MetersLength(h_traj[traj_index].turns.radius).value()) {

      if (course_out > state.get_heading_in_radians_mathematical()) {
	result.cross_track = cross_track; 
      } else {
	result.cross_track = -cross_track;
      }

    } else {
      if (course_out > state.get_heading_in_radians_mathematical()) {
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


std::vector<HorizontalPath> TrajectoryFromFile::getHorizontalData(void) {

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

  mAltAtFAF = Units::MetersLength(intent.getFms().AltWp[(intent.getNumberOfWaypoints()-1)]);


  // Set waypoints.

  waypoint_vector.clear(); // empties the waypoint vector for Aircraft Intent Waypoints

  double prev_dist = 0;

  //loop to translate all of the intent waypoints into Precalc Waypoints, works from back to front since precalc starts from the endpoint
  for( int loop = intent.getNumberOfWaypoints()-1; loop > 0; loop--)
    {
      double delta_x = intent.getFms().xWp[loop-1].value() - intent.getFms().xWp[loop].value();
      double delta_y = intent.getFms().yWp[loop-1].value() - intent.getFms().yWp[loop].value();

      // calculate leg distance in meters
      double leg_length = sqrt(pow(delta_x, 2) + pow(delta_y, 2));
		
      // calculate Psi course
      double course = atan2( delta_y,  delta_x );

      PrecalcWaypoint new_waypoint;
      new_waypoint.leg_length = leg_length;
      new_waypoint.course_angle = Units::RadiansAngle(course);
      new_waypoint.name = intent.getFms().Name[loop];
      new_waypoint.x_pos = intent.getFms().xWp[loop].value();
      new_waypoint.y_pos = intent.getFms().yWp[loop].value();

      // add new constraints
      new_waypoint.constraints.constraint_dist = leg_length + prev_dist;
      new_waypoint.constraints.constraint_altHi = intent.getFms().altHi[loop-1].value();
      new_waypoint.constraints.constraint_altLow = intent.getFms().altLow[loop-1].value();
      new_waypoint.constraints.constraint_speedHi = intent.getFms().speedHi[loop-1].value();
      new_waypoint.constraints.constraint_speedLow = intent.getFms().speedLow[loop-1].value();

      prev_dist += leg_length;

      waypoint_vector.push_back(new_waypoint);
    }

  // add the final waypoint
  PrecalcWaypoint new_waypoint;
  new_waypoint.name = intent.getFms().Name[0];
  new_waypoint.leg_length = 0;
  new_waypoint.course_angle = waypoint_vector.back().course_angle;//0;
  new_waypoint.x_pos = intent.getFms().xWp[0].value();
  new_waypoint.y_pos = intent.getFms().yWp[0].value();
  new_waypoint.constraints.constraint_dist = prev_dist;
  new_waypoint.constraints.constraint_altHi = intent.getFms().altHi[0].value();
  new_waypoint.constraints.constraint_altLow = intent.getFms().altLow[0].value();
  new_waypoint.constraints.constraint_speedHi = intent.getFms().speedHi[0].value();
  new_waypoint.constraints.constraint_speedLow = intent.getFms().speedLow[0].value();
  waypoint_vector.push_back(new_waypoint);

}
