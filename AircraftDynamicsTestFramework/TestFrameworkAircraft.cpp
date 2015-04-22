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

/*
 * TestFrameworkAircraft.cpp
 *
 *  Created on: Mar 2, 2015
 *      Author: sbowman
 */

#include "TestFrameworkAircraft.h"
#include "CustomMath.h"
#include <time.h>
#include "Waypoint.h"
#include "AircraftCalculations.h"
#include "Guidance.h"
#include <iostream>
#include <stdio.h>

using namespace std;

//FILE *TestFrameworkAircraft::mGuidanceOut = fopen("testframework-guidance-output.csv","w");

TestFrameworkAircraft::TestFrameworkAircraft() {
	// Set some default for the parameters that this class will NOT load
	id = 0;
	start_time = 1;
	mTargetIntentNotUsed = NULL;
	mInitialAltitude = Units::FeetLength(0);
	mInitialIas = Units::KnotsSpeed(0);
	mInitialMach = 0.78;
}

TestFrameworkAircraft::~TestFrameworkAircraft() {
	// destructor stub
}

TestFrameworkAircraft::TestFrameworkAircraft(const TestFrameworkAircraft &in)
{
	copy(in);
}

TestFrameworkAircraft& TestFrameworkAircraft::operator=(const TestFrameworkAircraft &in)
{
  if(this != &in) {
    copy(in);
  }
  return *this;
}

// helper method for copying values safely, used by copy constructor and operator=
void TestFrameworkAircraft::copy(const TestFrameworkAircraft &in)
{
	this->start_time = in.start_time;
	this->id = in.id;
	this->truth_state_vector_old = in.truth_state_vector_old;
	this->mIsFinished = in.mIsFinished;
	this->mAircraftIntent = in.mAircraftIntent;
	this->mTargetIntentNotUsed = in.mTargetIntentNotUsed;
	this->state_model = in.state_model;
	this->mPredictedWindX = in.mPredictedWindX;
	this->mPredictedWindY = in.mPredictedWindY;
	this->mInitialAltitude = in.mInitialAltitude;
	this->mInitialIas = in.mInitialIas;
	this->mInitialMach = in.mInitialMach;
}

bool TestFrameworkAircraft::load(DecodedStream *input) {
	set_stream(input);

	AircraftIntentFromFile aiFromFile;
	register_loadable_with_brackets("aircraft_intent", &aiFromFile, true);
	register_loadable_with_brackets("state_model", &state_model, true );
	register_var("initial_altitude", &mInitialAltitude,true );
	register_var("initial_ias", &mInitialIas, true );
	register_var("initial_mach", &mInitialMach, true );

	bool isloaded = complete(); // read the stream
	if (isloaded) {
		mAircraftIntent = &aiFromFile;
	}

	return isloaded;
}

void TestFrameworkAircraft::init(double& seed)
{
	//Initialize this->navigation_measurement_old with info in first waypoint and other info:
	init_truth_state_vector_old();

	//This aircraft has not finished simulation yet:
	this->mIsFinished = false;
}

void TestFrameworkAircraft::init_truth_state_vector_old(void)
{
	this->truth_state_vector_old.id = this->id;
	//Initialize this->truth_state_vector_old with info in first waypoint:
	this->truth_state_vector_old.x = this->state_model.Fms.xWp[0];
	this->truth_state_vector_old.y = this->state_model.Fms.yWp[0];
	const Units::Length initialAltitude = Units::MetersLength(mInitialAltitude);
	const Units::Speed initialIas = Units::MetersPerSecondSpeed(mInitialIas);
	this->truth_state_vector_old.z = Units::FeetLength(initialAltitude).value();
	this->state_model.recomputePathLengthLeft(truth_state_vector_old.x*FT_M,truth_state_vector_old.y*FT_M);
	truth_state_vector_old.distToGo = this->state_model.pathLengthLeft;

	// copy groundspeed components from ThreeDOFDynamics
	truth_state_vector_old.xd = state_model.dynamics->state.xd / FT_M;
	truth_state_vector_old.yd = state_model.dynamics->state.yd / FT_M;
	this->truth_state_vector_old.zd = 0.;

	ThreeDOFDynamics::Weather weather = state_model.dynamics->getWeatherFromTime(start_time);
	truth_state_vector_old.Vwx = Units::MetersPerSecondSpeed(weather.Vwx).value();
	truth_state_vector_old.Vwy = Units::MetersPerSecondSpeed(weather.Vwy).value();

	//Initialize the time in this->truth_state_vector_old with this aircraft's start time:
	this->truth_state_vector_old.time = this->start_time;
}

bool TestFrameworkAircraft::update(SimulationTime& time,
		State& aircraft_truth_state,
		double& seed
	)
{
	/*
	 *  In this very simple implementation, we only want to drive the
	 *  aircraft dynamics and airborne application. All other objects
	 *  are ignored.
	 */


	//If finished, this aircraft does nothing.
	if(this->mIsFinished)
	{
		return true;
	}

	//It is not time for this aircraft to start yet:
	//This aircraft is at the initial waypoint at its start_time. Its simulation starts at
	//its (start_time + simulation_time_step).
	if(time.get_current_simulation_time() <= this->start_time)
	{
		this->truth_state_vector_old.time = time.get_current_simulation_time();
		aircraft_truth_state.push_back(this->truth_state_vector_old);
		return false;
	}

	// if checks if there is a trajectory model, if not don't do new state calculations
	// run the state model if it has been loaded
	if( state_model.is_loaded() == true && state_model.is_finished() == false)
	{
		Guidance current_guidance;
		AircraftState state_result;

		// update FMS data
		state_model.Fms.update(truth_state_vector_old, state_model.precalc_traj.waypoint_vector,
				       state_model.precalc_traj.h_traj);

		if( state_model.precalc_traj.is_loaded() )
		{
			// LAW: use measured state to determine guidance
			current_guidance = state_model.precalc_traj.update(truth_state_vector_old, current_guidance);
		}

		// if airborne application is loaded check get airborne application guidance
		if( state_model.airborne_app.is_loaded() )
		{
			Guidance guidancefromairborneapplication;
			guidancefromairborneapplication = state_model.airborne_app.update(state_model.dynamics,
					truth_state_vector_old,
				// received_ADS_B_reports_list,
				current_guidance,
				mPredictedWindX,mPredictedWindY,
				seed);

			// check if guidance has valid data.
			if ( guidancefromairborneapplication.is_valid() && guidancefromairborneapplication.indicated_airspeed > 0.0)
			{
				// Use the guidance as modified by the airborne appliation
				current_guidance = guidancefromairborneapplication;
			}
		}

//		fprintf(mGuidanceOut,"%f,%f,%f,%f,%f,%f,%i\n",time.get_current_simulation_time(),current_guidance.indicated_airspeed,current_guidance.reference_altitude,current_guidance.altitude_rate,current_guidance.psi,current_guidance.cross_track,current_guidance.use_cross_track);

		// run aircraft dynamics model to process the next state
		state_model.dynamics->setFms(&state_model.Fms);
		state_result = state_model.dynamics->update(this->truth_state_vector_old, current_guidance, seed);
		state_result.id = this->id;
		state_result.time = time.get_current_simulation_time();

		state_model.recomputePathLengthLeft(state_result.x*FT_M,state_result.y*FT_M);

		state_result.distToGo = state_model.pathLengthLeft;

		AircraftState report_state = state_result; // aircraft state reported to ADS-B (adds noise)

		this->mIsFinished = state_model.is_finished();

		// check if the model is finished
		if( this->mIsFinished == true )
		{
			truth_state_vector_old.time = calculate_end_time(truth_state_vector_old, state_model.Fms);
//			fclose(mGuidanceOut);
//			cout << "done" << endl;
			return this->mIsFinished;
		}

		// update aircraft position and add to output list
		this->truth_state_vector_old = state_result;
		aircraft_truth_state.push_back(report_state);
	}

	return this->mIsFinished;

}

double TestFrameworkAircraft::calculate_end_time(AircraftState vector_in, FlightManagementSystem fms)
{
	// calculate the previous aircraft state
	AircraftState prev_vector = vector_in;
	prev_vector.x -= vector_in.xd;
	prev_vector.y -= vector_in.yd;
	prev_vector.z -= vector_in.zd;

	// get waypoint end coordinates
	AircraftState end_point;
	end_point.x = fms.xWp[fms.number_of_waypoints-1];
	end_point.y = fms.yWp[fms.number_of_waypoints-1];
	end_point.z = fms.AltWp[fms.number_of_waypoints-1];

	// translate origin point to the previous aircraft state
	// translate end point
	vector_in.x -= prev_vector.x;
	vector_in.y -= prev_vector.y;
	vector_in.z -= prev_vector.z;
	// translate last waypoint
	end_point.x -= prev_vector.x;
	end_point.y -= prev_vector.y;
	end_point.z -= prev_vector.z;
	// translate previous position to origin
	prev_vector.x -= prev_vector.x;
	prev_vector.y -= prev_vector.y;
	prev_vector.z -= prev_vector.z;

	// calculate the angle from the previous position to the final position
	double aircraft_angle = atan2(vector_in.y, vector_in.x);

	// calculate the angle from the previous position to the end waypoint
	double waypoint_angle = atan2(end_point.y, end_point.x);

	// calculate the angle of the nearest angle
	double nearest_angle = subtract_headings(aircraft_angle, waypoint_angle);

	// calculate the distance between the previous aircraft point and end waypoint
	double dist_waypoint = sqrt(SQR(end_point.x) + SQR(end_point.y));

	// calculate the distance between the previous aircraft point and the aircraft end point
	double dist_aircraft = sqrt(SQR(vector_in.x) + SQR(vector_in.y));

	// calculate distance to closest point between the previous and end point
	double dist_closest = dist_waypoint * cos(nearest_angle);

	// calculates the ratio of distance to the closest point compared to the distance to the aircraft end point
	double ratio = dist_closest / dist_aircraft;

	double end_time = vector_in.time - 1.0 + ratio; // sets the end time as given time -1 + distance ratio

	return end_time;
}


void TestFrameworkAircraft::post_load(double simulation_time_step, int predictedWindOpt, bool blendWind) {
	// initialize the state model values
	if( state_model.is_loaded() )
	{
		state_model.init(id, blendWind, start_time,
				 mAircraftIntent, mTargetIntentNotUsed, mInitialAltitude, mInitialIas, mInitialMach);
	}
}
