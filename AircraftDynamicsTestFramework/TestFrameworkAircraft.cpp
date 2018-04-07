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

#include "framework/TestFrameworkAircraft.h"
#include "math/CustomMath.h"
#include "utility/micros.h"
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
	id = 0;
	start_time = 1;
	mInitialAltitude = Units::FeetLength(0);
	mInitialIas = Units::KnotsSpeed(0);
	mInitialMach = 0.78;
	//mFms = new TestFrameworkFMS();
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
	this->mDynamics = in.mDynamics;
	this->mTargetIntentNotUsed = in.mTargetIntentNotUsed;
	this->precalc_traj = in.precalc_traj;
	this->airborne_app = in.airborne_app;
	this->mFms = in.mFms;
	this->mAircraftControl = in.mAircraftControl;
	this->mInitialAltitude = in.mInitialAltitude;
	this->mInitialIas = in.mInitialIas;
	this->mInitialMach = in.mInitialMach;
}

bool TestFrameworkAircraft::load(DecodedStream *input) {
	set_stream(input);

	//AircraftIntentFromFile aiFromFile;
	//register_loadable_with_brackets("state_model", &state_model, true );
	register_var("initial_altitude", &mInitialAltitude,true );
	register_var("initial_ias", &mInitialIas, true );
	register_var("initial_mach", &mInitialMach, true );
    register_loadable_with_brackets("aircraft_intent", &mAircraftIntent, true);
    register_loadable_with_brackets("dynamics", &mDynamics, false );
	register_loadable_with_brackets("precalc_traj_file", &precalc_traj, true);
	register_loadable_with_brackets("airborne_app", &airborne_app, true);

	bool isloaded = complete(); // read the stream
	if (isloaded) {
		//mAircraftIntent = &aiFromFile;
	}

	return isloaded;
}

void TestFrameworkAircraft::init(Units::Length adsbReceptionRangeThreshold)
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
	const AircraftIntent::Fms &fms(mAircraftIntent.getFms());
	this->truth_state_vector_old.x = Units::FeetLength(fms.xWp[0]).value();
	this->truth_state_vector_old.y = Units::FeetLength(fms.yWp[0]).value();
	this->truth_state_vector_old.z = mInitialAltitude.value();

	//this->state_model.recomputePathLengthLeft(truth_state_vector_old.x*FT_M,truth_state_vector_old.y*FT_M);
    Units::UnsignedRadiansAngle dummyCourse; // not used, but required
    Units::MetersLength distToGo;
    AircraftCalculations::getPathLengthFromPos(
        Units::FeetLength(truth_state_vector_old.x),
        Units::FeetLength(truth_state_vector_old.y),
		precalc_traj.h_traj,
		distToGo, dummyCourse);
	truth_state_vector_old.distToGo = distToGo.value();

	// copy groundspeed components from ThreeDOFDynamics
	truth_state_vector_old.xd = mDynamics.state.xd / FT_M;
	truth_state_vector_old.yd = mDynamics.state.yd / FT_M;
	this->truth_state_vector_old.zd = 0.;

	//ThreeDOFDynamics::Weather weather = mDynamics.getWeatherFromTime(start_time);
	truth_state_vector_old.Vwx = Units::MetersPerSecondSpeed(mDynamics.getVwx()).value();
	truth_state_vector_old.Vwy = Units::MetersPerSecondSpeed(mDynamics.getVwy()).value();

	//Initialize the time in this->truth_state_vector_old with this aircraft's start time:
	this->truth_state_vector_old.time = this->start_time;
}

bool TestFrameworkAircraft::update(const SimulationTime& time)
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
	if(time.get_sim_cycle() <= this->start_time)
	{
		this->truth_state_vector_old.time = time.get_sim_cycle();
		//mTruthAircraftStates.push_back(this->truth_state_vector_old);
		return false;
	}

	Guidance current_guidance;
	AircraftState state_result;

	// update FMS data
	mFms.update(truth_state_vector_old,
			precalc_traj.waypoint_vector,
			precalc_traj.h_traj);

	// TODO ? if( state_model.precalc_traj.is_loaded() )
	{
		// LAW: use measured state to determine guidance
		current_guidance = precalc_traj.update(truth_state_vector_old, current_guidance);
	}

	// if airborne application is loaded check get airborne application guidance
	if( airborne_app.is_loaded() )
	{
		Guidance guidancefromairborneapplication;
		guidancefromairborneapplication = airborne_app.update(
				time,
				mDynamics,
				truth_state_vector_old,
				current_guidance);

		// check if guidance has valid data.
		if ( guidancefromairborneapplication.is_valid() && guidancefromairborneapplication.indicated_airspeed > 0.0)
		{
			// Use the guidance as modified by the airborne appliation
			current_guidance = guidancefromairborneapplication;
		}
	}

	//		fprintf(mGuidanceOut,"%f,%f,%f,%f,%f,%f,%i\n",time.get_current_simulation_time(),current_guidance.indicated_airspeed,current_guidance.reference_altitude,current_guidance.altitude_rate,current_guidance.psi,current_guidance.cross_track,current_guidance.use_cross_track);

	// run aircraft dynamics model to process the next state
	mDynamics.setFms(&mFms);
	state_result = mDynamics.update(this->truth_state_vector_old, current_guidance);
	state_result.id = this->id;
	state_result.time = time.get_sim_cycle();

	//state_model.recomputePathLengthLeft(state_result.x*FT_M,state_result.y*FT_M);
	Units::MetersLength distToGo;
	Units::Angle dummyCourse;
	AircraftCalculations::getPathLengthFromPos(
			  Units::FeetLength(state_result.x),
			  Units::FeetLength(state_result.y),
			  precalc_traj.getHorizontalData(),
			  distToGo, dummyCourse);
	state_result.distToGo = distToGo.value();

	InternalObserver::getInstance()->storeStateModel(state_result,
            mDynamics.state.flapConfig, mDynamics.state.speed_brake,
            mDynamics.state.v_cas.value());

	AircraftState report_state = state_result; // aircraft state reported to ADS-B (adds noise)

	//this->mIsFinished = state_model.is_finished();
	this->mIsFinished = (distToGo.value() < 0);

	// check if the model is finished
	if( this->mIsFinished == true )
	{
		truth_state_vector_old.time = calculate_end_time(truth_state_vector_old);
		//			fclose(mGuidanceOut);
		//			cout << "done" << endl;
		return this->mIsFinished;
	}

	// update aircraft position and add to output list
	this->truth_state_vector_old = state_result;

	return this->mIsFinished;

}

double TestFrameworkAircraft::calculate_end_time(AircraftState vector_in)
{
	// calculate the previous aircraft state
	AircraftState prev_vector = vector_in;
	prev_vector.x -= vector_in.xd;
	prev_vector.y -= vector_in.yd;
	prev_vector.z -= vector_in.zd;

	// get waypoint end coordinates
	AircraftState end_point;
	end_point.x = mFms.xWp[mFms.number_of_waypoints-1];
	end_point.y = mFms.yWp[mFms.number_of_waypoints-1];
	end_point.z = mFms.AltWp[mFms.number_of_waypoints-1];

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


void TestFrameworkAircraft::post_load(Units::Time simulation_time_step, int predictedWindOpt, bool blendWind) {
	// initialize the state model values
/*	if( state_model.is_loaded() )
	{
		state_model.init(id, blendWind, start_time,
				 mAircraftIntent, mTargetIntentNotUsed, mInitialAltitude, mInitialIas, mInitialMach);
	}*/

    // initialize the FMS information
    mFms.copy_waypoints_from_intent(mAircraftIntent); // copy in waypoint data from aircraft intent
    mFms.init(); // initialize the FMS data

    if (precalc_traj.is_loaded())
    {
      precalc_traj.calculateWaypoints(mAircraftIntent);
    }


    // initialize eom dynamics
    const TrajectoryFromFile::VerticalData verticalTrajectoryFromFile = this->precalc_traj.getVerticalData();
    const Units::MetersLength finalAltitude = Units::MetersLength(verticalTrajectoryFromFile.mAlt[0]);
    mDynamics.setFms(&mFms);
    mDynamics.init(precalc_traj.mMassPercentile,
    		finalAltitude,
    		mInitialAltitude,
			mInitialIas,	// FIXME should be TAS
			mInitialMach,
			(double) start_time);
}
