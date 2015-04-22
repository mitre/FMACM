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

#pragma once

#include "DMatrix.h"
#include "AircraftState.h"
#include "HorizontalTraj.h"
#include <vector>
#include <Length.h>
#include <Time.h>
#include <Speed.h>

// Structure of distances in computing path length
// from position.  Data inserted from smallest to
// greatest.


class AircraftCalculations
{

public:
	AircraftCalculations(void);
	~AircraftCalculations(void);

	static double getTemp(double h);
	static void airDensity(double h, double &rho, double &P);
	static Units::Speed CAS2TAS(Units::Speed vcas, Units::Length alt);
	static Units::Speed TAS2CAS(Units::Speed vtas, Units::Length alt);

	// method to get the position and course of an Aircraft based on current distance and precalculated Horizontal Trajectory
	static void getPosFromPathLength(double dist_in, std::vector<HorizontalTraj> &traj_in, double &x_out, double &y_out, double &course_out, int &traj_index);
	// method to get the distance and course of the Aircraft based on the current position and precalculated Horizontal Trajectory

	static void getPathLengthFromPos(double x, double y, std::vector<HorizontalTraj> &hTraj, double &dist, double &trk);

	static double convert0to2Pi(double course_in);
	static double convertPitoPi(double course_in);

	// method for calculating the Cas ESF
	static double ESFconstantCAS(double v_tas, double alt);


 private:

	struct mPathDistance {
	  int mIx; // Index to point in horizontal trajectory.
	  Units::Length mDist;
	};

	static std::vector<mPathDistance> computePathDistances(
	    double x,double y,std::vector<HorizontalTraj> &hTraj);

	static void crossTrackError(double x, double y,
	       int trajIx, std::vector<HorizontalTraj> hTraj,
	       int &nextTrajIx, double &cte);

};
