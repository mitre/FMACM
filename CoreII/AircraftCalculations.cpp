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

#include "AircraftCalculations.h"
#include "constants.h"
#include "micros.h"
#include "DVector.h"
#include "CustomMath.h"
#include <list>
#include <iostream>
#include <stdlib.h>

using namespace std;

AircraftCalculations::AircraftCalculations(void)
{
}

AircraftCalculations::~AircraftCalculations(void)
{
}

//---------------------
// Inputs:
//   altitude, h (meters)
// Outputs: 
//   temperature, T (Kelvin)
double AircraftCalculations::getTemp(double h)
{
//global T0 h_trop T_trop;
double T;
if (h < H_TROP)
    T = T0 - 6.5*h/1000;
else
    T = T_TROP;

return T;
}

// Inputs: 
//   altitude, h (meters)
// Outputs:
//   air density, rho (kg/m^3)
//   air pressure, P (kg/m^2)

void AircraftCalculations::airDensity(double h, double &rho, double &P)
{

	//global h_trop G T0 RHO0 P0 R K_T rho_trop P_trop;

	// Find air temperature (Kelvin), density (kg/m^3), and pressure (kg/m^2)
	double T = getTemp(h);

	if (h < H_TROP)
	{
	    rho = RHO0*pow((T/T0),(-G/(K_T*R)-1));
	    P = P0*pow((T/T0),(-G/(K_T*R)));
	}
	else
	{
		rho = RHO_TROP*exp(-G/(R*T)*(h-H_TROP));
	    P = P_TROP*exp(-G/(R*T)*(h-H_TROP));
	}

}

// Inputs:
//   calibrate air speed, vcas
//   altitude, alt
// Outputs:
//   true air speed, Vtas
Units::Speed AircraftCalculations::CAS2TAS(Units::Speed vcas,Units::Length alt)
{
  // global RHO0 P0 mu;

  // Get the air density
  double rho,p;

  airDensity(Units::MetersLength(alt).value(),rho,p);

  // Terms in the conversion
  double temp1 = 1 + MU/2*(RHO0/P0)*SQR(Units::MetersPerSecondSpeed(vcas).value());

  double temp2 = pow(temp1, (1/MU)) - 1;

  double temp3 = pow((1 + P0/p*temp2), MU);

  double temp4 = 2/MU*p/rho*(temp3-1);

  return Units::MetersPerSecondSpeed(sqrt(temp4));
}



//--------------------------------
// Inputs:
//   true air speed, vtas.
//   altitude, alt.
// Outputs:
//   calibrated air speed.
Units::Speed AircraftCalculations::TAS2CAS(Units::Speed vtas, Units::Length alt)
{
	//global RHO0 P0 MU;

	// Get the air density
	double rho,p;

	airDensity(Units::MetersLength(alt).value(),rho,p);

	// Terms in the conversion
	double temp1 = 1 + MU/2*(rho/p)*SQR(Units::MetersPerSecondSpeed(vtas).value());

	double temp2 = pow(temp1, (1/MU)) - 1;

	double temp3 = pow((1 + (p/P0)*temp2), MU);

	double temp4 = 2/MU*(P0/RHO0)*(temp3 - 1);

	return Units::MetersPerSecondSpeed(sqrt(temp4));
}

// method to get the position and course of an Aircraft based on current distance (in meters) and precalculated Horizontal Trajectory
void AircraftCalculations::getPosFromPathLength(double dist_in, vector<HorizontalTraj> &traj_in, double &x_out, double &y_out, double &course_out, int &traj_index)
{
	int index = 0; // stores the index last position < current distance

	// loop to find the distance
	bool found = false;
	for( int loop = 0; loop < traj_in.size() && found == false; loop++)
	{
		if( dist_in <= traj_in[loop].L )
		{
			found = true;
			index = loop - 1; // sets position to the last position that was less than the given distance

			if( index < 0 )
			{
				index = 0;
			}
		}
	}

	if( found ==  true )
	{
		// calculate position based on if it's a straight or turning path
		if(traj_in[index].segment == "straight")
		{
			double course = traj_in[index].course; // get the course for the given index

			// calculate output values
			x_out = traj_in[index].x + ((dist_in - traj_in[index].L) * cos(course));
			y_out = traj_in[index].y + ((dist_in - traj_in[index].L) * sin(course));
			course_out = course + M_PI; 		
		}
		else if(traj_in[index].segment == "turn")
		{
			double radius = traj_in[index].turns.radius;
			double start = traj_in[index].turns.q_start;
			double end = traj_in[index].turns.q_end;

			// calculate course change between the start and end of turn
			double course_change = convertPitoPi(end - start);
			
			// calculate difference in distance
			double delta = (dist_in - traj_in[index].L)/radius;

			// calculate the theta of the turn
			double theta = start + delta*SIGN(course_change);

			// calculate X and Y positions
			x_out = traj_in[index].turns.x_turn + radius*cos(theta);
			y_out = traj_in[index].turns.y_turn + radius*sin(theta);

			double course1 = AircraftCalculations::convert0to2Pi(theta + M_PI/2.0);
			double course2 = AircraftCalculations::convert0to2Pi(theta - M_PI/2.0);

			if( course1 != (traj_in[index-1].course + M_PI)/* && course1 != (traj_in[index+1].course + M_PI)*/)
			{
				course_out = course1;
			}
			else
			{
				course_out = course2; 
			}
		}
	}

	traj_index = index;
}


void AircraftCalculations::getPathLengthFromPos(double x, double y, vector<HorizontalTraj> &hTraj, double &dist, double &trk) {

  // Computes distance and course based on aircraft position
  // and horizontal trajectory.  Distance is from aircraft
  // to nearest point from the horizontal trajectory along
  // the path.
  //
  // x,y:aircraft position in meters.
  // hTraj:horizontal trajectory of aircraft,
  //       (point positions in meters).
  // dist:computed distance in meters (output).
  // trk:aircraft course to point in radians (output).


  // Dummy values

  dist = -99999.99999;
  trk = 99999.99999;


  // Find closest point

  // Compute Euclidean distances for all horizontal trajectory points
  // and order in ascending sequence.

  vector<mPathDistance> distances = 
    AircraftCalculations::computePathDistances(x,y,hTraj);  


  // Find smallest distance with an acceptable cross track error.

  int nextTrajIx = -1;

  for (int i = 0; ((i < distances.size()) && (nextTrajIx == -1)); i++) {

    double cte;

    int computedNextIx;

    AircraftCalculations::crossTrackError(x,y,
	     distances[i].mIx,hTraj,computedNextIx,cte);

    if (cte <= (2.5*NM_M)) {
      nextTrajIx = computedNextIx;
    }
  }

  if (nextTrajIx == -1) {
    cout << "Trajectory point with acceptable cross track error not found" << endl;
    exit(-37);
  }


  // Calculate DTG and course of aircraft.

  double dap;

  if (hTraj[nextTrajIx].segment == "straight") {

    double d = sqrt(SQR(x-hTraj[nextTrajIx].x) +
		    SQR(y-hTraj[nextTrajIx].y));

    double theta = atan2(hTraj[nextTrajIx].y-y,
			 hTraj[nextTrajIx].x-x);

    trk = hTraj[nextTrajIx].course + PI;

    double deltaTheta = convertPitoPi(theta - trk);

    dap = d * cos(deltaTheta);

  } else if (hTraj[nextTrajIx].segment == "turn") {

    double theta = atan2(y-hTraj[nextTrajIx].turns.y_turn,
			 x-hTraj[nextTrajIx].turns.x_turn);

    double deltaTheta = convertPitoPi(hTraj[nextTrajIx].turns.q_start-theta);

    dap = hTraj[nextTrajIx].turns.radius * fabs(deltaTheta);

    trk = convert0to2Pi((hTraj[nextTrajIx-1].course + PI) - deltaTheta);

  } else {

    cout << "Non straight non turn segment in getPathLengthFromPos" << endl;
    exit(-43);

  }

  dist = dap + hTraj[nextTrajIx].L;

} // getPathLengthFromPos


vector<AircraftCalculations::mPathDistance> AircraftCalculations::computePathDistances(
   double x,double y,vector<HorizontalTraj> &hTraj) {

  // Computes distances for a position along the horiontal
  // trajectory.
  //
  // x,y:position in meters.
  // hTraj:horizontal trajectory-containing points in meters.
  // returns distances and cross track errors for all points in a vector
  //         ordered ascending by distance.

  vector<mPathDistance> pdVect;

  for (int i=0; i<hTraj.size(); i++) {

    mPathDistance pd;

    double d = sqrt(SQR(x-hTraj[i].x) + SQR(y-hTraj[i].y));
    pd.mDist = Units::MetersLength(d);
    pd.mIx = i;

    if (isnan(d)) {
      cout << "undefined distance computed" << endl;
      if (isnan(x)) {
	cout << "undefined x" << endl;
      }
      if (isnan(hTraj[i].x)) {
	cout << "undefined trajectory x at point " << i << endl;
      }
      if (isnan(y)) {
	cout << "undefined y" << endl;
      }
      if (isnan(hTraj[i].y)) {
	cout << "undefined trajectory y at point " << i << endl;
      }
    }

    if (pdVect.empty()) {
      pdVect.push_back(pd);
    } else {

      vector<mPathDistance>::iterator iter = pdVect.begin();

      while (iter < pdVect.end()) {

	if ((*iter).mDist > pd.mDist) {
	  break;
	}

	iter++;
      }

      pdVect.insert(iter, pd);
    }
  }

  return pdVect;

}


// coverts angle into a range from 0 to 2PI (0 to 360 degrees)
double AircraftCalculations::convert0to2Pi(double course_in)
{
	double result = course_in;

	while( result >= 2.0*M_PI )
	{
		result -= 2.0*M_PI; 
	}
	while( result < 0 )
	{
		result += 2.0*M_PI;
	}

	return result;
}

// coverts angle into a range from -PI to PI (-180 to 180 degrees)
double AircraftCalculations::convertPitoPi(double course_in)
{
	double result = course_in;

	while( result >= M_PI )
	{
		result -= 2.0*M_PI; 
	}
	while( result < -M_PI )
	{
		result += 2.0*M_PI;
	}

	return result;
}


// method for calculating the Cas ESF
//double v_tas: true airspeed (m/s)
//double alt: altitude (meters)
double AircraftCalculations::ESFconstantCAS(double v_tas, double alt)
{
	double esf;
	double temperature;
	double mach;
	double temp1, temp2, temp3;

	temperature = AircraftCalculations::getTemp(alt);
	mach = v_tas/sqrt(GAMMA*R*temperature);

	temp1 = 1.0 + (GAMMA-1.0)/2*SQR(mach);
	temp2 = (pow(temp1,(-1.0/(GAMMA-1))))*(pow(temp1,(GAMMA/(GAMMA-1))) - 1.0);

	if( alt <= H_TROP)
	{
		temp3 = 1.0 + (GAMMA*R*K_T)/(2*G) * SQR(mach) + temp2;
	}
	else
	{
		temp3 = 1.0 + temp2;
	}

	esf = 1.0/temp3;
	return esf;
}


void AircraftCalculations::crossTrackError(
    double x, double y, int trajIx, 
    vector<HorizontalTraj> hTraj,
    int &nextTrajIx, double &cte) {

  // Computes cross track error for a position relative to a
  // horizontal trajectory and next trajectory point.
  //
  // x,y:aircraft position in meters.
  // trajIx:index of trajectory point.
  // hTraj:horizontal trajectory for aircraft; positions, turn radius
  //       in meters; course in radians.  Segment type, course, and 
  //       position are used from here.
  // nextTrajIx:downstream trajectory point (output).
  // cte:cross track error in meters (output).


  // dummy values

  cte = 99999.99999;
  nextTrajIx = -99999;


  // Compute course

  double course;

  if (hTraj[trajIx].segment == "turn") {

    // Compute cross track error for turn.

    if (trajIx == 0) {
      cout << "Illegal condition computing cross track error." << endl;
      cout << "First trajectory point cannot be for a turn." << endl;
      exit(-39);
    }

    course = convert0to2Pi(hTraj[trajIx-1].course+PI);

  } else {

    course = convert0to2Pi(hTraj[trajIx].course+PI);
  }


  // Compute angle closest to x, y point.

  double theta = convert0to2Pi(atan2(y-hTraj[trajIx].y,x-hTraj[trajIx].x));


  // Compute quadrant

  /* Quadrant calculation models this:

                              |
                              |
                              |
                              x wp 3
                              |
                              |
                              |
			      /
                             / 
                            / (turn)
            4 | 3          /
     _________x___________/
            1 | 2
             wp 2

  */


  double courseLow = course;
  double courseHigh = course + PI_OVER_TWO;

  int quad = -1;

  for (int i = 1; ((i <= 4) && (quad==-1)); i++) {
    double delta1 = convertPitoPi(theta-courseLow);
    double delta2 = convertPitoPi(theta-courseHigh);

    if ((fabs(delta1) <= PI_OVER_TWO) && (fabs(delta2) <= PI_OVER_TWO)) {
      quad = i;
    }

    courseLow += PI_OVER_TWO;
    courseHigh += PI_OVER_TWO;
  }

  if (quad == -1) {
    cout << "Bad computation of quadrant of point relative to turn segment."
	 << endl;
    exit (-38);
  }


  // Get next downstream point

  if ((hTraj[trajIx].segment == "turn") &&
      ((quad == 1) || (quad == 4))) {

    // Next segment
    nextTrajIx = trajIx - 1;

  } else if ((hTraj[trajIx].segment == "turn") &&
      ((quad == 2) || (quad == 3))) {
    
    // Current segment
    nextTrajIx = trajIx;

  } else if ((hTraj[trajIx].segment == "straight") &&
      ((quad == 1) || (quad == 4))) {

    if (trajIx == 0) {

      // First segment
      nextTrajIx = trajIx;

    } else {

      // Next segment
      nextTrajIx = trajIx - 1;
    }

  } else if ((hTraj[trajIx].segment == "straight") &&
	     ((quad == 2) || (quad == 3))) {

    // Current segment
    nextTrajIx = trajIx;

  } else {

    // Error

    cout << "Invalid segment condition found in crossTrackError" << endl;

    exit(-44);

  }


  // Compute cte

  if (hTraj[nextTrajIx].segment == "straight") {

    course = convert0to2Pi(hTraj[nextTrajIx].course + PI);
    theta = convert0to2Pi(atan2(y-hTraj[nextTrajIx].y,x-hTraj[nextTrajIx].x));

    double dist = sqrt(SQR(x-hTraj[nextTrajIx].x) +
		       SQR(y-hTraj[nextTrajIx].y));

    double deltaTheta = convertPitoPi(theta-course);

    cte = fabs(dist*sin(deltaTheta));

  } else {

    double r1 = sqrt(SQR(x - hTraj[nextTrajIx].turns.x_turn) +
		     SQR(y - hTraj[nextTrajIx].turns.y_turn));
    cte = fabs(r1 - hTraj[nextTrajIx].turns.radius);

  }

} // crossTrackError
