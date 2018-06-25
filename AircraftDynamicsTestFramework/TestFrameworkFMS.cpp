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

TestFrameworkFMS::TestFrameworkFMS(void)
{
}

TestFrameworkFMS::~TestFrameworkFMS(void)
{
}


// primary calculation method to update the FMS model
void TestFrameworkFMS::update(AircraftState state, std::vector<PrecalcWaypoint> &precalcWaypoints,
							  std::vector<HorizontalPath> &hTraj) {

	double xWp, yWp, dx, dy;

	double DesiredCourse;
	double xdot, ydot;
	double Course;
	double TrackError;
	double CrossTrackError;

	//-------------------------------------------------------------------------------------------------
	//	Computes RANGE to next WP
	//	- - - - - - - - - - - - -
	xWp			=  this->xWp[this->NextWp];
	yWp			=  this->yWp[this->NextWp];

	dx			=  xWp - state.x;
	dy			=  yWp - state.y;

	this->RangeToNextWpM1	=  this->RangeToNextWp;
	this->RangeToNextWp	=  sqrt(dx*dx + dy*dy);


	if (this->Mode == TRACKING)
	{
		//Gwang 2010-08: This may give a false premature indication of turn because of the noisiness of Fms.RangeToNextWp.
		if (this->RangeToNextWp > this->RangeToNextWpM1 && this->RangeToNextWp < 4000.0)
		{
			this->NextWp++;

			if (this->NextWp > this->number_of_waypoints - 1)
				//The next waypoint (Fms.NextWp) is beyond the final waypoint (Fms.number_of_waypoints - 1)
				//which means that the final waypoint has just been passed.
			{
				return;
			}
			xWp			=  this->xWp[this->NextWp];
			yWp			=  this->yWp[this->NextWp];
			dx			=  xWp-state.x;
			dy			=  yWp-state.y;
			this->RangeToNextWpM1	=  1.0e+10;
			this->RangeToNextWp	=  sqrt(dx*dx + dy*dy);
		}


		//	Determine if the a/c changes course between two waypoints
		//	- - - - - - - - - - - - - - - - - -
		if(this->NextWp >= this->number_of_waypoints - 1)
			this->DeltaTrack = 0.;
		else
			this->DeltaTrack	=  this->Track[this->NextWp+1] - this->Track[this->NextWp];

		double BankAngle = CoreUtils::limit(this->DeltaTrack,
											Units::RadiansAngle(-MAX_BANK_ANGLE).value(),
											Units::RadiansAngle(MAX_BANK_ANGLE).value());

		if (BankAngle != 0.00)
		{
			double gs = Units::FeetPerSecondSpeed(state.getGroundSpeed()).value();
			this->TurnRadius = gs * gs /
							   (GRAV_MPS / FT_M * tan(BankAngle)); // v^2/(g*tan theta)

			this->RangeStartTurn	=  fabs(this->TurnRadius*tan(0.5*this->DeltaTrack));
		}
		else
		{
			this->TurnRadius		=  1.0e+10;
			this->RangeStartTurn	=  0.00;
		}

		if (this->RangeToNextWp <= (this->RangeStartTurn))
		{
			this->Mode		=  TURNING;
			this->NextWp++;

			if (this->NextWp > this->number_of_waypoints - 1)
			{
				return;
			}

			xWp			=  this->xWp[this->NextWp];
			yWp			=  this->yWp[this->NextWp];
			dx			=  xWp-state.x;
			dy			=  yWp-state.y;
			this->RangeToNextWpM1	=  1.0e+10;
			this->RangeToNextWp	=  sqrt(dx*dx + dy*dy);
		}
	}

	//--------------------------------------------------------------------------------------------
	DesiredCourse = this->Track[this->NextWp];

	// sin and cos swapped to account for heading instead of angle
	CrossTrackError = -1.0*(-dy*sin(DesiredCourse) + dx*cos(DesiredCourse));
	//gwang
	double gs = Units::FeetPerSecondSpeed(state.getGroundSpeed()).value();
	xdot = gs*sin(state.get_heading())*cos(state.gamma); // + state.wind_x; // changed from cos to sin, since the value is a heading
	ydot = gs*cos(state.get_heading())*cos(state.gamma); // + state.wind_y; // changed from sin to cos, since the value is a heading
	//end gwang
	Course = atan3(xdot,ydot); // atan2(ydot, xdot); switched from Angle to Heading

	TrackError = Course - DesiredCourse;

	// fixes angles > +-180 degrees
	if (TrackError > M_PI)
	{
		while(TrackError > M_PI)
			TrackError = TrackError - 2.0*M_PI;
	}
	if (TrackError < -M_PI)
	{
		while(TrackError < -M_PI)
			TrackError = TrackError + 2.0*M_PI;
	}

	//--------------------------------------------------------------------------------------------

	if(this->Mode == TURNING)
	{
		double DeltaGroundTrack = -1.0*TrackError;

		if (CrossTrackError < 0.00 && DeltaGroundTrack < 10.00 * DTORAD)
			this->Mode = TRACKING;

		if (CrossTrackError > 0.00 && DeltaGroundTrack > 10.00 * DTORAD)
			this->Mode = TRACKING;

		if(fabs(CrossTrackError) < 1000.0 && fabs(TrackError) < 5.0*DTORAD)
			this->Mode = TRACKING;
	}


	// Recompute NextWp

	Units::MetersLength currDist;
	Units::RadiansAngle tempCrs;
	AircraftCalculations::getPathLengthFromPos(
			Units::FeetLength(state.x), Units::FeetLength(state.y),
			hTraj,currDist,tempCrs);

	this->NextWp = precalcWaypoints.size() - 1;

	for (int ix = 1;(ix<precalcWaypoints.size());ix++) {

		if ((currDist.value() < precalcWaypoints[ix-1].constraints.constraint_dist) &&
			(currDist.value() >= precalcWaypoints[ix].constraints.constraint_dist)) {
			this->NextWp = ix;
			break;
		}

	}
}


// init method to initialize the FMS data
void TestFrameworkFMS::init()
{
	int i;
	double dx;
	double dy;
	double Heading;
	for (i=1 ; i<this->number_of_waypoints ; i++)
	{
		dx	=  this->xWp[i]-this->xWp[i-1];
		dy	=  this->yWp[i]-this->yWp[i-1];
		Heading	= (double)atan3(dx, dy);// atan2(dy, dx); switched from Angle to Heading

		this->Track[i]	=  Heading;
		this->psi[i] = (double)atan2(dy,dx); // psi measured from east counter-clockwise
		this->Length[i]	=  sqrt(dx*dx + dy*dy);
	}

	for (i=this->number_of_waypoints ; i<127 ; i++)
	{
		this->Track[i]	=  -999.9;
		this->psi[i] = -999.9;
		this->Length[i]	=  -999.0;
	}

	this->NextWp		= 1;
	this->Mode		= TRACKING;
	this->RangeToNextWp = 1.0e+10;
	this->RangeToNextWpM1	=  1.0e+10;
}

void TestFrameworkFMS::copy_waypoints_from_intent(AircraftIntent intent_in)
{
	number_of_waypoints =  intent_in.getNumberOfWaypoints();

	const AircraftIntent::Fms &fms(intent_in.getFms());
	for(int j = 0; j < number_of_waypoints; j++)
	{
		xWp[j] = Units::FeetLength(fms.xWp[j]).value();
		yWp[j] = Units::FeetLength(fms.yWp[j]).value();
		AltWp[j] = Units::FeetLength(fms.AltWp[j]).value();
		nominal_IAS_at_waypoint[j] = fms.nominal_IAS_at_waypoint[j].value();
		MACH_at_waypoint[j] = fms.MACH_at_waypoint[j];
		constraints[j].constraint_altHi = fms.altHi[j].value();
		constraints[j].constraint_altLow = fms.altLow[j].value();
		constraints[j].constraint_speedHi = fms.speedHi[j].value();
		constraints[j].constraint_speedLow = fms.speedLow[j].value();
	}
}

// check to see if the FMS has reached the last waypoint
bool TestFrameworkFMS::is_finished()
{
	bool result = false;

	if( NextWp > number_of_waypoints -1)
	{
		result = true;
	}

	return result;
}

// get the psi of the given index if in range, returns -999.9 if out of range
double TestFrameworkFMS::get_psi(int index)
{
	double psi_result = -999.9;

	if( index >= 0 && index < number_of_waypoints)
	{
		psi_result = this->psi[index];
	}

	return psi_result;
}
