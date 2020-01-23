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

#pragma  once


#include <Angle.h>
#include <Force.h>
#include <Speed.h>
#include <Length.h>

class EquationsOfMotionState
{
public:
   /*
   X[1]: aircraft position east coordinate (m)
   X[2]: aircraft position  north coordinate (m)
   X[3]: aircraft position altitude (m)
   X[4]: aircraft true airspeed (m/s)
   X[5]: aircraft flight-path angle (rad). NOTE: for flight-path angle (gamma), heading down is positive; heading up is negative
   X[6]: aircraft heading (psi) measured from east counter-clockwise (rad).
   X[7]: aircraft thrust (N)
   X[8]: aircraft roll angle (phi) (rad)
   X[9]: aircraft speed brake (% of deployment)
   */
   Units::Length enu_x, enu_y, enu_z; // [1..3] east, north, altitude
   Units::Speed true_airspeed; // [4] true airspeed
   Units::Angle gamma; // [5] flight-path angle NOTE: for flight-path angle (gamma), heading down is positive; heading up is negative
   Units::Angle psi; // [6] heading measured from east counter-clockwise
   Units::Force thrust; // [7] thrust
   Units::Angle phi; // [8] roll angle
   double speedBrake; // [9] speed brake (% of deployment)
   int flapConfig; // [10] flap configuration
};
