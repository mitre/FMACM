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

#pragma once

#include <AngularSpeed.h>

class EquationsOfMotionStateDeriv
{
public:
   Units::Speed enu_velocity_x, enu_velocity_y, enu_velocity_z; // east, north, altitude change
   Units::Acceleration true_airspeed_deriv; // true airspeed change
   Units::AngularSpeed gamma_deriv;   // flight-path angle change NOTE: for flight-path angle (gamma), heading down is positive; heading up is negative
   Units::AngularSpeed heading_deriv; // heading change measured from east counter-clockwise
   Units::ForceChange thrust_deriv; // thrust change
   Units::AngularSpeed roll_rate; // roll angle change
   double speed_brake_deriv; // peed brake (% of deployment) change rate
   int flap_configuration; // new flap configuration
};


