// ****************************************************************************
// NOTICE
//
// This work was produced for the U.S. Government under Contract 693KA8-22-C-00001
// and is subject to Federal Aviation Administration Acquisition Management System
// Clause 3.5-13, Rights In Data-General, Alt. III and Alt. IV (Oct. 1996).
//
// The contents of this document reflect the views of the author and The MITRE
// Corporation and do not necessarily reflect the views of the Federal Aviation
// Administration (FAA) or the Department of Transportation (DOT). Neither the FAA
// nor the DOT makes any warranty or guarantee, expressed or implied, concerning
// the content or accuracy of these views.
//
// For further information, please contact The MITRE Corporation, Contracts Management
// Office, 7515 Colshire Drive, McLean, VA 22102-7539, (703) 983-6000.
//
// 2022 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#pragma once

#include <scalar/AngularSpeed.h>

class EquationsOfMotionStateDeriv {
  public:
   Units::Speed enu_velocity_x, enu_velocity_y, enu_velocity_z;  // east, north, altitude change
   Units::Acceleration true_airspeed_deriv;                      // true airspeed change
   Units::AngularSpeed gamma_deriv;    // flight-path angle change NOTE: for flight-path angle (gamma), heading down is
                                       // positive; heading up is negative
   Units::AngularSpeed heading_deriv;  // heading change measured from east counter-clockwise
   Units::ForceChange thrust_deriv;    // thrust change
   Units::AngularSpeed roll_rate;      // roll angle change
   double speed_brake_deriv;           // speed brake (% of deployment) change rate
   aaesim::open_source::bada_utils::FlapConfiguration flap_configuration;  // flap configuration
};
