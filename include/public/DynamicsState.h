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

#include "BadaUtils.h"
#include <utility/CustomUnits.h>
#include <scalar/Speed.h>
#include <scalar/Length.h>
#include <scalar/SignedAngle.h>
#include <scalar/Force.h>

namespace aaesim {
namespace open_source {

typedef struct {
   // This structure is not used at all within the EOM function. It only serves to
   // represent the state outside the EOM function. Within the EOM function, the state is
   // represented with EquationsOfMotionState.
   int id;
   Units::MetersLength x;                        // aircraft position east coordinate
   Units::MetersLength y;                        // aircraft position north coordinate
   Units::MetersLength h;                        // aircraft altitude
   Units::MetersPerSecondSpeed v_true_airspeed;  // True airspeed
   Units::KnotsSpeed v_indicated_airspeed;       // calibrated/indicated airspeed
   Units::SignedAngle psi;                       // aircraft heading angle measured from east counter-clockwise
   Units::RadiansAngle phi;                      // aircraft roll angle
   Units::RadiansAngle gamma;  // aircraft flight-path angle (rad) NOTE: for gamma, heading down is positive; heading
   // up is negative
   Units::NewtonsForce thrust;
   Units::MetersPerSecondSpeed xd;                                         // ground speed x component
   Units::MetersPerSecondSpeed yd;                                         // ground speed y component
   double speed_brake;                                                     // speed brake (% of deployment)
   aaesim::open_source::bada_utils::FlapConfiguration flap_configuration;  // for flaps speed
   Units::Mass current_mass;
} DynamicsState;
}  // namespace open_source
}  // namespace aaesim
