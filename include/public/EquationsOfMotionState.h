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
// 2023 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#pragma once

#include <scalar/Angle.h>
#include <scalar/Force.h>
#include <scalar/Speed.h>
#include <scalar/Length.h>
#include "public/BadaUtils.h"

class EquationsOfMotionState {
  public:
   Units::Length enu_x, enu_y, altitude_msl;
   Units::Speed true_airspeed;
   Units::Angle gamma;
   Units::SignedAngle psi_enu;
   Units::Force thrust;
   Units::Angle phi;
   double speed_brake_percentage;
   aaesim::open_source::bada_utils::FlapConfiguration flap_configuration;
};
