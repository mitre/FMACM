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

#include "scalar/Length.h"
#include "scalar/Mass.h"
#include "scalar/Speed.h"

#include "public/BadaUtils.h"
#include "public/WeatherPrediction.h"
#include "utility/BoundedValue.h"

namespace aaesim {
namespace open_source {
struct SpeedCommandLimiter {

   virtual ~SpeedCommandLimiter() = default;

   virtual Units::Speed LimitSpeedCommand(
         const Units::Speed previous_ias_speed_command, const Units::Speed current_ias_speed_command,
         const Units::Speed reference_velocity_mps, const Units::Length speed_quantization_distance,
         const Units::Length distance_to_end_of_route, const Units::Length current_altitude,
         const aaesim::open_source::bada_utils::FlapConfiguration flap_configuration) = 0;

   virtual BoundedValue<double, 0, 2> LimitMachCommand(
         const BoundedValue<double, 0, 2> &previous_reference_speed_command_mach,
         const BoundedValue<double, 0, 2> &estimated_mach, const BoundedValue<double, 0, 2> &nominal_mach,
         const Units::Mass &current_mass, const Units::Length &current_altitude,
         const WeatherPrediction &weather_prediction) = 0;
};
}  // namespace open_source
}  // namespace aaesim
