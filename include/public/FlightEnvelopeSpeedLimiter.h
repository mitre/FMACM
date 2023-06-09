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

#include "public/SpeedCommandLimiter.h"

namespace aaesim {
namespace open_source {
class FlightEnvelopeSpeedLimiter : public SpeedCommandLimiter {

  public:
   FlightEnvelopeSpeedLimiter(const aaesim::open_source::bada_utils::FlapSpeeds &flap_speeds,
                              const aaesim::open_source::bada_utils::FlightEnvelope &flight_envelope);

   Units::Speed LimitSpeedCommand(const Units::Speed previous_ias_speed_command,
                                  const Units::Speed current_ias_speed_command,
                                  const Units::Speed reference_velocity_mps,
                                  const Units::Length speed_quantization_distance,
                                  const Units::Length distance_to_end_of_route, const Units::Length current_altitude,
                                  const aaesim::open_source::bada_utils::FlapConfiguration flap_configuration) override;

   BoundedValue<double, 0, 2> LimitMachCommand(const BoundedValue<double, 0, 2> &previous_reference_speed_command_mach,
                                               const BoundedValue<double, 0, 2> &current_mach_command,
                                               const BoundedValue<double, 0, 2> &nominal_mach,
                                               const Units::Mass &current_mass, const Units::Length &current_altitude,
                                               const WeatherPrediction &weather_prediction) override;

   static const Units::Speed MINIMUM_IAS_LIMIT;
   static const BoundedValue<double, 0, 2> MINIMUM_MACH_LIMIT;

  private:
   aaesim::open_source::bada_utils::FlapSpeeds m_flap_speeds;
   aaesim::open_source::bada_utils::FlightEnvelope m_flight_envelope;
};
}  // namespace open_source
}  // namespace aaesim