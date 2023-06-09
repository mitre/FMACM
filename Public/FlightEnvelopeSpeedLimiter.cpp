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

#include "public/FlightEnvelopeSpeedLimiter.h"

using namespace aaesim::open_source;

const Units::Speed FlightEnvelopeSpeedLimiter::MINIMUM_IAS_LIMIT = Units::KnotsSpeed(150);
const BoundedValue<double, 0, 2> FlightEnvelopeSpeedLimiter::MINIMUM_MACH_LIMIT = BoundedValue<double, 0, 2>(0.68);

FlightEnvelopeSpeedLimiter::FlightEnvelopeSpeedLimiter(
      const aaesim::open_source::bada_utils::FlapSpeeds &flap_speeds,
      const aaesim::open_source::bada_utils::FlightEnvelope &flight_envelope)
   : m_flap_speeds(flap_speeds), m_flight_envelope(flight_envelope) {}

Units::Speed FlightEnvelopeSpeedLimiter::LimitSpeedCommand(
      const Units::Speed previous_ias_speed_command, const Units::Speed current_ias_speed_command,
      const Units::Speed reference_velocity_mps, const Units::Length speed_quantization_distance,
      const Units::Length distance_to_end_of_route, const Units::Length current_altitude,
      const aaesim::open_source::bada_utils::FlapConfiguration flap_configuration) {

   Units::Speed limited_ias = Units::max(MINIMUM_IAS_LIMIT, current_ias_speed_command);

   switch (flap_configuration) {
      case bada_utils::FlapConfiguration::TAKEOFF:
         if (limited_ias < m_flap_speeds.cas_takeoff_minimum) {
            return m_flap_speeds.cas_takeoff_minimum;
         }
         return limited_ias;
      case bada_utils::FlapConfiguration::INITIAL_CLIMB:
         if (limited_ias < m_flap_speeds.cas_climb_minimum) {
            return m_flap_speeds.cas_climb_minimum;
         }
         return limited_ias;
      case bada_utils::FlapConfiguration::CRUISE:
         if (limited_ias < m_flap_speeds.cas_cruise_minimum) {
            return m_flap_speeds.cas_cruise_minimum;
         }
         return limited_ias;
      case bada_utils::FlapConfiguration::APPROACH:
         if (limited_ias < m_flap_speeds.cas_approach_minimum) {
            return m_flap_speeds.cas_approach_minimum;
         } else if (limited_ias > m_flap_speeds.cas_approach_maximum) {
            return m_flap_speeds.cas_approach_maximum;
         }
         return limited_ias;
      case bada_utils::FlapConfiguration::LANDING:
         if (limited_ias < m_flap_speeds.cas_landing_minimum) {
            return m_flap_speeds.cas_landing_minimum;
         } else if (limited_ias > m_flap_speeds.cas_landing_maximum) {
            return m_flap_speeds.cas_landing_maximum;
         }
         return limited_ias;
      case bada_utils::FlapConfiguration::GEAR_DOWN:
         if (limited_ias < m_flap_speeds.cas_gear_out_minimum) {
            return m_flap_speeds.cas_gear_out_minimum;
         } else if (limited_ias > m_flap_speeds.cas_gear_out_maximum) {
            return m_flap_speeds.cas_gear_out_maximum;
         }
         return limited_ias;
      default:
         return limited_ias;
   }
}

BoundedValue<double, 0, 2> FlightEnvelopeSpeedLimiter::LimitMachCommand(
      const BoundedValue<double, 0, 2> &previous_reference_speed_command_mach,
      const BoundedValue<double, 0, 2> &current_mach_command, const BoundedValue<double, 0, 2> &nominal_mach,
      const Units::Mass &current_mass, const Units::Length &current_altitude,
      const WeatherPrediction &weather_prediction) {
   if (current_mach_command > m_flight_envelope.M_mo) {
      return BoundedValue<double, 0, 2>(m_flight_envelope.M_mo);
   } else if (current_mach_command < MINIMUM_MACH_LIMIT) {
      return MINIMUM_MACH_LIMIT;
   }
   return current_mach_command;
}
