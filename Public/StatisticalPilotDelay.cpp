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

#include <iomanip>

#include "public/StatisticalPilotDelay.h"
#include "public/ScenarioUtils.h"
#include "public/CustomMath.h"

using namespace aaesim::open_source;

log4cplus::Logger StatisticalPilotDelay::m_logger =
      log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("StatisticalPilotDelay"));

void StatisticalPilotDelay::IterationReset() {
   m_time_to_next_speed_change = Units::SecondsTime(-1.0);
   m_guidance_ias = Units::zero();
   m_guidance_mach = 0.0;
}

/**
 * Recomputes and updates mach as necessary according to the time delay
 * and recomputes ias accordingly.  Manages time to the next speed change.
 *
 * @param previous_im_speed_command_mach mach command from previous time.
 * @param input_im_speed_command_mach mach computed from calling method.
 * @param current_altitude altitude flight is at.
 * @param altitude_at_end_of_route altitude at the FAF or last point.
 *
 * @return recomputed guidance ias speed.
 */
Units::Speed StatisticalPilotDelay::UpdateMach(double previous_speed_command_mach, double proposed_speed_command_mach,
                                               Units::Length current_altitude, Units::Length altitude_at_end_of_route) {
   if ((proposed_speed_command_mach != previous_speed_command_mach) && (m_time_to_next_speed_change < Units::zero())) {
      // Reset time delay counter and set m_guidance_mach to previous speed.
      m_time_to_next_speed_change = ComputeTimeToSpeedChange(current_altitude, altitude_at_end_of_route);
      m_guidance_mach = previous_speed_command_mach;
   } else if (m_guidance_ias == Units::zero()) {
      // ias has not changed and guidance ias not set. Compute guidance ias
      // from guidance mach.
      m_guidance_mach = previous_speed_command_mach;
      SetInitialIAS(current_altitude, m_atmosphere->MachToIAS(previous_speed_command_mach, current_altitude));

      if (m_time_to_next_speed_change < Units::zero()) {
         // past delay time-recompute new delay time and update guidance ias.
         m_time_to_next_speed_change = ComputeTimeToSpeedChange(current_altitude, altitude_at_end_of_route);
      }
   }

   if (m_time_to_next_speed_change == Units::zero()) {
      // At time to change speed-set to input mach.
      m_guidance_mach = proposed_speed_command_mach;
   }

   // Update time delay counter and return guidance speed in ias.
   m_time_to_next_speed_change -= Units::SecondsTime(1.0);

   return m_atmosphere->MachToIAS(m_guidance_mach, current_altitude);
}

/**
 * Recomputes and updates ias as necessary.  The pilot delay time is also updated.
 *
 * @param previous_speed_command_ias Computed ias from the last time.
 * @param proposed_speed_command_ias computed ias from calling method.
 * @param current_altitude aircraft altitude.
 * @param altitude_at_end_of_route altitude at the FAF or last point.
 *
 * @return guidance ias speed.
 */
Units::Speed StatisticalPilotDelay::UpdateIAS(Units::Speed previous_speed_command_ias,
                                              Units::Speed proposed_speed_command_ias, Units::Length current_altitude,
                                              Units::Length altitude_at_end_of_route) {

   if (proposed_speed_command_ias != previous_speed_command_ias) {
      // ias has changed.
      if (m_time_to_next_speed_change < Units::zero()) {
         // past delay time-recompute new delay time and update guidance ias.
         m_time_to_next_speed_change = ComputeTimeToSpeedChange(current_altitude, altitude_at_end_of_route);

         if (m_guidance_ias == Units::zero()) {
            // compute first guidance ias
            SetInitialIAS(current_altitude, previous_speed_command_ias);
         } else {
            // set guidance ias from previous ias.
            m_guidance_ias = previous_speed_command_ias;
         }
      } else if ((m_time_to_next_speed_change > Units::zero()) && (m_guidance_ias == Units::zero())) {
         // not at delay time yet and guidance ias not set-compute guidance ias
         // from guidance mach.
         SetInitialIAS(current_altitude, previous_speed_command_ias);
      }
   } else if (m_guidance_ias == Units::zero()) {
      // ias has not changed and guidance ias not set-compute guidnace ias
      // from guidance mach.
      SetInitialIAS(current_altitude, previous_speed_command_ias);
   }

   if (m_time_to_next_speed_change == Units::zero()) {
      // time to process delay-set guidance ias from input ias.
      m_guidance_ias = proposed_speed_command_ias;
   }

   // update delay time and return guidance ias.
   m_time_to_next_speed_change -= Units::SecondsTime(1.0);

   if (m_guidance_ias == Units::zero()) {
      throw std::runtime_error("Zero guidance IAS computed.");
   }

   return m_guidance_ias;
}

/**
 * Computes time to next speed change.
 *
 * @param current_altitude altitude flight is at.
 * @param altitude_at_end_of_route altitude at FAF or last point.
 *
 * @return time to next speed change.
 */
Units::Time StatisticalPilotDelay::ComputeTimeToSpeedChange(Units::Length current_altitude,
                                                            Units::Length altitude_at_end_of_route) {
   Units::SecondsTime tval;

   if ((current_altitude - altitude_at_end_of_route) > Units::FeetLength(9000.0)) {
      tval = aaesim::open_source::ScenarioUtils::RANDOM_NUMBER_GENERATOR.TruncatedGaussianSample(
            m_pilot_delay_mean, m_pilot_delay_standard_deviation, STANDARD_DEVIATION_LIMIT);
   } else {
      tval = aaesim::open_source::ScenarioUtils::RANDOM_NUMBER_GENERATOR.TruncatedGaussianSample(
            m_pilot_delay_mean / 2, m_pilot_delay_standard_deviation / 2, STANDARD_DEVIATION_LIMIT);
   }

   tval = abs(quantize(tval, Units::SecondsTime(1)));

   m_delay_count++;
   double t = tval.value();
   m_delay_sum += t;
   m_delay_square_sum += t * t;
   m_delay_frequency[t]++;

   return tval;
}

void StatisticalPilotDelay::SetPilotDelayParameters(const Units::Time mean, const Units::Time standard_deviation) {
   m_pilot_delay_mean = mean;
   m_pilot_delay_standard_deviation = standard_deviation;
   if (m_pilot_delay_is_on && (m_pilot_delay_standard_deviation * STANDARD_DEVIATION_LIMIT > m_pilot_delay_mean)) {
      Units::SecondsTime low = m_pilot_delay_mean - m_pilot_delay_standard_deviation * STANDARD_DEVIATION_LIMIT;
      Units::SecondsTime high = m_pilot_delay_mean + m_pilot_delay_standard_deviation * STANDARD_DEVIATION_LIMIT;
      LOG4CPLUS_WARN(m_logger, "Pilot delay can range from "
                                     << low << " to " << high << " based on mean=" << m_pilot_delay_mean
                                     << ", standard deviation=" << m_pilot_delay_standard_deviation
                                     << ", and standard deviation cap=" << STANDARD_DEVIATION_LIMIT << "." << std::endl
                                     << "Computed negative delays will be flipped to positive.");
   }
}

/**
 * Sets the guidance IAS to the converted Mach if known; otherwise the provided fallback.
 */
void StatisticalPilotDelay::SetInitialIAS(Units::Length current_altitude, Units::Speed fallback_IAS) {

   // Have we been using Mach?
   if (m_guidance_mach != 0) {
      m_guidance_ias = m_atmosphere->MachToIAS(m_guidance_mach, current_altitude);
   } else {
      m_guidance_ias = fallback_IAS;
   }
}
