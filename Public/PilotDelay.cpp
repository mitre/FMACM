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
// Copyright 2019 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#include "public/PilotDelay.h"
#include "public/Scenario.h"
#include "math/CustomMath.h"

const double PilotDelay::STANDARD_DEVIATION_LIMIT(3);
log4cplus::Logger PilotDelay::m_logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("PilotDelay"));

PilotDelay::PilotDelay()
      : m_atmosphere(nullptr),
        m_pilot_delay_mean(Units::SecondsTime(12.0)),
        m_pilot_delay_standard_deviation(Units::SecondsTime(0.0)),
        m_pilot_delay_is_on(true) {
   IterationReset();
}

PilotDelay::~PilotDelay() {
}

void PilotDelay::IterationReset() {
   m_time_to_next_speed_change = Units::SecondsTime(-1.0);
   m_guidance_ias = Units::zero();
   m_guidance_mach = 0.0;
}

Units::Speed PilotDelay::UpdateMach(double previous_im_speed_command_mach,
                                    double input_im_speed_command_mach,
                                    Units::Length current_altitude,
                                    Units::Length altitude_at_end_of_route) {
   if ((input_im_speed_command_mach != previous_im_speed_command_mach) &&
         (m_time_to_next_speed_change < Units::zero())) {
      m_time_to_next_speed_change = ComputeTimeToSpeedChange(current_altitude, altitude_at_end_of_route);

      m_guidance_mach = previous_im_speed_command_mach;
   }

   if (m_time_to_next_speed_change == Units::zero()) {
      m_guidance_mach = input_im_speed_command_mach;
   }

   m_time_to_next_speed_change -= Units::SecondsTime(1.0);

   return m_atmosphere->MachToIAS(m_guidance_mach, current_altitude);
}

Units::Speed PilotDelay::UpdateIAS(Units::Speed previous_im_speed_command_ias,
                                   Units::Speed input_im_speed_command_ias,
                                   Units::Length current_altitude,
                                   Units::Length altitude_at_end_of_route) {
   if (input_im_speed_command_ias != previous_im_speed_command_ias) {
      if (m_time_to_next_speed_change < Units::zero()) {
         m_time_to_next_speed_change = ComputeTimeToSpeedChange(current_altitude, altitude_at_end_of_route);

         if (m_guidance_ias == Units::zero()) {
            m_guidance_ias = m_atmosphere->MachToIAS(m_guidance_mach, current_altitude);
         } else {
            m_guidance_ias = previous_im_speed_command_ias;
         }
      } else if ((m_time_to_next_speed_change > Units::zero()) && (m_guidance_ias == Units::zero())) {
         m_guidance_ias = m_atmosphere->MachToIAS(m_guidance_mach, current_altitude);
      }
   } else if (m_guidance_ias == Units::zero()) {
      m_guidance_ias = m_atmosphere->MachToIAS(m_guidance_mach, current_altitude);
   }

   if (m_time_to_next_speed_change == Units::zero()) {
      m_guidance_ias = input_im_speed_command_ias;
   }

   m_time_to_next_speed_change -= Units::SecondsTime(1.0);

   return m_guidance_ias;
}

Units::Time PilotDelay::ComputeTimeToSpeedChange(Units::Length current_altitude,
                                                 Units::Length altitude_at_end_of_route) {
   Units::Time tval;

   if ((current_altitude - altitude_at_end_of_route) > Units::FeetLength(9000.0)) {
      tval = Scenario::m_rand.TruncatedGaussianSample(m_pilot_delay_mean, m_pilot_delay_standard_deviation,
                                                      STANDARD_DEVIATION_LIMIT);
   } else {
      tval = Scenario::m_rand.TruncatedGaussianSample(m_pilot_delay_mean / 2, m_pilot_delay_standard_deviation / 2,
                                                      STANDARD_DEVIATION_LIMIT);
   }

   tval = abs(quantize(tval, Units::SecondsTime(1)));

   return tval;
}

void PilotDelay::SetPilotDelayParameters(const Units::Time mean,
                                         const Units::Time standard_deviation) {
   m_pilot_delay_mean = mean;
   m_pilot_delay_standard_deviation = standard_deviation;
   if (m_pilot_delay_is_on && (m_pilot_delay_standard_deviation * STANDARD_DEVIATION_LIMIT > m_pilot_delay_mean)) {
      Units::SecondsTime low = m_pilot_delay_mean - m_pilot_delay_standard_deviation * STANDARD_DEVIATION_LIMIT;
      Units::SecondsTime high = m_pilot_delay_mean + m_pilot_delay_standard_deviation * STANDARD_DEVIATION_LIMIT;
      LOG4CPLUS_WARN(m_logger,
                     "Pilot delay can range from " << low << " to "
                                                   << high << " based on mean=" << m_pilot_delay_mean
                                                   << ", standard deviation=" << m_pilot_delay_standard_deviation
                                                   << ", and standard deviation cap=" << STANDARD_DEVIATION_LIMIT
                                                   << "." << std::endl
                                                   << "Computed negative delays will be flipped to positive.");
   }
}

void PilotDelay::DumpParameters(std::string str) {
   LOG4CPLUS_DEBUG(PilotDelay::m_logger, std::endl << "Pilot delay parms for "
                                                   << str.c_str() << std::endl << std::endl);
   LOG4CPLUS_DEBUG(PilotDelay::m_logger, "m_pilot_delay_is_on          " << m_pilot_delay_is_on << std::endl);
   LOG4CPLUS_DEBUG(PilotDelay::m_logger, "m_pilot_delay_mean        "
         << Units::SecondsTime(m_pilot_delay_mean).value() << std::endl);

   LOG4CPLUS_DEBUG(PilotDelay::m_logger, "mTimetoNextSpeedChange "
         << Units::SecondsTime(m_time_to_next_speed_change).value() << std::endl);
   LOG4CPLUS_DEBUG(PilotDelay::m_logger, "m_guidance_ias           "
         << Units::MetersPerSecondSpeed(m_guidance_ias).value() << std::endl);
   LOG4CPLUS_DEBUG(PilotDelay::m_logger, "m_guidance_mach          " << m_guidance_mach << std::endl);
}
