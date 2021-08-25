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

#include <map>
#include "utility/Logging.h"
#include <Length.h>
#include <Time.h>
#include <Speed.h>
#include "public/Atmosphere.h"


// PilotDelay acts as an executive class which times and computes speed updates 
// when pilot delay is indicated in the scenario.  This is setup to be used
// during the update methods of the IM algorithms.

class PilotDelay
{

public:
   PilotDelay();

   virtual ~PilotDelay();

   void IterationReset();

   Units::Speed UpdateIAS(Units::Speed previous_im_speed_command_as_ias,
                          Units::Speed im_speed_command_as_ias,
                          Units::Length current_altitude,
                          Units::Length altitude_at_end_of_route);

   Units::Speed UpdateMach(double previous_im_speed_command_as_mach,
                           double im_speed_command_as_mach,
                           Units::Length current_altitude,
                           Units::Length altitude_at_end_of_route);

   void SetUsePilotDelay(const bool delay_enabled);

   void SetPilotDelayParameters(const Units::Time mean,
                                const Units::Time standard_deviation);

   void SetAtmosphere(std::shared_ptr<Atmosphere> atmosphere);

   void DumpParameters(std::string str);

   void DumpStatistics();

   bool IsPilotDelayOn();

   std::pair<Units::Time, Units::Time> GetPilotDelayParameters() const;

private:
   Units::Time ComputeTimeToSpeedChange(Units::Length current_altitude,
                                        Units::Length altitude_at_end_of_route);
   void SetInitialIAS(Units::Length current_altitude, Units::Speed fallback_IAS);

   // for speed conversion
   std::shared_ptr<Atmosphere> m_atmosphere;

   Units::Time m_time_to_next_speed_change;
   Units::Speed m_guidance_ias;
   Units::SecondsTime m_pilot_delay_mean;
   Units::SecondsTime m_pilot_delay_standard_deviation;

   double m_guidance_mach;
   bool m_pilot_delay_is_on;

   Units::Speed m_current_speed_command_mach_as_ias, m_next_speed_command_mach_as_ias;

   // for statistical output
   int m_delay_count;
   double m_delay_sum;
   double m_delay_square_sum;
   std::map<double,int> m_delay_frequency;

   static const double STANDARD_DEVIATION_LIMIT;
   static log4cplus::Logger m_logger;
};

inline void PilotDelay::SetAtmosphere(std::shared_ptr<Atmosphere> atmosphere) {
   m_atmosphere = atmosphere;
}

inline void PilotDelay::SetUsePilotDelay(const bool delay_enabled) {
   m_pilot_delay_is_on = delay_enabled;
}

inline bool PilotDelay::IsPilotDelayOn() {
   return m_pilot_delay_is_on;
}

inline std::pair<Units::Time, Units::Time> PilotDelay::GetPilotDelayParameters() const {
   return std::pair<Units::Time, Units::Time>(m_pilot_delay_mean, m_pilot_delay_standard_deviation);
}
