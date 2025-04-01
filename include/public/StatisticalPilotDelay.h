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

#include "public/PilotDelay.h"

#include <map>

#include "public/Logging.h"
#include "scalar/Length.h"
#include "scalar/Time.h"
#include "scalar/Speed.h"
#include "public/Atmosphere.h"

namespace aaesim::open_source {
class StatisticalPilotDelay final : public PilotDelay {

  public:
   static StatisticalPilotDelay NoDelay();
   static StatisticalPilotDelay WithDelayDefaults(std::shared_ptr<Atmosphere> atmosphere);
   static StatisticalPilotDelay WithDelay(Units::Time gaussian_mean, Units::Time gaussian_std,
                                          std::shared_ptr<Atmosphere> atmosphere);

   StatisticalPilotDelay() = default;

   void IterationReset();

   Units::Speed UpdateIAS(Units::Speed previous_speed_command_ias, Units::Speed proposed_speed_command_ias,
                          Units::Length current_altitude, Units::Length altitude_at_end_of_route) override;

   Units::Speed UpdateMach(double previous_speed_command_as_mach, double proposed_command_as_mach,
                           Units::Length current_altitude, Units::Length altitude_at_end_of_route) override;

   void SetUsePilotDelay(const bool delay_enabled);

   bool IsPilotDelayOn() const;

   std::pair<Units::Time, Units::Time> GetPilotDelayParameters() const;

  private:
   Units::Time ComputeTimeToSpeedChange(Units::Length current_altitude, Units::Length altitude_at_end_of_route);
   void SetAtmosphere(std::shared_ptr<Atmosphere> atmosphere);
   void SetInitialIAS(Units::Length current_altitude, Units::Speed fallback_IAS);
   void SetPilotDelayParameters(const Units::Time mean, const Units::Time standard_deviation);

   // for speed conversion
   std::shared_ptr<Atmosphere> m_atmosphere{};

   Units::Time m_time_to_next_speed_change{Units::SecondsTime(-1.0)};
   Units::Speed m_guidance_ias{Units::zero()};
   Units::SecondsTime m_pilot_delay_mean{Units::SecondsTime(12.0)};
   Units::SecondsTime m_pilot_delay_standard_deviation{Units::zero()};

   double m_guidance_mach{0};
   bool m_pilot_delay_is_on{true};

   // for statistical output
   int m_delay_count{0};
   double m_delay_sum{0};
   double m_delay_square_sum{0};
   std::map<double, int> m_delay_frequency{};

   inline static const double STANDARD_DEVIATION_LIMIT{3};
   static log4cplus::Logger m_logger;
};

inline void StatisticalPilotDelay::SetAtmosphere(std::shared_ptr<Atmosphere> atmosphere) { m_atmosphere = atmosphere; }

inline void StatisticalPilotDelay::SetUsePilotDelay(const bool delay_enabled) { m_pilot_delay_is_on = delay_enabled; }

inline bool StatisticalPilotDelay::IsPilotDelayOn() const { return m_pilot_delay_is_on; }

inline std::pair<Units::Time, Units::Time> StatisticalPilotDelay::GetPilotDelayParameters() const {
   return std::pair<Units::Time, Units::Time>(m_pilot_delay_mean, m_pilot_delay_standard_deviation);
}

inline StatisticalPilotDelay StatisticalPilotDelay::NoDelay() {
   StatisticalPilotDelay no_delay{};
   no_delay.SetUsePilotDelay(false);
   return no_delay;
}

inline StatisticalPilotDelay StatisticalPilotDelay::WithDelay(Units::Time gaussian_mean, Units::Time gaussian_std,
                                                              std::shared_ptr<Atmosphere> atmosphere) {
   StatisticalPilotDelay delayed{};
   delayed.SetUsePilotDelay(true);
   delayed.SetPilotDelayParameters(gaussian_mean, gaussian_std);
   delayed.SetAtmosphere(atmosphere);
   return delayed;
}

inline StatisticalPilotDelay StatisticalPilotDelay::WithDelayDefaults(std::shared_ptr<Atmosphere> atmosphere) {
   StatisticalPilotDelay delayed{};
   delayed.SetAtmosphere(atmosphere);
   return delayed;
}

}  // namespace aaesim::open_source
