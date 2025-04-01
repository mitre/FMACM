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

#include "bada/FlapProgressionCalculator.h"
#include "public/BadaUtils.h"
#include "public/Logging.h"

namespace aaesim {
namespace bada {
class ReverseFlapProgression final : public FlapProgressionCalculator {
  private:
   inline static log4cplus::Logger logger_{log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("ReverseFlapProgression"))};

  public:
   ReverseFlapProgression() = default;
   ~ReverseFlapProgression() = default;
   aaesim::open_source::bada_utils::FlapConfiguration ComputeFlapProgression(
         aaesim::open_source::bada_utils::FlapSpeeds flap_speeds,
         aaesim::open_source::bada_utils::FlapConfiguration current_flap_configuration,
         Units::Speed calibrated_airspeed, const bool near_final_approach_fix) const override {
      // This logic uses these transition rules:
      // -- the flap configuration is defined on the interval [GEAR_DOWN, CRUISE];
      // -- the flap configuration can only decrement and never increments;
      // -- the flap configuration can now decrement multiple steps;
      // -- the flap configuration transition occurs as early in the CAS profile as possible
      //    based on the flaps speed schedule.

      bool advanced(true);
      while (advanced) {
         aaesim::open_source::bada_utils::FlapConfiguration previous_flap_configuration(current_flap_configuration);
         if (current_flap_configuration == aaesim::open_source::bada_utils::FlapConfiguration::GEAR_DOWN &&
             calibrated_airspeed < flap_speeds.cas_gear_out_minimum && near_final_approach_fix) {
            current_flap_configuration = aaesim::open_source::bada_utils::FlapConfiguration::GEAR_DOWN;
         } else if (current_flap_configuration == aaesim::open_source::bada_utils::FlapConfiguration::GEAR_DOWN &&
                    (calibrated_airspeed >= flap_speeds.cas_gear_out_maximum ||
                     calibrated_airspeed < flap_speeds.cas_landing_minimum) &&
                    near_final_approach_fix) {
            current_flap_configuration = aaesim::open_source::bada_utils::FlapConfiguration::LANDING;
         } else if (near_final_approach_fix &&
                    (current_flap_configuration == aaesim::open_source::bada_utils::FlapConfiguration::LANDING ||
                     current_flap_configuration == aaesim::open_source::bada_utils::FlapConfiguration::APPROACH) &&
                    (calibrated_airspeed >= flap_speeds.cas_landing_maximum ||
                     calibrated_airspeed < flap_speeds.cas_approach_minimum)) {
            current_flap_configuration = aaesim::open_source::bada_utils::FlapConfiguration::APPROACH;
         } else if ((current_flap_configuration == aaesim::open_source::bada_utils::FlapConfiguration::APPROACH ||
                     current_flap_configuration == aaesim::open_source::bada_utils::FlapConfiguration::CRUISE) &&
                    calibrated_airspeed >= flap_speeds.cas_approach_minimum) {
            current_flap_configuration = aaesim::open_source::bada_utils::FlapConfiguration::CRUISE;
         } else {
            // None of the above rules fired:  Exit the loop.
            advanced = false;
         }
         if (current_flap_configuration == previous_flap_configuration) {
            // Flap configuration did not change:  Exit the loop.
            advanced = false;
         }
      }  // end while (advanced)
      return current_flap_configuration;
   };
};
}  // namespace bada
}  // namespace aaesim
