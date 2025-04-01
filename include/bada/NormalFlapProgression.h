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

namespace aaesim {
namespace bada {
class NormalFlapProgression final : public FlapProgressionCalculator {
  public:
   NormalFlapProgression() = default;
   ~NormalFlapProgression() = default;
   aaesim::open_source::bada_utils::FlapConfiguration ComputeFlapProgression(
         aaesim::open_source::bada_utils::FlapSpeeds flap_speeds,
         aaesim::open_source::bada_utils::FlapConfiguration current_flap_configuration,
         Units::Speed calibrated_airspeed, const bool near_final_approach_fix) const override {
      // This logic uses these transition rules:
      // -- the flap configuration is defined on the interval [TAKEOFF, GEAR_DOWN];
      // -- the flap configuration can only increment and never decrements;
      // -- the flap configuration can only increment by one step;
      // -- the flap configuration transition occurs as late in the CAS profile as possible
      //    based on the flaps speed schedule. Maximums are not used; only when
      //    calibrated_airspeed has dropped below a minimum does the logic allow a transition.
      if (calibrated_airspeed < flap_speeds.cas_climb_minimum &&
          current_flap_configuration == aaesim::open_source::bada_utils::FlapConfiguration::TAKEOFF) {
         return aaesim::open_source::bada_utils::FlapConfiguration::TAKEOFF;
      } else if (calibrated_airspeed >= flap_speeds.cas_takeoff_minimum &&
                 calibrated_airspeed < flap_speeds.cas_climb_minimum &&
                 current_flap_configuration == aaesim::open_source::bada_utils::FlapConfiguration::TAKEOFF) {
         return aaesim::open_source::bada_utils::FlapConfiguration::TAKEOFF;
      } else if (calibrated_airspeed >= flap_speeds.cas_climb_minimum &&
                 calibrated_airspeed <= flap_speeds.cas_cruise_minimum &&
                 (current_flap_configuration == aaesim::open_source::bada_utils::FlapConfiguration::TAKEOFF ||
                  current_flap_configuration == aaesim::open_source::bada_utils::FlapConfiguration::INITIAL_CLIMB)) {
         return aaesim::open_source::bada_utils::FlapConfiguration::INITIAL_CLIMB;
      } else if (calibrated_airspeed > flap_speeds.cas_cruise_minimum &&
                 (current_flap_configuration == aaesim::open_source::bada_utils::FlapConfiguration::INITIAL_CLIMB ||
                  current_flap_configuration == aaesim::open_source::bada_utils::FlapConfiguration::CRUISE)) {
         return aaesim::open_source::bada_utils::FlapConfiguration::CRUISE;
      } else if ((calibrated_airspeed <= flap_speeds.cas_approach_minimum &&
                  calibrated_airspeed > flap_speeds.cas_landing_minimum) &&
                 near_final_approach_fix &&
                 (current_flap_configuration == aaesim::open_source::bada_utils::FlapConfiguration::CRUISE ||
                  current_flap_configuration == aaesim::open_source::bada_utils::FlapConfiguration::APPROACH)) {
         return aaesim::open_source::bada_utils::FlapConfiguration::APPROACH;
      } else if ((calibrated_airspeed <= flap_speeds.cas_landing_minimum &&
                  calibrated_airspeed > flap_speeds.cas_gear_out_minimum) &&
                 near_final_approach_fix &&
                 (current_flap_configuration == aaesim::open_source::bada_utils::FlapConfiguration::APPROACH ||
                  current_flap_configuration == aaesim::open_source::bada_utils::FlapConfiguration::LANDING)) {
         return aaesim::open_source::bada_utils::FlapConfiguration::LANDING;
      } else if (calibrated_airspeed <= flap_speeds.cas_gear_out_minimum && near_final_approach_fix &&
                 (current_flap_configuration == aaesim::open_source::bada_utils::FlapConfiguration::LANDING ||
                  current_flap_configuration == aaesim::open_source::bada_utils::FlapConfiguration::GEAR_DOWN)) {
         return aaesim::open_source::bada_utils::FlapConfiguration::GEAR_DOWN;
      }
      return current_flap_configuration;
   };
};
}  // namespace bada
}  // namespace aaesim