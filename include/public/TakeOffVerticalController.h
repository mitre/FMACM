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

#include "public/AbstractAscentController.h"
#include "public/FixedMassAircraftPerformance.h"

namespace aaesim::open_source {
class TakeOffVerticalController final : public AbstractAscentController {
  public:
   TakeOffVerticalController() = default;
   ~TakeOffVerticalController() = default;
   void ComputeAscentCommands(const Guidance &guidance, const EquationsOfMotionState &equations_of_motion_state,
                              std::shared_ptr<const aaesim::open_source::TrueWeatherOperator> &sensed_weather,
                              Units::Force &thrust_command, Units::Angle &gamma_command, Units::Speed &tas_command,
                              aaesim::open_source::bada_utils::FlapConfiguration &flap_command) override {
      thrust_command = aircraft_performance_->GetMaxThrust(
            equations_of_motion_state.altitude_msl, bada_utils::FlapConfiguration::TAKEOFF,
            bada_utils::EngineThrustMode::MAXIMUM_CLIMB, Units::ZERO_CELSIUS);
      gamma_command = Units::zero();
      flap_command = bada_utils::TAKEOFF;
      tas_command =
            sensed_weather->GetTrueWeather()->CAS2TAS(guidance.m_ias_command, equations_of_motion_state.altitude_msl);
   }
};

};  // namespace aaesim::open_source