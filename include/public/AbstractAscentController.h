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

#include "public/AscentController.h"

namespace aaesim::open_source {
class AbstractAscentController : public AscentController, public VerticalController {
  public:
   AbstractAscentController() = default;
   ~AbstractAscentController() = default;
   void ComputeVerticalCommands(const Guidance &guidance, const EquationsOfMotionState &equations_of_motion_state,
                                std::shared_ptr<const aaesim::open_source::TrueWeatherOperator> &sensed_weather,
                                Units::Force &thrust_command, Units::Angle &gamma_command, Units::Speed &tas_command,
                                BoundedValue<double, 0, 1> &speed_brake_command,
                                aaesim::open_source::bada_utils::FlapConfiguration &flap_command) override {
      speed_brake_command = 0.0;
      ComputeAscentCommands(guidance, equations_of_motion_state, sensed_weather, thrust_command, gamma_command,
                            tas_command, flap_command);
   }
   void Initialize(
         std::shared_ptr<aaesim::open_source::FixedMassAircraftPerformance> &performance_calculator) override {
      aircraft_performance_ = performance_calculator;
   }
   double GetSpeedBrakeGain() const override { return 0.0; };

  protected:
   std::shared_ptr<FixedMassAircraftPerformance> aircraft_performance_{};
};

class NullAscentController final : public AbstractAscentController {
  public:
   NullAscentController() = default;
   ~NullAscentController() = default;
   void ComputeAscentCommands(const Guidance &guidance, const EquationsOfMotionState &equations_of_motion_state,
                              std::shared_ptr<const aaesim::open_source::TrueWeatherOperator> &sensed_weather,
                              Units::Force &thrust_command, Units::Angle &gamma_command, Units::Speed &tas_command,
                              aaesim::open_source::bada_utils::FlapConfiguration &flap_command) override {
      thrust_command = equations_of_motion_state.thrust;
      gamma_command = equations_of_motion_state.gamma;
      tas_command = equations_of_motion_state.true_airspeed;
      flap_command = equations_of_motion_state.flap_configuration;
   }
};

}  // namespace aaesim::open_source