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

#include "public/AbstractDescentController.h"
#include "public/SpeedOnThrustControl.h"

namespace aaesim::open_source {
class SpeedOnPitchControl final : public AbstractDescentController {
  public:
   SpeedOnPitchControl(const Units::Speed speed_threshold, const Units::Length altitude_threshold)
      : speed_threshold_{speed_threshold}, altitude_threshold_{altitude_threshold} {};
   SpeedOnPitchControl() = delete;
   void Initialize(std::shared_ptr<aaesim::open_source::FixedMassAircraftPerformance> &aircraft_performance) override;
   void ComputeVerticalCommands(const Guidance &guidance, const EquationsOfMotionState &equations_of_motion_state,
                                std::shared_ptr<const aaesim::open_source::TrueWeatherOperator> &sensed_weather,
                                Units::Force &thrust_command, Units::Angle &gamma_command, Units::Speed &tas_command,
                                BoundedValue<double, 0, 1> &speed_brake_command,
                                aaesim::open_source::bada_utils::FlapConfiguration &flap_configuration) override;

  private:
   inline static log4cplus::Logger logger_{log4cplus::Logger::getInstance("SpeedOnPitchControl")};
   Units::Speed speed_threshold_{Units::KnotsSpeed{20.0}};
   Units::Length altitude_threshold_{Units::FeetLength{500.0}};
   bool is_level_flight_{true};
   std::shared_ptr<SpeedOnThrustControl> speed_on_thrust_controller_{std::make_shared<SpeedOnThrustControl>()};
};
}  // namespace aaesim::open_source
