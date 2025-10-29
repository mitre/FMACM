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

#include "public/AircraftControl.h"

#include <nlohmann/json.hpp>

using namespace aaesim::open_source;

AircraftControl::AircraftControl(
      const std::map<aaesim::open_source::GuidanceFlightPhase,
                     std::pair<std::shared_ptr<aaesim::open_source::LateralController>,
                               std::shared_ptr<aaesim::open_source::VerticalController>>> &controller_pairs) {
   controller_map_ = controller_pairs;
}

void AircraftControl::Initialize(
      std::shared_ptr<aaesim::open_source::FixedMassAircraftPerformance> aircraft_performance) {
   for (auto &entry : controller_map_) {
      const auto &vertical = entry.second.second;
      if (vertical) {
         vertical->Initialize(aircraft_performance);
      }
   }
}

std::pair<ControlCommands, ControlGains> AircraftControl::CalculateControlCommands(
      const Guidance &guidance, const EquationsOfMotionState &equations_of_motion_state,
      std::shared_ptr<const aaesim::open_source::TrueWeatherOperator> sensed_weather) {
   const auto &pair = controller_map_.at(guidance.m_active_guidance_phase);
   const auto &lateral_controller = pair.first;
   const auto &vertical_controller = pair.second;

   Units::Angle phi_command =
         lateral_controller->ComputeRollCommand(guidance, equations_of_motion_state, sensed_weather);

   aaesim::open_source::bada_utils::FlapConfiguration flap_configuration{
         aaesim::open_source::bada_utils::FlapConfiguration::UNDEFINED};
   BoundedValue<double, 0, 1> speed_brake_command{0};
   Units::Force thrust_command{Units::zero()};
   Units::Angle gamma_command{Units::zero()};
   Units::Speed true_airspeed_command{Units::zero()};
   vertical_controller->ComputeVerticalCommands(guidance, equations_of_motion_state, sensed_weather, thrust_command,
                                                gamma_command, true_airspeed_command, speed_brake_command,
                                                flap_configuration);

   return std::make_pair(ControlCommands{phi_command, thrust_command, gamma_command, true_airspeed_command,
                                         speed_brake_command, flap_configuration},
                         ControlGains{vertical_controller->GetGammaGain(), vertical_controller->GetThrustGain(),
                                      lateral_controller->GetRollGain(), vertical_controller->GetSpeedBrakeGain()});
}
