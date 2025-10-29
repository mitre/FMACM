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

#include "public/AircraftControl.h"
#include "public/AircraftIntent.h"
#include "public/BadaUtils.h"
#include "public/ClimbPhaseVerticalController.h"
#include "public/DefaultLateralController.h"
#include "public/SpeedOnPitchControl.h"
#include "public/SpeedOnThrustControl.h"
#include "public/TakeOffVerticalController.h"

namespace aaesim::open_source {
class AircraftControllerFactory final {
  public:
   enum DescentSpeedControlStrategy { THRUST, PITCH, NONE };
   struct DescentSpeedControlConfig {
      DescentSpeedControlStrategy type{DescentSpeedControlStrategy::NONE};
      Units::Speed speed_threshold{Units::KnotsSpeed{20.0}};
      Units::Length altitude_threshold{Units::FeetLength{500.0}};
   };
   struct AircraftControllerConfig {
      DescentSpeedControlConfig descent_controller_config;
      Units::Angle maximum_allowable_roll_angle{Units::DegreesAngle{30.0}};
   };
   static std::shared_ptr<AircraftControl> BuildController(const AircraftControllerConfig &config,
                                                           const AircraftIntent &aircraft_intent) {
      auto lateral_controller = std::make_shared<DefaultLateralController>(config.maximum_allowable_roll_angle);
      AircraftControl::Builder builder{};
      if (config.descent_controller_config.type == DescentSpeedControlStrategy::NONE and
          aircraft_intent.ContainsDescentWaypoints()) {
         throw std::runtime_error(
               "Invalid vertical controller configuration for descent. Must specify the descent strategy");
      }

      if (aircraft_intent.ContainsDescentWaypoints() || aircraft_intent.ContainsCruiseWaypoints()) {
         builder.WithCruiseDescentLateralController(lateral_controller);
         if (config.descent_controller_config.type == THRUST) {
            auto descent_controller = std::make_shared<aaesim::open_source::SpeedOnThrustControl>();
            builder.WithCruiseDescentVerticalController(descent_controller);
         } else if (config.descent_controller_config.type == PITCH) {
            auto descent_controller = std::make_shared<aaesim::open_source::SpeedOnPitchControl>(
                  config.descent_controller_config.speed_threshold,
                  config.descent_controller_config.altitude_threshold);
            builder.WithCruiseDescentVerticalController(descent_controller);
         }
      }

      if (aircraft_intent.ContainsAscentWaypoints()) {
         auto ascent_controller = std::make_shared<aaesim::open_source::ClimbPhaseVerticalController>();
         builder.WithClimbVerticalController(ascent_controller);
         builder.WithClimbLateralController(lateral_controller);
         builder.WithTakeoffLateralController(std::make_shared<NoTurnLateralController>());
         builder.WithTakeoffVerticalController(std::make_shared<TakeOffVerticalController>());
      }

      return builder.Build();
   };
   static std::shared_ptr<AircraftControl> BuildForCruiseDescentOnly(const AircraftControllerConfig &config) {
      if (config.descent_controller_config.type == DescentSpeedControlStrategy::NONE) {
         throw std::runtime_error(
               "Invalid vertical controller configuration for descent. Must specify the descent strategy");
      }
      AircraftControl::Builder builder{};
      auto lateral_controller = std::make_shared<DefaultLateralController>(config.maximum_allowable_roll_angle);
      builder.WithCruiseDescentLateralController(lateral_controller);
      if (config.descent_controller_config.type == THRUST) {
         auto descent_controller = std::make_shared<aaesim::open_source::SpeedOnThrustControl>();
         builder.WithCruiseDescentVerticalController(descent_controller);
      } else if (config.descent_controller_config.type == PITCH) {
         auto descent_controller = std::make_shared<aaesim::open_source::SpeedOnPitchControl>(
               config.descent_controller_config.speed_threshold, config.descent_controller_config.altitude_threshold);
         builder.WithCruiseDescentVerticalController(descent_controller);
      }
      return builder.Build();
   };
};
};  // namespace aaesim::open_source