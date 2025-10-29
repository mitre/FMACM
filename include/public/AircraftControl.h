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

#include "public/DefaultLateralController.h"
#include "public/EquationsOfMotionState.h"
#include "public/FixedMassAircraftPerformance.h"
#include "public/Guidance.h"
#include "public/LateralController.h"
#include "public/TrueWeatherOperator.h"
#include "public/VerticalController.h"

namespace aaesim::open_source {
struct ControlCommands {
   Units::Angle roll_angle_command{Units::zero()};
   Units::Force thrust_command{Units::zero()};
   Units::Angle flight_path_angle_command{Units::zero()};
   Units::Speed true_airspeed_command{Units::zero()};
   double speed_brake_command{0.0};
   aaesim::open_source::bada_utils::FlapConfiguration flap_configuration{
         aaesim::open_source::bada_utils::FlapConfiguration::UNDEFINED};
};

struct ControlGains {
   Units::Frequency k_flight_path_angle{Units::zero()};
   Units::Frequency k_thrust{Units::zero()};
   Units::Frequency k_roll{Units::zero()};
   double k_speed_brake{0.0};
};

class AircraftControl final {
  public:
   AircraftControl(
         const std::map<aaesim::open_source::GuidanceFlightPhase,
                        std::pair<std::shared_ptr<aaesim::open_source::LateralController>,
                                  std::shared_ptr<aaesim::open_source::VerticalController>>> &controller_pairs);

   virtual ~AircraftControl() = default;

   void Initialize(std::shared_ptr<aaesim::open_source::FixedMassAircraftPerformance> aircraft_performance);

   std::pair<ControlCommands, ControlGains> CalculateControlCommands(
         const Guidance &guidance, const EquationsOfMotionState &equations_of_motion_state,
         std::shared_ptr<const aaesim::open_source::TrueWeatherOperator> sensed_weather);

   class Builder {
     public:
      Builder &WithCruiseDescentVerticalController(std::shared_ptr<VerticalController> ctrl) {
         descent_vertical_controller_ = std::move(ctrl);
         return *this;
      }
      Builder &WithTakeoffVerticalController(std::shared_ptr<VerticalController> ctrl) {
         takeoff_vertical_controller_ = std::move(ctrl);
         return *this;
      }
      Builder &WithClimbVerticalController(std::shared_ptr<VerticalController> ctrl) {
         climb_vertical_controller_ = std::move(ctrl);
         return *this;
      }
      Builder &WithTakeoffLateralController(std::shared_ptr<LateralController> ctrl) {
         takeoff_lateral_controller_ = std::move(ctrl);
         return *this;
      }
      Builder &WithClimbLateralController(std::shared_ptr<LateralController> ctrl) {
         climb_lateral_controller_ = std::move(ctrl);
         return *this;
      }
      Builder &WithCruiseDescentLateralController(std::shared_ptr<LateralController> ctrl) {
         cruise_descent_lateral_controller_ = std::move(ctrl);
         return *this;
      }
      std::shared_ptr<AircraftControl> Build() const {
         std::map<aaesim::open_source::GuidanceFlightPhase,
                  std::pair<std::shared_ptr<aaesim::open_source::LateralController>,
                            std::shared_ptr<aaesim::open_source::VerticalController>>>
               controller_map;

         if (takeoff_lateral_controller_ && takeoff_vertical_controller_) {
            controller_map.emplace(aaesim::open_source::GuidanceFlightPhase::TAKEOFF_ROLL,
                                   std::make_pair(takeoff_lateral_controller_, takeoff_vertical_controller_));
         }

         if (climb_lateral_controller_ && climb_vertical_controller_) {
            controller_map.emplace(aaesim::open_source::GuidanceFlightPhase::CLIMB,
                                   std::make_pair(climb_lateral_controller_, climb_vertical_controller_));
         }

         if (cruise_descent_lateral_controller_ && descent_vertical_controller_) {
            controller_map.emplace(aaesim::open_source::GuidanceFlightPhase::CRUISE_DESCENT,
                                   std::make_pair(cruise_descent_lateral_controller_, descent_vertical_controller_));
         }

         if (controller_map.size() == 0) {
            throw std::runtime_error(
                  "Configuration Error: AircraftControl cannot be built because there are no controllers provided");
         }
         return std::make_shared<AircraftControl>(controller_map);
      }

     private:
      std::shared_ptr<VerticalController> descent_vertical_controller_{};
      std::shared_ptr<VerticalController> takeoff_vertical_controller_{};
      std::shared_ptr<VerticalController> climb_vertical_controller_{};
      std::shared_ptr<LateralController> takeoff_lateral_controller_{};
      std::shared_ptr<LateralController> climb_lateral_controller_{};
      std::shared_ptr<LateralController> cruise_descent_lateral_controller_{};
   };

  private:
   std::map<aaesim::open_source::GuidanceFlightPhase,
            std::pair<std::shared_ptr<aaesim::open_source::LateralController>,
                      std::shared_ptr<aaesim::open_source::VerticalController>>>
         controller_map_{};
};
}  // namespace aaesim::open_source
