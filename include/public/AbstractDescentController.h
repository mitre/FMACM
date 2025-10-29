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

#include <nlohmann/json.hpp>

#include "public/FixedMassAircraftPerformance.h"
#include "public/SpeedBrakeController.h"
#include "public/TrueWeatherOperator.h"
#include "public/VerticalController.h"

namespace aaesim::open_source {
class AbstractDescentController : public VerticalController {
  public:
   AbstractDescentController() = default;
   ~AbstractDescentController() = default;
   void Initialize(
         std::shared_ptr<aaesim::open_source::FixedMassAircraftPerformance> &performance_calculator) override {
      aircraft_performance_ = performance_calculator;
   }
   double GetSpeedBrakeGain() const override { return speed_brake_controller_->GetSpeedBrakeGain(); };

  protected:
   inline static void DoLogging(log4cplus::Logger &logger, const EquationsOfMotionState &state, Units::Length error_alt,
                                bool is_level_flight, Units::Force thrust_command, Units::Force max_thrust,
                                Units::Force min_thrust, Units::Speed error_tas, double speed_brake_command,
                                bada_utils::FlapConfiguration flap_configuration, Units::Angle gamma_command) {
      if (logger.getLogLevel() == log4cplus::TRACE_LOG_LEVEL) {
         using json = nlohmann::json;
         json j;
         j["altitude_error_ft"] = Units::FeetLength(error_alt).value();
         j["is_level_flight_bool"] = is_level_flight;
         j["thrust_command_newtons"] = Units::NewtonsForce(thrust_command).value();
         j["dynamics_thrust_newtons"] = Units::NewtonsForce(state.thrust).value();
         j["max_thrust_newtons"] = Units::NewtonsForce(max_thrust).value();
         j["min_thrust_newtons"] = Units::NewtonsForce(min_thrust).value();
         j["true_airspeed_error_knots"] = Units::KnotsSpeed(error_tas).value();
         j["speed_brake_command"] = speed_brake_command;
         j["flap_configuration"] = bada_utils::GetFlapConfigurationAsString(flap_configuration);
         j["gamma_command_deg"] = Units::DegreesAngle(gamma_command).value();
         LOG4CPLUS_TRACE(logger, j.dump());
      }
   };
   std::shared_ptr<FixedMassAircraftPerformance> aircraft_performance_{};
   std::shared_ptr<SpeedBrakeController> speed_brake_controller_{std::make_shared<SpeedBrakeController>()};
};

}  // namespace aaesim::open_source