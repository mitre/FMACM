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

#include "public/AbstractAscentController.h"
#include "public/FixedMassAircraftPerformance.h"
#include "public/Logging.h"

namespace aaesim::open_source {
class ClimbPhaseVerticalController final : public AbstractAscentController {
  public:
   ClimbPhaseVerticalController() = default;
   ~ClimbPhaseVerticalController() = default;
   void ComputeAscentCommands(const Guidance &guidance, const EquationsOfMotionState &equations_of_motion_state,
                              std::shared_ptr<const aaesim::open_source::TrueWeatherOperator> &sensed_weather,
                              Units::Force &thrust_command, Units::Angle &gamma_command, Units::Speed &tas_command,
                              aaesim::open_source::bada_utils::FlapConfiguration &flap_command) override;

  private:
   inline static log4cplus::Logger logger_{log4cplus::Logger::getInstance("ClimbPhaseVerticalController")};
   static void DoLogging(const Units::Length &error_alt, const Units::Force &thrust_command, const Units::Force &thrust,
                         const Units::Force &min_thrust, const Units::Force &max_thrust,
                         const bada_utils::FlapConfiguration &flap_configuration, const Units::Speed &error_tas,
                         const Units::Speed &tas_command, const Units::Angle &gamma_command) {
      if (logger_.getLogLevel() == log4cplus::TRACE_LOG_LEVEL) {
         using json = nlohmann::json;
         json j;
         j["altitude_error"] = Units::FeetLength(error_alt).value();
         j["thrust_command"] = Units::NewtonsForce(thrust_command).value();
         j["dynamics_thrust"] = Units::NewtonsForce(thrust).value();
         j["max_thrust"] = Units::NewtonsForce(max_thrust).value();
         j["min_thrust"] = Units::NewtonsForce(min_thrust).value();
         j["new_flap_configuration"] = bada_utils::GetFlapConfigurationAsString(flap_configuration);
         j["true_airspeed_error"] = Units::KnotsSpeed(error_tas).value();
         j["true_airspeed_command"] = Units::KnotsSpeed(tas_command).value();
         j["gamma_command"] = Units::DegreesAngle(gamma_command).value();
         LOG4CPLUS_TRACE(logger_, j.dump());
      }
   }
};
}  // namespace aaesim::open_source