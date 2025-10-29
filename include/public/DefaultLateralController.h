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

#include "public/LateralController.h"
#include "public/Logging.h"

namespace aaesim::open_source {
class DefaultLateralController final : public LateralController {
  public:
   DefaultLateralController(const Units::Angle &max_bank_angle) : max_bank_angle_{max_bank_angle} {};
   virtual ~DefaultLateralController() = default;
   Units::Angle ComputeRollCommand(
         const Guidance &guidance, const EquationsOfMotionState &equations_of_motion_state,
         std::shared_ptr<const aaesim::open_source::TrueWeatherOperator> &sensed_weather) override;
   Units::Frequency GetRollGain() const override { return gain_phi_; }

  private:
   inline static const log4cplus::Logger logger_{log4cplus::Logger::getInstance("DefaultLateralController")};
   inline static const Units::Frequency gain_phi_{Units::HertzFrequency(0.40)};
   static void DoLogging(const Units::Length &cross_track_error, const Units::Angle &track_angle_error,
                         const Units::Angle roll_command) {
      if (logger_.getLogLevel() == log4cplus::TRACE_LOG_LEVEL) {
         using json = nlohmann::json;
         json j;
         j["cross_track_error_ft"] = Units::FeetLength(cross_track_error).value();
         j["track_angle_error_deg"] = Units::DegreesAngle(track_angle_error).value();
         j["roll_command_deg"] = Units::DegreesAngle(roll_command).value();
         LOG4CPLUS_TRACE(logger_, j.dump());
      }
   }
   Units::Angle max_bank_angle_{};
};
}  // namespace aaesim::open_source