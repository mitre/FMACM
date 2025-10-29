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

#include "utility/BoundedValue.h"

namespace aaesim::open_source {
class SpeedBrakeController final {
  public:
   SpeedBrakeController() = default;
   ~SpeedBrakeController() = default;
   double GetSpeedBrakeGain() const { return gain_speedbrake_; };
   BoundedValue<double, 0, 1> GetCurrentCommand() const { return speed_brake_command_; }
   bool IsDeployed() const { return is_speedbrake_deployed_; }
   BoundedValue<double, 0, 1> Deploy() {
      ++speedbrake_counter_;
      speed_brake_command_ = speedbrake_command_maximum_;
      is_speedbrake_deployed_ = true;
      return GetCurrentCommand();
   }
   BoundedValue<double, 0, 1> Retract() {
      speedbrake_counter_ = 0;
      speed_brake_command_ = speedbrake_command_minimum_;
      is_speedbrake_deployed_ = false;
      return GetCurrentCommand();
   }
   BoundedValue<double, 0, 1> Update(bool thrust_command_is_minimum) {
      if (not is_speedbrake_deployed_) return GetCurrentCommand();
      const bool beyond_minimum_deployment_duration = speedbrake_counter_ > speedbrake_counter_maximum;
      if (not beyond_minimum_deployment_duration) {
         ++speedbrake_counter_;
         return GetCurrentCommand();
      }

      if (not thrust_command_is_minimum) {
         Retract();
      }
      return GetCurrentCommand();
   }

  private:
   inline static const double gain_speedbrake_{0.10};
   inline static const unsigned int speedbrake_counter_maximum{30};
   inline static const double speedbrake_command_maximum_{0.5};
   inline static const double speedbrake_command_minimum_{0.0};
   BoundedValue<double, 0, 1> speed_brake_command_{0};
   unsigned int speedbrake_counter_{0};
   bool is_speedbrake_deployed_{false};
};

}  // namespace aaesim::open_source