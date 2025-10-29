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

#include "public/EquationsOfMotionState.h"
#include "public/Guidance.h"
#include "public/TrueWeatherOperator.h"
#include "scalar/Angle.h"

namespace aaesim::open_source {
struct LateralController {
   virtual Units::Angle ComputeRollCommand(
         const Guidance &guidance, const EquationsOfMotionState &equations_of_motion_state,
         std::shared_ptr<const aaesim::open_source::TrueWeatherOperator> &sensed_weather) = 0;
   virtual Units::Frequency GetRollGain() const = 0;
};

class NoTurnLateralController final : public LateralController {
  public:
   NoTurnLateralController() = default;
   ~NoTurnLateralController() = default;
   Units::Angle ComputeRollCommand(
         const Guidance &guidance, const EquationsOfMotionState &equations_of_motion_state,
         std::shared_ptr<const aaesim::open_source::TrueWeatherOperator> &sensed_weather) override {
      return Units::zero();
   };
   Units::Frequency GetRollGain() const override { return Units::zero(); }
};
}  // namespace aaesim::open_source