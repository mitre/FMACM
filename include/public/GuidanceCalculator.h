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

#include "public/AircraftState.h"
#include "public/Guidance.h"

namespace aaesim {
namespace open_source {
struct GuidanceCalculator {
   virtual open_source::Guidance Update(const open_source::AircraftState &current_state) = 0;
   static aaesim::open_source::Guidance CombineGuidance(const aaesim::open_source::Guidance &vertical_guidance,
                                                        const aaesim::open_source::Guidance &horizontal_guidance) {
      aaesim::open_source::Guidance full_guidance;

      full_guidance.m_cross_track_error = horizontal_guidance.m_cross_track_error;
      full_guidance.m_reference_bank_angle = horizontal_guidance.m_reference_bank_angle;
      full_guidance.m_use_cross_track = horizontal_guidance.m_use_cross_track;
      full_guidance.m_enu_track_angle = horizontal_guidance.m_enu_track_angle;

      full_guidance.m_reference_altitude = vertical_guidance.m_reference_altitude;
      full_guidance.m_vertical_speed = vertical_guidance.m_vertical_speed;
      full_guidance.m_ias_command = vertical_guidance.m_ias_command;
      full_guidance.m_ground_speed = vertical_guidance.m_ground_speed;
      full_guidance.m_active_guidance_phase = vertical_guidance.m_active_guidance_phase;
      full_guidance.SetSelectedSpeed(vertical_guidance.GetSelectedSpeed());
      full_guidance.SetMachCommand(vertical_guidance.GetMachCommand());

      full_guidance.SetValid(true);
      return full_guidance;
   }
};
}  // namespace open_source
}  // namespace aaesim