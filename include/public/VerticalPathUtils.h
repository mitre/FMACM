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

#include "scalar/Time.h"
#include "scalar/Length.h"
#include "scalar/Speed.h"

namespace aaesim {
namespace open_source {
struct VerticalPathUtils {

   static Units::Time CalculateTimeToFly(const VerticalPath &vertical_path,
                                         Units::Length estimated_distance_to_path_end);

   static Units::Speed CalculateSpeedGuidance(const VerticalPath &vertical_path,
                                              Units::Length estimated_distance_to_path_end);
};
}  // namespace open_source
}  // namespace aaesim

inline Units::Speed aaesim::open_source::VerticalPathUtils::CalculateSpeedGuidance(
      const VerticalPath &vertical_path, Units::Length estimated_distance_to_path_end) {
   auto reference_lookup_index = CoreUtils::FindNearestIndex(
         Units::MetersLength(estimated_distance_to_path_end).value(), vertical_path.along_path_distance_m);

   Units::Speed cas_guiance = Units::zero();
   if (reference_lookup_index == 0) {
      cas_guiance = Units::MetersPerSecondSpeed(vertical_path.cas_mps[0]);
   } else {
      cas_guiance = Units::MetersPerSecondSpeed(CoreUtils::LinearlyInterpolate(
            reference_lookup_index, Units::MetersLength(estimated_distance_to_path_end).value(),
            vertical_path.along_path_distance_m, vertical_path.cas_mps));
   }

   return cas_guiance;
}

inline Units::Time aaesim::open_source::VerticalPathUtils::CalculateTimeToFly(
      const VerticalPath &vertical_path, Units::Length estimated_distance_to_path_end) {
   auto reference_lookup_index = CoreUtils::FindNearestIndex(
         Units::MetersLength(estimated_distance_to_path_end).value(), vertical_path.along_path_distance_m);

   Units::Time time_to_fly = Units::zero();
   if (reference_lookup_index == 0) {
      time_to_fly = Units::SecondsTime(vertical_path.time_to_go_sec[0]);
   } else {
      time_to_fly = Units::SecondsTime(CoreUtils::LinearlyInterpolate(
            reference_lookup_index, Units::MetersLength(estimated_distance_to_path_end).value(),
            vertical_path.along_path_distance_m, vertical_path.time_to_go_sec));
   }

   return time_to_fly;
}
