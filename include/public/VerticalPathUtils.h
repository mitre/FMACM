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
#include "public/CoreUtils.h"

namespace aaesim {
namespace open_source {
struct VerticalPathUtils {

   struct VerticalPathDataSet final {
      Units::Length along_path_distance{};
      Units::Length altitude_msl{};
      Units::Speed calibrated_airspeed{};
      double mach{INT32_MIN};
      Units::Speed altitude_rate{};
      Units::Speed true_airspeed{};
      Units::Acceleration tas_rate{};
      Units::Angle theta{};
      Units::Speed ground_speed{};
      Units::Time time_to_go{};
      Units::Mass mass{};
      Units::MetersPerSecondSpeed wind_velocity_east{};
      Units::MetersPerSecondSpeed wind_velocity_north{};
      aaesim::open_source::bada_utils::FlapConfiguration flap_setting{
            aaesim::open_source::bada_utils::FlapConfiguration::UNDEFINED};
      int resolved_index{INT32_MIN};
   };

   static Units::Time CalculateTimeToFly(const VerticalPath &vertical_path,
                                         Units::Length estimated_distance_to_path_end);

   static Units::Speed CalculateSpeedGuidance(const VerticalPath &vertical_path,
                                              Units::Length estimated_distance_to_path_end);

   static double CalculateMachGuidance(const VerticalPath &vertical_path, Units::Length estimated_distance_to_path_end);

   static Units::Mass GetExpectedMass(const VerticalPath &vertical_path, Units::Length estimated_distance_to_path_end);

   static VerticalPathDataSet GetVerticalPathData(const VerticalPath &vertical_path,
                                                  Units::Length estimated_distance_to_path_end);

   static VerticalPathDataSet GetInterpolatedPathData(const VerticalPath &vertical_path,
                                                      Units::Length estimated_distance_to_path_end);

   static VerticalPathDataSet GetPathDataAtIndex(const VerticalPath &vertical_path, int index);
};
}  // namespace open_source
}  // namespace aaesim

inline aaesim::open_source::VerticalPathUtils::VerticalPathDataSet
      aaesim::open_source::VerticalPathUtils::GetVerticalPathData(const VerticalPath &vertical_path,
                                                                  Units::Length estimated_distance_to_path_end) {
   const Units::MetersLength distance_to_go{estimated_distance_to_path_end};
   const auto reference_lookup_index =
         CoreUtils::FindNearestIndex(distance_to_go.value(), vertical_path.along_path_distance_m);
   return GetPathDataAtIndex(vertical_path, reference_lookup_index);
}

inline aaesim::open_source::VerticalPathUtils::VerticalPathDataSet
      aaesim::open_source::VerticalPathUtils::GetPathDataAtIndex(const VerticalPath &vertical_path, int index) {
   aaesim::open_source::VerticalPathUtils::VerticalPathDataSet single_data_row{};
   single_data_row.resolved_index = index;
   single_data_row.along_path_distance = Units::MetersLength(vertical_path.along_path_distance_m[index]);
   single_data_row.altitude_msl = Units::MetersLength(vertical_path.altitude_m[index]);
   single_data_row.calibrated_airspeed = Units::MetersPerSecondSpeed(vertical_path.cas_mps[index]);
   single_data_row.mach = vertical_path.mach[index];
   single_data_row.altitude_rate = Units::MetersPerSecondSpeed(vertical_path.altitude_rate_mps[index]);
   single_data_row.true_airspeed = vertical_path.true_airspeed[index];
   single_data_row.tas_rate = Units::MetersSecondAcceleration(vertical_path.tas_rate_mps[index]);
   single_data_row.theta = Units::RadiansAngle(vertical_path.theta_radians[index]);
   single_data_row.ground_speed = Units::MetersPerSecondSpeed(vertical_path.gs_mps[index]);
   single_data_row.time_to_go = Units::SecondsTime(vertical_path.time_to_go_sec[index]);
   single_data_row.mass = Units::KilogramsMass(vertical_path.mass_kg[index]);
   single_data_row.wind_velocity_east = vertical_path.wind_velocity_east[index];
   single_data_row.wind_velocity_north = vertical_path.wind_velocity_north[index];
   single_data_row.flap_setting = vertical_path.flap_setting[index];
   return single_data_row;
}

inline aaesim::open_source::VerticalPathUtils::VerticalPathDataSet
      aaesim::open_source::VerticalPathUtils::GetInterpolatedPathData(const VerticalPath &vertical_path,
                                                                      Units::Length estimated_distance_to_path_end) {

   const Units::MetersLength distance_to_go{estimated_distance_to_path_end};
   const auto reference_lookup_index =
         CoreUtils::FindNearestIndex(distance_to_go.value(), vertical_path.along_path_distance_m);

   if (reference_lookup_index < 1) {
      return GetPathDataAtIndex(vertical_path, reference_lookup_index);
   }

   aaesim::open_source::VerticalPathUtils::VerticalPathDataSet single_data_row{};
   single_data_row.resolved_index = reference_lookup_index;
   single_data_row.altitude_msl = Units::MetersLength(
         CoreUtils::LinearlyInterpolate(reference_lookup_index, distance_to_go.value(),
                                        vertical_path.along_path_distance_m, vertical_path.altitude_m));
   single_data_row.along_path_distance = estimated_distance_to_path_end;
   single_data_row.calibrated_airspeed = Units::MetersPerSecondSpeed(CoreUtils::LinearlyInterpolate(
         reference_lookup_index, distance_to_go.value(), vertical_path.along_path_distance_m, vertical_path.cas_mps));
   single_data_row.mach = CoreUtils::LinearlyInterpolate(reference_lookup_index, distance_to_go.value(),
                                                         vertical_path.along_path_distance_m, vertical_path.mach);
   single_data_row.altitude_rate = Units::MetersPerSecondSpeed(
         CoreUtils::LinearlyInterpolate(reference_lookup_index, distance_to_go.value(),
                                        vertical_path.along_path_distance_m, vertical_path.altitude_rate_mps));
   single_data_row.true_airspeed = CoreUtils::LinearlyInterpolate(
         reference_lookup_index, distance_to_go, vertical_path.along_path_distance_m, vertical_path.true_airspeed);
   single_data_row.tas_rate = Units::MetersSecondAcceleration(
         CoreUtils::LinearlyInterpolate(reference_lookup_index, distance_to_go.value(),
                                        vertical_path.along_path_distance_m, vertical_path.tas_rate_mps));
   single_data_row.theta = Units::RadiansAngle(
         CoreUtils::LinearlyInterpolate(reference_lookup_index, distance_to_go.value(),
                                        vertical_path.along_path_distance_m, vertical_path.theta_radians));
   single_data_row.ground_speed = Units::MetersPerSecondSpeed(CoreUtils::LinearlyInterpolate(
         reference_lookup_index, distance_to_go.value(), vertical_path.along_path_distance_m, vertical_path.gs_mps));
   single_data_row.time_to_go = Units::SecondsTime(
         CoreUtils::LinearlyInterpolate(reference_lookup_index, distance_to_go.value(),
                                        vertical_path.along_path_distance_m, vertical_path.time_to_go_sec));
   single_data_row.mass = Units::KilogramsMass(CoreUtils::LinearlyInterpolate(
         reference_lookup_index, distance_to_go.value(), vertical_path.along_path_distance_m, vertical_path.mass_kg));
   single_data_row.wind_velocity_east = CoreUtils::LinearlyInterpolate(
         reference_lookup_index, distance_to_go, vertical_path.along_path_distance_m, vertical_path.wind_velocity_east);
   single_data_row.wind_velocity_north =
         CoreUtils::LinearlyInterpolate(reference_lookup_index, distance_to_go, vertical_path.along_path_distance_m,
                                        vertical_path.wind_velocity_north);
   single_data_row.flap_setting = vertical_path.flap_setting[reference_lookup_index];
   return single_data_row;
}

inline Units::Speed aaesim::open_source::VerticalPathUtils::CalculateSpeedGuidance(
      const VerticalPath &vertical_path, Units::Length estimated_distance_to_path_end) {
   auto reference_lookup_index = CoreUtils::FindNearestIndex(
         Units::MetersLength(estimated_distance_to_path_end).value(), vertical_path.along_path_distance_m);

   Units::Speed cas_guidance = Units::zero();
   if (reference_lookup_index == 0) {
      cas_guidance = Units::MetersPerSecondSpeed(vertical_path.cas_mps[0]);
   } else {
      cas_guidance = Units::MetersPerSecondSpeed(CoreUtils::LinearlyInterpolate(
            reference_lookup_index, Units::MetersLength(estimated_distance_to_path_end).value(),
            vertical_path.along_path_distance_m, vertical_path.cas_mps));
   }

   return cas_guidance;
}

inline double aaesim::open_source::VerticalPathUtils::CalculateMachGuidance(
      const VerticalPath &vertical_path, Units::Length estimated_distance_to_path_end) {
   auto reference_lookup_index = CoreUtils::FindNearestIndex(
         Units::MetersLength(estimated_distance_to_path_end).value(), vertical_path.along_path_distance_m);

   double mach_guidance = 0;
   if (reference_lookup_index == 0) {
      mach_guidance = vertical_path.mach[0];
   } else {
      mach_guidance = CoreUtils::LinearlyInterpolate(reference_lookup_index,
                                                     Units::MetersLength(estimated_distance_to_path_end).value(),
                                                     vertical_path.along_path_distance_m, vertical_path.mach);
   }

   return mach_guidance;
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

inline Units::Mass aaesim::open_source::VerticalPathUtils::GetExpectedMass(
      const VerticalPath &vertical_path, Units::Length estimated_distance_to_path_end) {
   auto reference_lookup_index = CoreUtils::FindNearestIndex(
         Units::MetersLength(estimated_distance_to_path_end).value(), vertical_path.along_path_distance_m);

   Units::Mass mass = Units::zero();
   if (reference_lookup_index == 0) {
      mass = Units::KilogramsMass(vertical_path.mass_kg[0]);
   } else {
      mass = Units::KilogramsMass(CoreUtils::LinearlyInterpolate(
            reference_lookup_index, Units::MetersLength(estimated_distance_to_path_end).value(),
            vertical_path.along_path_distance_m, vertical_path.mass_kg));
   }

   return mass;
}
