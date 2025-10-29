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

#include "public/DefaultLateralController.h"

#include "public/CoreUtils.h"

Units::Angle aaesim::open_source::DefaultLateralController::ComputeRollCommand(
      const Guidance &guidance, const EquationsOfMotionState &equations_of_motion_state,
      std::shared_ptr<const aaesim::open_source::TrueWeatherOperator> &sensed_weather) {
   const Units::InvertedLength k_xtrk = Units::PerMeterInvertedLength(5e-4);  // meters^-1
   const double k_trk = 3;                                                    // unitless

   // States
   const Units::Speed tas = equations_of_motion_state.true_airspeed;  // true airspeed
   const Units::Angle gamma = equations_of_motion_state.gamma;        // flight-path angle
   const Units::Angle psi = equations_of_motion_state.psi_enu;  // heading angle measured from east counter-clockwise

   const Units::Speed Vw_para = sensed_weather->GetWindSpeedEast() * cos(guidance.m_enu_track_angle) +
                                sensed_weather->GetWindSpeedNorth() * sin(guidance.m_enu_track_angle);
   const Units::Speed Vw_perp = -sensed_weather->GetWindSpeedEast() * sin(guidance.m_enu_track_angle) +
                                sensed_weather->GetWindSpeedNorth() * cos(guidance.m_enu_track_angle);

   const Units::Speed wind_magnitude =
         sqrt(Units::sqr(sensed_weather->GetWindSpeedEast()) + Units::sqr(sensed_weather->GetWindSpeedNorth()));
   const Units::Speed estimated_ground_speed = sqrt(Units::sqr(tas * cos(gamma)) - Units::sqr(Vw_perp)) + Vw_para;

   double temp = (Units::sqr(tas * cos(gamma)) + Units::sqr(estimated_ground_speed) - Units::sqr(wind_magnitude)) /
                 (tas * 2 * cos(gamma) * estimated_ground_speed);

   // Limit temp so acos function doesn't give undefined value.
   if (temp > 1.0) {
      temp = 1.0;
   } else if (temp < -1.0) {
      temp = -1.0;
   }

   const Units::Angle beta =
         Units::RadiansAngle(acos(temp)) * -1.0 * CoreUtils::SignOfValue(Units::MetersPerSecondSpeed(Vw_perp).value());

   // Convert track guidance to heading using winds (beta is the Wind Correction Angle)
   Units::Angle heading_command = guidance.m_enu_track_angle + beta;

   // Error in heading angle
   Units::SignedAngle e_trk = heading_command - psi;
   e_trk.normalize();

   // Along-path distance and Cross-track Error
   Units::Length e_xtrk = Units::zero();

   // check if guidance has cross track error and use it if so
   if (guidance.m_use_cross_track) {
      if (guidance.m_reference_bank_angle != Units::ZERO_ANGLE) {
         e_xtrk = guidance.m_cross_track_error - (guidance.m_reference_bank_angle / k_xtrk);
      } else {
         e_xtrk = guidance.m_cross_track_error;
      }
   }

   // Calculate commanded roll angle
   // We had to add a conversion from unitless to radians in the formula for roll_angle_command.
   Units::Angle roll_angle_command = -k_xtrk * e_xtrk * Units::ONE_RADIAN_ANGLE - k_trk * e_trk;
   const double unlimited_roll_angle_command = Units::RadiansAngle(roll_angle_command).value();

   // Limit the commanded roll angle
   double sign_roll_command = CoreUtils::SignOfValue(unlimited_roll_angle_command);
   if (roll_angle_command * sign_roll_command > max_bank_angle_) {
      roll_angle_command = max_bank_angle_ * sign_roll_command;
   }

   DoLogging(e_xtrk, e_trk, roll_angle_command);

   return roll_angle_command;
};
