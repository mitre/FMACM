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

#include "public/SpeedOnPitchControl.h"

#include <nlohmann/json.hpp>

using namespace aaesim::open_source;

void SpeedOnPitchControl::Initialize(
      std::shared_ptr<aaesim::open_source::FixedMassAircraftPerformance> &aircraft_performance) {
   AbstractDescentController::Initialize(aircraft_performance);
   speed_on_thrust_controller_->Initialize(aircraft_performance_);
}

void SpeedOnPitchControl::ComputeVerticalCommands(
      const Guidance &guidance, const EquationsOfMotionState &equations_of_motion_state,
      std::shared_ptr<const aaesim::open_source::TrueWeatherOperator> &sensed_weather, Units::Force &thrust_command,
      Units::Angle &gamma_command, Units::Speed &true_airspeed_command, BoundedValue<double, 0, 1> &speed_brake_command,
      aaesim::open_source::bada_utils::FlapConfiguration &flap_configuration) {
   Units::Force lift, drag;
   ConfigureFlapsAndEstimateKineticForces(equations_of_motion_state, sensed_weather, aircraft_performance_, lift, drag,
                                          flap_configuration);

   // Commands
   const Units::Speed ias_com = guidance.m_ias_command;
   true_airspeed_command = sensed_weather->GetTrueWeather()->CAS2TAS(ias_com, equations_of_motion_state.altitude_msl);
   const Units::Force max_thrust = Units::NewtonsForce(aircraft_performance_->GetMaxThrust(
         equations_of_motion_state.altitude_msl, flap_configuration,
         aaesim::open_source::bada_utils::EngineThrustMode::MAXIMUM_CRUISE, Units::ZERO_CELSIUS));
   const Units::Force min_thrust = Units::NewtonsForce(aircraft_performance_->GetMaxThrust(
         equations_of_motion_state.altitude_msl, flap_configuration,
         aaesim::open_source::bada_utils::EngineThrustMode::DESCENT, Units::ZERO_CELSIUS));
   const Units::Length alt_ref = Units::FeetLength(guidance.m_reference_altitude);
   const Units::Length error_alt = equations_of_motion_state.altitude_msl - alt_ref;
   const Units::Speed guidance_vertical_speed = guidance.m_vertical_speed;
   const Units::Speed error_tas = true_airspeed_command - equations_of_motion_state.true_airspeed;

   if (is_level_flight_) {
      speed_on_thrust_controller_->ComputeVerticalCommands(guidance, equations_of_motion_state, sensed_weather,
                                                           thrust_command, gamma_command, true_airspeed_command,
                                                           speed_brake_command, flap_configuration);

      // determine if staying in level flight
      const static Units::Length tolerance = Units::FeetLength(200);
      const bool altitude_above_constraint =
            equations_of_motion_state.altitude_msl - guidance.m_active_precalc_constraints.constraint_altLow >
            tolerance;
      const bool altitude_rate_nonzero = guidance_vertical_speed != Units::zero();
      if (altitude_above_constraint && altitude_rate_nonzero) {
         is_level_flight_ = false;
         thrust_command = min_thrust;
      }

      return;
   }

   // manage altitude with thrust, speed with pitch
   double esf = sensed_weather->GetTrueWeather()->ESFconstantCAS(equations_of_motion_state.true_airspeed,
                                                                 equations_of_motion_state.altitude_msl);

   // adjust esf based on velocity error compared to the speed threshold
   if (error_tas <= -speed_threshold_) {
      esf = 0.3;
   } else if (error_tas > -speed_threshold_ && error_tas <= Units::ZERO_SPEED) {
      esf = (esf - 0.3) / speed_threshold_ * error_tas + esf;
   } else if (error_tas > Units::ZERO_SPEED && error_tas <= speed_threshold_) {
      esf = (1.7 - esf) / speed_threshold_ * error_tas + esf;
   } else if (error_tas > speed_threshold_) {
      esf = 1.7;
   }

   // descent rate
   const auto ac_mass = aircraft_performance_->GetAircraftMass();
   Units::Speed dh_dt = ((equations_of_motion_state.thrust - drag) * equations_of_motion_state.true_airspeed) /
                        (ac_mass * Units::ONE_G_ACCELERATION) * esf;

   gamma_command = Units::RadiansAngle(asin(-dh_dt / equations_of_motion_state.true_airspeed));

   if (error_alt < -altitude_threshold_) {
      thrust_command = 0.50 * max_thrust;
   } else if (error_alt > altitude_threshold_) {
      thrust_command = min_thrust;
   } else {
      thrust_command = (min_thrust - 0.50 * max_thrust) / (altitude_threshold_ * 2) * error_alt +
                       (0.50 * max_thrust + min_thrust) / 2.0;
   }

   // Check if flight should level off
   const bool altitude_near_constraint_low =
         equations_of_motion_state.altitude_msl - guidance.m_active_precalc_constraints.constraint_altLow <
         Units::FeetLength(100);
   const bool altitude_rate_is_zero = guidance_vertical_speed == Units::zero();
   if (altitude_near_constraint_low || altitude_rate_is_zero) {
      is_level_flight_ = true;
   }

   // limit thrust to max and min limits
   bool min_thrust_commanded = false;
   if (thrust_command > max_thrust) {
      thrust_command = max_thrust;
   } else if (thrust_command < min_thrust) {
      thrust_command = min_thrust;
      min_thrust_commanded = true;
   }

   // Determine if speed brake is needed
   if (min_thrust_commanded) {
      aaesim::open_source::bada_utils::FlapConfiguration updated_flap_configuration{
            aaesim::open_source::bada_utils::FlapConfiguration::UNDEFINED};
      if (error_alt > altitude_threshold_) {
         Units::Speed v_cas = sensed_weather->GetTrueWeather()->TAS2CAS(equations_of_motion_state.true_airspeed,
                                                                        equations_of_motion_state.altitude_msl);
         aircraft_performance_->GetConfigurationForIncreasedDrag(
               v_cas, Units::MetersLength(equations_of_motion_state.altitude_msl), updated_flap_configuration);

         if (updated_flap_configuration == flap_configuration &&
             updated_flap_configuration <= aaesim::open_source::bada_utils::FlapConfiguration::LANDING) {
            speed_brake_controller_->Deploy();
         }
         flap_configuration = updated_flap_configuration;
      }
   }
   speed_brake_command = speed_brake_controller_->Update(min_thrust_commanded);

   DoLogging(logger_, equations_of_motion_state, error_alt, is_level_flight_, thrust_command, max_thrust, min_thrust,
             error_tas, speed_brake_command, flap_configuration, gamma_command);
}
