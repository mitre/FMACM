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

#include "public/SpeedOnThrustControl.h"

#include <nlohmann/json.hpp>

#include "public/Environment.h"

using namespace aaesim::open_source;

void SpeedOnThrustControl::ComputeVerticalCommands(
      const Guidance &guidance, const EquationsOfMotionState &equations_of_motion_state,
      std::shared_ptr<const aaesim::open_source::TrueWeatherOperator> &sensed_weather, Units::Force &thrust_command,
      Units::Angle &gamma_command, Units::Speed &tas_command, BoundedValue<double, 0, 1> &speed_brake_command,
      aaesim::open_source::bada_utils::FlapConfiguration &flap_configuration) {
   const Units::Speed hdot_ref = guidance.m_vertical_speed;
   const Units::Length alt_ref = guidance.m_reference_altitude;
   const Units::Length error_alt = alt_ref - equations_of_motion_state.altitude_msl;

   double temp_gamma = -(hdot_ref + gain_altitude_ * error_alt) / equations_of_motion_state.true_airspeed;
   if (temp_gamma > 1.0) {
      temp_gamma = 1.0;
   } else if (temp_gamma < -1.0) {
      temp_gamma = -1.0;
   }
   gamma_command = Units::RadiansAngle(asin(temp_gamma));

   tas_command =
         sensed_weather->GetTrueWeather()->CAS2TAS(guidance.m_ias_command, equations_of_motion_state.altitude_msl);

   const Units::Speed error_tas = tas_command - equations_of_motion_state.true_airspeed;
   const Units::Acceleration vel_dot_com = gain_true_airspeed_ * error_tas;

   Units::Force lift{}, drag{};
   ConfigureFlapsAndEstimateKineticForces(equations_of_motion_state, sensed_weather, aircraft_performance_, lift, drag,
                                          flap_configuration);

   // Thrust to maintain speed
   const auto ac_mass = aircraft_performance_->GetAircraftMass();
   const Units::Force thrust_equilibrium =
         ac_mass * vel_dot_com + drag - ac_mass * Units::ONE_G_ACCELERATION * sin(equations_of_motion_state.gamma) -
         ac_mass * equations_of_motion_state.true_airspeed *
               (sensed_weather->GetWindSpeedVerticalDerivativeEast() * cos(equations_of_motion_state.psi_enu) +
                sensed_weather->GetWindSpeedVerticalDerivativeNorth() * sin(equations_of_motion_state.psi_enu)) *
               sin(equations_of_motion_state.gamma) * cos(equations_of_motion_state.gamma);
   const Units::Force max_thrust = Units::NewtonsForce(aircraft_performance_->GetMaxThrust(
         equations_of_motion_state.altitude_msl, flap_configuration,
         aaesim::open_source::bada_utils::EngineThrustMode::MAXIMUM_CRUISE, Units::ZERO_CELSIUS));
   const Units::Force min_thrust = Units::NewtonsForce(aircraft_performance_->GetMaxThrust(
         equations_of_motion_state.altitude_msl, flap_configuration,
         aaesim::open_source::bada_utils::EngineThrustMode::DESCENT, Units::ZERO_CELSIUS));

   // Check Configuration if min_thrust is commanded
   thrust_command = thrust_equilibrium;
   bool min_thrust_commanded{false};
   if (thrust_equilibrium < min_thrust) {
      thrust_command = min_thrust;
      min_thrust_commanded = true;
      ++min_thrust_counter_;

      const Units::Speed calibrated_airspeed = sensed_weather->GetTrueWeather()->TAS2CAS(
            equations_of_motion_state.true_airspeed, equations_of_motion_state.altitude_msl);

      aaesim::open_source::bada_utils::FlapConfiguration updated_flap_configuration =
            aaesim::open_source::bada_utils::FlapConfiguration::UNDEFINED;
      aircraft_performance_->GetConfigurationForIncreasedDrag(
            calibrated_airspeed, equations_of_motion_state.altitude_msl, updated_flap_configuration);
      flap_configuration = updated_flap_configuration;
   } else {
      min_thrust_counter_ = 0;
      if (thrust_equilibrium > max_thrust) {
         thrust_command = max_thrust;
      }
   }

   static const Units::KnotsSpeed tas_error_tolerance{-5};
   static const unsigned int minimum_thrust_duration{15};
   if (min_thrust_counter_ > minimum_thrust_duration and error_tas < tas_error_tolerance) {
      if (flap_configuration <= aaesim::open_source::bada_utils::FlapConfiguration::LANDING) {
         speed_brake_controller_->Deploy();
      } else {
         speed_brake_controller_->Retract();
      }
   }
   speed_brake_command = speed_brake_controller_->Update(min_thrust_commanded);

   DoLogging(logger_, equations_of_motion_state, error_alt, true, thrust_command, max_thrust, min_thrust, error_tas,
             speed_brake_command, flap_configuration, gamma_command);
}
