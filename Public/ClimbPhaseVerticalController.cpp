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

#include "public/ClimbPhaseVerticalController.h"

void aaesim::open_source::ClimbPhaseVerticalController::ComputeAscentCommands(
      const Guidance &guidance, const EquationsOfMotionState &equations_of_motion_state,
      std::shared_ptr<const aaesim::open_source::TrueWeatherOperator> &sensed_weather, Units::Force &thrust_command,
      Units::Angle &gamma_command, Units::Speed &tas_command,
      aaesim::open_source::bada_utils::FlapConfiguration &flap_command) {
   const Units::Frequency gain_altitude = Units::HertzFrequency(0.20);
   const Units::Frequency velocity_gain = Units::sqr(natural_frequency_) / thrust_gain_;

   const Units::Speed hdot_ref = guidance.m_vertical_speed;
   const Units::Length alt_ref = guidance.m_reference_altitude;
   const Units::Length error_alt = alt_ref - equations_of_motion_state.altitude_msl;

   double temp_gamma = -(hdot_ref + gain_altitude * error_alt) /
                       equations_of_motion_state.true_airspeed;  // calculate change in altitude
   // if fabs(change) > 1 set to 1 required for asin calculation
   if (temp_gamma > 1.0) {
      temp_gamma = 1.0;
   } else if (temp_gamma < -1.0) {
      temp_gamma = -1.0;
   }
   gamma_command = Units::RadiansAngle(asin(temp_gamma));

   // Speed Control
   tas_command =
         sensed_weather->GetTrueWeather()->CAS2TAS(guidance.m_ias_command, equations_of_motion_state.altitude_msl);

   // Speed Error
   Units::Speed error_tas = tas_command - equations_of_motion_state.true_airspeed;
   Units::Acceleration vel_dot_com = velocity_gain * error_tas;

   // Estimate kinetic forces for this state
   Units::Force lift{}, drag{};
   ConfigureFlapsAndEstimateKineticForces(equations_of_motion_state, sensed_weather, aircraft_performance_, lift, drag,
                                          flap_command);

   // Thrust to maintain speed
   // Nominal Thrust (no acceleration) at desired speed
   const auto ac_mass = aircraft_performance_->GetAircraftMass();
   Units::Force thrust_nominal =
         ac_mass * vel_dot_com + drag - ac_mass * Units::ONE_G_ACCELERATION * sin(equations_of_motion_state.gamma) -
         ac_mass * equations_of_motion_state.true_airspeed *
               (sensed_weather->GetWindSpeedVerticalDerivativeEast() * cos(equations_of_motion_state.psi_enu) +
                sensed_weather->GetWindSpeedVerticalDerivativeNorth() * sin(equations_of_motion_state.psi_enu)) *
               sin(equations_of_motion_state.gamma) * cos(equations_of_motion_state.gamma);
   thrust_command = thrust_nominal;

   // Thrust Limits
   Units::Force max_thrust = Units::NewtonsForce(aircraft_performance_->GetMaxThrust(
         equations_of_motion_state.altitude_msl, flap_command,
         aaesim::open_source::bada_utils::EngineThrustMode::MAXIMUM_CLIMB, Units::ZERO_CELSIUS));
   Units::Force min_thrust = Units::NewtonsForce(aircraft_performance_->GetMaxThrust(
         equations_of_motion_state.altitude_msl, flap_command,
         aaesim::open_source::bada_utils::EngineThrustMode::DESCENT, Units::ZERO_CELSIUS));

   DoLogging(error_alt, thrust_command, equations_of_motion_state.thrust, min_thrust, max_thrust, flap_command,
             error_tas, tas_command, gamma_command);
};