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
// 2022 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#include <nlohmann/json.hpp>
#include "public/SpeedOnPitchControl.h"
#include "public/AircraftCalculations.h"

using namespace aaesim::open_source;

Units::Frequency SpeedOnPitchControl::m_gain_alt = Units::HertzFrequency(0.20);
Units::Frequency SpeedOnPitchControl::m_gain_gamma = Units::HertzFrequency(0.40);
Units::Frequency SpeedOnPitchControl::m_gain_phi = Units::HertzFrequency(0.40);
double SpeedOnPitchControl::m_gain_speedbrake = 0.10;

log4cplus::Logger SpeedOnPitchControl::m_logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("SpeedOnPitchControl"));

SpeedOnPitchControl::SpeedOnPitchControl(const Units::Speed speed_threshold,
                                         const Units::Length altitude_threshold) {
   // Map this class's gains to the parent class
   m_alt_gain = m_gain_alt;
   m_gamma_gain = m_gain_gamma;
   m_phi_gain = m_gain_phi;
   m_speed_brake_gain = m_gain_speedbrake;
   m_thrust_gain = CalculateThrustGain(); // use the parent's provided thrust gain calculation
   m_gain_velocity = Units::sqr(m_natural_frequency) / m_thrust_gain;

   m_speed_threshold = speed_threshold;
   m_altitude_threshold = altitude_threshold;
   m_is_level_flight = true;
   m_min_thrust_counter = 0.0;
   m_speed_brake_counter = 0.0;
   m_is_speed_brake_on = false;
}

void SpeedOnPitchControl::Initialize(std::shared_ptr<aaesim::BadaPerformanceCalculator> bada_calculator,
                                     const Units::Angle &max_bank_angle) {
   AircraftControl::Initialize(bada_calculator, max_bank_angle);
   m_is_level_flight = true;
   m_min_thrust_counter = 0.0;
   m_speed_brake_counter = 0.0;
   m_is_speed_brake_on = false;
}

void SpeedOnPitchControl::DoVerticalControl(const Guidance &guidance,
                                            const EquationsOfMotionState &equations_of_motion_state,
                                            Units::Force &thrust_command,
                                            Units::Angle &gamma_command,
                                            Units::Speed &true_airspeed_command,
                                            double &speed_brake_command,
                                            aaesim::open_source::bada_utils::FlapConfiguration &new_flap_configuration,
                                            const WeatherTruth &true_weather) {
   // Estimate kinetic forces for this state
   Units::Force lift, drag;
   ConfigureFlapsAndEstimateKineticForces(equations_of_motion_state, lift, drag, new_flap_configuration, true_weather);

   // Commands
   const Units::Speed ias_com = guidance.m_ias_command;
   true_airspeed_command = true_weather.CAS2TAS(ias_com, equations_of_motion_state.altitude_msl);
   const Units::Force max_thrust = Units::NewtonsForce(m_bada_calculator->GetMaxThrust(equations_of_motion_state.altitude_msl,
                                                                                       new_flap_configuration,
                                                                                       aaesim::open_source::bada_utils::EngineThrustMode::MAXIMUM_CRUISE, Units::ZERO_CELSIUS));
   const Units::Force min_thrust = Units::NewtonsForce(
      m_bada_calculator->GetMaxThrust(equations_of_motion_state.altitude_msl, new_flap_configuration, aaesim::open_source::bada_utils::EngineThrustMode::DESCENT, Units::ZERO_CELSIUS));
   const Units::Length alt_ref = Units::FeetLength(guidance.m_reference_altitude);
   const Units::Length error_alt = equations_of_motion_state.altitude_msl - alt_ref;
   const Units::Speed guidance_vertical_speed = guidance.m_vertical_speed;
   const Units::Speed error_tas = true_airspeed_command - equations_of_motion_state.true_airspeed;

   if (m_is_level_flight) {
      // manage speed with thrust and altitude with pitch
      SpeedOnThrustControl::DoVerticalControl(guidance,
                                              equations_of_motion_state,
                                              thrust_command,
                                              gamma_command,
                                              true_airspeed_command,
                                              speed_brake_command,
                                              new_flap_configuration,
                                              true_weather);

      // determine if staying in level flight
      const static Units::Length tolerance = Units::FeetLength(200);
      const bool altitude_above_constraint = equations_of_motion_state.altitude_msl - guidance.m_active_precalc_constraints.constraint_altLow > tolerance;
      const bool altitude_rate_nonzero = guidance_vertical_speed != Units::zero();
      if (altitude_above_constraint && altitude_rate_nonzero) {
         m_is_level_flight = false;
         thrust_command = min_thrust;
      }

   }
   else {
      // manage altitude with thrust, speed with pitch
      double esf = true_weather.ESFconstantCAS(equations_of_motion_state.true_airspeed, equations_of_motion_state.altitude_msl);

      // adjust esf based on velocity error compared to the speed threshold
      if (error_tas <= -m_speed_threshold) {
         esf = 0.3;
      } else if (error_tas > -m_speed_threshold && error_tas < Units::ZERO_SPEED) {
         esf = (esf - 0.3) / m_speed_threshold * error_tas + esf;
      } else if (error_tas > Units::ZERO_SPEED && error_tas < m_speed_threshold) {
         esf = (1.7 - esf) / m_speed_threshold * error_tas + esf;
      } else if (error_tas >= m_speed_threshold) {
         esf = 1.7;
      }

      // descent rate
      Units::Speed dh_dt = ((equations_of_motion_state.thrust - drag) * equations_of_motion_state.true_airspeed) / (m_ac_mass * Units::ONE_G_ACCELERATION) * esf;

      gamma_command = Units::RadiansAngle(asin(-dh_dt / equations_of_motion_state.true_airspeed));

      if (error_alt < -m_altitude_threshold) {
         thrust_command = 0.50 * max_thrust;
      } else if (error_alt > m_altitude_threshold) {
         thrust_command = min_thrust;
      } else {
         thrust_command = (min_thrust - 0.50 * max_thrust) / (m_altitude_threshold * 2) * error_alt + (0.50 * max_thrust + min_thrust) / 2.0;
      }

      // Check if flight should level off
      const bool altitude_near_constraint_low = equations_of_motion_state.altitude_msl - guidance.m_active_precalc_constraints.constraint_altLow < Units::FeetLength(100);
      const bool altitude_rate_is_zero = guidance_vertical_speed == Units::zero();
      if (altitude_near_constraint_low || altitude_rate_is_zero) {
         m_is_level_flight = true;
      }

   }

   // limit thrust to max and min limits
   if (thrust_command > max_thrust) {
      thrust_command = max_thrust;
   } else if (thrust_command < min_thrust) {
      thrust_command = min_thrust;
   }

   // Determine if speed brake is needed
   Units::Speed v_cas = true_weather.TAS2CAS(equations_of_motion_state.true_airspeed, equations_of_motion_state.altitude_msl);

   if (thrust_command == min_thrust) {

      aaesim::open_source::bada_utils::FlapConfiguration updated_flap_configuration = aaesim::open_source::bada_utils::FlapConfiguration::UNDEFINED;

      if (m_is_level_flight || (error_alt > m_altitude_threshold))
      {
         m_bada_calculator->GetConfigurationForIncreasedDrag(v_cas,
                                                             Units::MetersLength(equations_of_motion_state.altitude_msl),
                                                             updated_flap_configuration);
         new_flap_configuration = updated_flap_configuration;
      }

      // test for minThrustCounter > 14, because in SpeedOnThrustControl, minThrustCounter
      // is incremented before this test instead of after.
      if (m_is_level_flight && m_min_thrust_counter > 14.0 && error_tas < Units::KnotsSpeed(-5.0)){
         if (new_flap_configuration <= aaesim::open_source::bada_utils::FlapConfiguration::LANDING) {
            if (!m_is_speed_brake_on) {
               m_speed_brake_counter = 0.0;
               speed_brake_command = 0.5;
               m_is_speed_brake_on = true;
            }
            else {
               m_speed_brake_counter += 1;
               speed_brake_command = 0.5;
            }
         }
         else {
            m_speed_brake_counter = 0.0;
            speed_brake_command = 0.0;
            m_is_speed_brake_on = false;
         }
      }

      if (error_alt > m_altitude_threshold) {
         m_bada_calculator->GetConfigurationForIncreasedDrag(v_cas,
                                                             Units::MetersLength(equations_of_motion_state.altitude_msl),
                                                             updated_flap_configuration);

         if (updated_flap_configuration == new_flap_configuration && updated_flap_configuration <= aaesim::open_source::bada_utils::FlapConfiguration::LANDING) {
            speed_brake_command = 0.5;
            m_is_speed_brake_on = true;
            m_speed_brake_counter = m_speed_brake_counter + 1;
         }

         new_flap_configuration = updated_flap_configuration;
      }
      m_min_thrust_counter = m_min_thrust_counter + 1;
   } else {
      m_min_thrust_counter = 0;
   }

   // If no longer commanding Min Thrust
   if (m_is_speed_brake_on && m_speed_brake_counter <= 30.0) {
      m_speed_brake_counter = m_speed_brake_counter + 1;
      speed_brake_command = 0.5;
   } else if (m_is_speed_brake_on && m_speed_brake_counter > 30.0) {
      if (m_min_thrust_counter == 0) {
         m_speed_brake_counter = 0.0;
         speed_brake_command = 0.0;
         m_is_speed_brake_on = false;
      } else {
         m_speed_brake_counter = m_speed_brake_counter + 1;
         speed_brake_command = 0.5;
      }
   }

   if (m_logger.getLogLevel() == log4cplus::TRACE_LOG_LEVEL) {
      using json = nlohmann::json;
      json j;
      j["altitude_error_ft"] = Units::FeetLength(error_alt).value();
      j["is_level_flight_bool"] = m_is_level_flight;
      j["thrust_command_newtons"] = Units::NewtonsForce(thrust_command).value();
      j["dynamics_thrust_newtons"] = Units::NewtonsForce(equations_of_motion_state.thrust).value();
      j["max_thrust_newtons"] = Units::NewtonsForce(max_thrust).value();
      j["min_thrust_newtons"] = Units::NewtonsForce(min_thrust).value();
      j["true_airspeed_error_knots"] = Units::KnotsSpeed(error_tas).value();
      j["speed_brake_command"] = speed_brake_command;
      j["new_flap_configuration"] = bada_utils::GetFlapConfigurationAsString(new_flap_configuration);
      j["gamma_command_deg"] = Units::DegreesAngle(gamma_command).value();
      LOG4CPLUS_TRACE(m_logger, j.dump());
   }

}
