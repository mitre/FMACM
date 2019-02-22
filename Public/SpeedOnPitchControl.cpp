// ****************************************************************************
// NOTICE
//
// This is the copyright work of The MITRE Corporation, and was produced
// for the U. S. Government under Contract Number DTFAWA-10-C-00080, and
// is subject to Federal Aviation Administration Acquisition Management
// System Clause 3.5-13, Rights In Data-General, Alt. III and Alt. IV
// (Oct. 1996).  No other use other than that granted to the U. S.
// Government, or to those acting on behalf of the U. S. Government,
// under that Clause is authorized without the express written
// permission of The MITRE Corporation. For further information, please
// contact The MITRE Corporation, Contracts Office, 7515 Colshire Drive,
// McLean, VA  22102-7539, (703) 983-6000. 
//
// Copyright 2019 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#include "public/SpeedOnPitchControl.h"
#include "public/AircraftCalculations.h"

Units::Frequency SpeedOnPitchControl::m_gain_alt = Units::HertzFrequency(0.20);
Units::Frequency SpeedOnPitchControl::m_gain_gamma = Units::HertzFrequency(0.40);
Units::Frequency SpeedOnPitchControl::m_gain_phi = Units::HertzFrequency(0.40);
double SpeedOnPitchControl::m_gain_speedbrake = 0.10;

log4cplus::Logger SpeedOnPitchControl::m_logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("SpeedOnPitchControl"));

SpeedOnPitchControl::SpeedOnPitchControl(const Units::Speed speedThreshold,
                                         const Units::Length altitudeThreshold) {
   // Map this class's gains to the parent class
   m_alt_gain = m_gain_alt;
   m_gamma_gain = m_gain_gamma;
   m_phi_gain = m_gain_phi;
   m_speed_brake_gain = m_gain_speedbrake;
   m_thrust_gain = calculateThrustGain(); // use the parent's provided thrust gain calculation
   m_gain_velocity = Units::sqr(m_natural_frequency) / m_thrust_gain;

   m_speed_threshold = speedThreshold;
   m_altitude_threshold = altitudeThreshold;
   m_is_level_flight = true;
   m_min_thrust_counter = 0.0;
   m_speed_brake_counter = 0.0;
   m_is_speed_brake_on = false;
}

void SpeedOnPitchControl::Initialize(BadaWithCalc &bada_calculator,
                                     const Units::Length &altitude_msl_at_faf,
                                     const Units::Angle &max_bank_angle,
                                     const PrecalcWaypoint &final_waypoint) {
   AircraftControl::Initialize(bada_calculator, altitude_msl_at_faf, max_bank_angle, final_waypoint);
   m_is_level_flight = true;
   m_min_thrust_counter = 0.0;
   m_speed_brake_counter = 0.0;
   m_is_speed_brake_on = false;
}


ControlCommands SpeedOnPitchControl::CalculateControlCommands(const Guidance &guidance,
                                                              const EquationsOfMotionState &eqmState,
                                                              const WeatherTruth &wind) {

   // Get Winds and Wind Gradients at altitude
   calculateSensedWind(wind, Units::MetersLength(eqmState.enu_z));

   // Lateral Control
   Units::Angle phi_com = doLateralControl(guidance, eqmState);

   // Vertical Control
   double speedBrakeCom(0);
   int flapConfig_new;
   Units::Force T_com;
   Units::Angle gamma_com;
   Units::Speed tas_com;
   DoVerticalControl(guidance, eqmState, T_com, gamma_com, tas_com, speedBrakeCom, flapConfig_new, wind);

   ControlCommands controlCommands(phi_com, T_com, gamma_com, tas_com, speedBrakeCom, flapConfig_new);
   return controlCommands;
}

void SpeedOnPitchControl::DoVerticalControl(const Guidance &guidance,
                                            const EquationsOfMotionState &eqmState,
                                            Units::Force &thrust_command,
                                            Units::Angle &gamma_command,
                                            Units::Speed &true_airspeed_command,
                                            double &speed_brake_command,
                                            int &new_flap_config,
                                            const WeatherTruth &true_weather) {

   // Estimate kinetic forces for this state
   Units::Force lift, drag;
   estimateKineticForces(eqmState, lift, drag, new_flap_config, true_weather);

   // Commands
   const Units::Speed ias_com = guidance.m_ias_command;
   true_airspeed_command = true_weather.getAtmosphere()->CAS2TAS(ias_com, eqmState.enu_z);

   const Units::Force maxThrust = Units::NewtonsForce(m_bada_calculator->getMaxThrust(eqmState.enu_z, new_flap_config, "cruise"));
   const Units::Force minThrust = Units::NewtonsForce(
         m_bada_calculator->getMaxThrust(eqmState.enu_z, new_flap_config, "descent"));

   //Speed Management Method check
   const Units::Length alt_ref = Units::FeetLength(guidance.m_reference_altitude);
   const Units::Length e_alt = eqmState.enu_z - alt_ref;
   const Units::Speed h_dot = guidance.m_vertical_speed;
   thrust_command = Units::NewtonsForce(0.0);
   gamma_command = Units::RadiansAngle(0.0);
   Units::Speed ev = true_airspeed_command - eqmState.true_airspeed;

   // LEVEL FLIGHT - manage speed with thrust and altitude with pitch
   if (m_is_level_flight) {
      Units::Acceleration vDotCom = m_gain_velocity * ev;

      // calculate the Thrust Command
      thrust_command = m_ac_mass * vDotCom
              + drag
              - m_ac_mass * Units::ONE_G_ACCELERATION * sin(eqmState.gamma)
              - m_ac_mass * eqmState.true_airspeed * (m_dVwx_dh * cos(eqmState.psi) + m_dVwy_dh * sin(eqmState.psi))
                * sin(eqmState.gamma) * cos(eqmState.gamma);

      // command a level altitude
      gamma_command = Units::RadiansAngle(0.0);

      if (Units::MetersLength(eqmState.enu_z).value() - guidance.m_active_precalc_constraints.constraint_altLow > 200.0 * FEET_TO_METERS &&
          Units::MetersPerSecondSpeed(h_dot).value() != 0.0) {
         m_is_level_flight = false;
         thrust_command = minThrust;
      }

   }
      // DESCENDING FLIGHT - manage altitude with thrust, speed with pitch
   else {
      double esf = AircraftCalculations::ESFconstantCAS(eqmState.true_airspeed, eqmState.enu_z,
                                                        true_weather.getAtmosphere()->GetTemperature(eqmState.enu_z));

      // adjust esf based on velocity error compared to the speed threshold
      if (ev <= -m_speed_threshold) {
         esf = 0.3;
      } else if (ev > -m_speed_threshold && ev < Units::ZERO_SPEED) {
         esf = (esf - 0.3) / m_speed_threshold * ev + esf;
      } else if (ev > Units::ZERO_SPEED && ev < m_speed_threshold) {
         esf = (1.7 - esf) / m_speed_threshold * ev + esf;
      } else if (ev >= m_speed_threshold) {
         esf = 1.7;
      }

      // descent rate
      Units::Speed dh_dt = ((eqmState.thrust - drag) * eqmState.true_airspeed) / (m_ac_mass * Units::ONE_G_ACCELERATION) * esf;

      gamma_command = Units::RadiansAngle(asin(-dh_dt / eqmState.true_airspeed));

      //if(gamma_com > 4.0*PI/180)
      //	gamma_com = 4.0*PI/180;

      if (e_alt < -m_altitude_threshold) {
         thrust_command = 0.50 * maxThrust;
      } else if (e_alt > m_altitude_threshold) {
         thrust_command = minThrust;
      } else {
         thrust_command = (minThrust - 0.50 * maxThrust) / (m_altitude_threshold * 2) * e_alt + (0.50 * maxThrust + minThrust) / 2.0;
      }

      // Check if flight should level off
      if (Units::MetersLength(eqmState.enu_z).value() - guidance.m_active_precalc_constraints.constraint_altLow < 100.0 * FEET_TO_METERS
              || Units::MetersPerSecondSpeed(h_dot).value() == 0.0) {
         m_is_level_flight = true;
      }

   }

   // limit thrust to max and min limits
   if (thrust_command > maxThrust) {
      thrust_command = maxThrust;
   } else if (thrust_command < minThrust) {
      thrust_command = minThrust;
   }

   // Determine if speed brake is needed
   Units::Speed v_cas = true_weather.getAtmosphere()->TAS2CAS(Units::MetersPerSecondSpeed(eqmState.true_airspeed), Units::MetersLength(
         eqmState.enu_z)); // current indicated airspeed in meters per second

   if (thrust_command == minThrust) {

      int mode_new = 0;

      if (m_is_level_flight || (e_alt > m_altitude_threshold))
      {
         m_bada_calculator->getConfigForDrag(v_cas,
                                         Units::MetersLength(eqmState.enu_z),
                                         m_alt_at_FAF,
                                         new_flap_config,
                                         mode_new);
         new_flap_config = mode_new;
      }

      // test for minThrustCounter > 14, because in SpeedOnThrustControl, minThrustCounter
      // is incremented before this test instead of after.
      if (m_is_level_flight && m_min_thrust_counter > 14.0 && ev < Units::KnotsSpeed(-5.0)){
         if (new_flap_config <= 2) {
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

      if (e_alt > m_altitude_threshold) {
         m_bada_calculator->getConfigForDrag(v_cas,
                                         Units::MetersLength(eqmState.enu_z),
                                         m_alt_at_FAF,
                                         new_flap_config,
                                         mode_new);

         if (mode_new == new_flap_config && mode_new <= 2) {
            speed_brake_command = 0.5;
            m_is_speed_brake_on = true;
            m_speed_brake_counter = m_speed_brake_counter + 1;
         }

         new_flap_config = mode_new;
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

}
