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

#include "public/SpeedOnThrustControl.h"
#include "public/Environment.h"

Units::Frequency SpeedOnThrustControl::m_gain_altitude = Units::HertzFrequency(0.20);
Units::Frequency SpeedOnThrustControl::m_gain_gamma = Units::HertzFrequency(0.20);
Units::Frequency SpeedOnThrustControl::m_gain_phi = Units::HertzFrequency(0.40);
double SpeedOnThrustControl::m_gain_speedbrake = 0.20;

log4cplus::Logger SpeedOnThrustControl::m_logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("SpeedOnThrustControl"));

SpeedOnThrustControl::SpeedOnThrustControl() {
   // Map this class's gains to the parent class
   m_alt_gain = m_gain_altitude;
   m_gamma_gain = m_gain_gamma;
   m_phi_gain = m_gain_phi;
   m_speed_brake_gain = m_gain_speedbrake;
   m_thrust_gain = calculateThrustGain(); // use the parent's provided thrust gain calculation
   m_gain_velocity = Units::sqr(m_natural_frequency) / m_thrust_gain; // new velocity gain, roughly .117

   m_min_thrust_counter = 0.0;
   m_speedbrake_counter = 0.0;
   m_is_speedbrake_on = false;
}

void SpeedOnThrustControl::Initialize(BadaWithCalc &bada_calculator,
                                      const Units::Length &altitude_at_final_waypoint,
                                      const Units::Angle &max_bank_angle,
                                      const PrecalcWaypoint &final_waypoint) {
   AircraftControl::Initialize(bada_calculator, altitude_at_final_waypoint, max_bank_angle, final_waypoint);
   m_min_thrust_counter = 0.0;
   m_speedbrake_counter = 0.0;
   m_is_speedbrake_on = false;
}

ControlCommands SpeedOnThrustControl::CalculateControlCommands(const Guidance &guidance,
                                                               const EquationsOfMotionState &eqmState,
                                                               const WeatherTruth &truth_wind) {

   // Update environmental wind
   calculateSensedWind(truth_wind, Units::MetersLength(eqmState.enu_z));

   // Lateral Control
   Units::Angle phi_com = doLateralControl(guidance, eqmState);

   // Vertical Control
   int newFlapConfig;
   double speedBrakeCom = 0.0;
   Units::Force T_com;
   Units::Angle gamma_com;
   Units::Speed tas_com;
   DoVerticalControl(guidance, eqmState, T_com, gamma_com, tas_com, speedBrakeCom, newFlapConfig, truth_wind);

   ControlCommands controlCommands(phi_com, T_com, gamma_com, tas_com, speedBrakeCom, newFlapConfig);
   return controlCommands;
}

void SpeedOnThrustControl::DoVerticalControl(const Guidance &guidance,
                                             const EquationsOfMotionState &eqmState,
                                             Units::Force &thrust_command,
                                             Units::Angle &gamma_com,
                                             Units::Speed &tas_com,
                                             double &speed_brake_command,
                                             int &flapConfig_new,
                                             const WeatherTruth &weather) {

   const Units::Speed hdot_ref = guidance.m_vertical_speed;
   const Units::Length alt_ref = guidance.m_reference_altitude;
   const Units::Length e_alt = alt_ref - eqmState.enu_z;

   double temp_gamma = -(hdot_ref + m_gain_altitude * e_alt) / eqmState.true_airspeed; // calculate change in altitude
   // if fabs(change) > 1 set to 1 required for asin calculation
   if (temp_gamma > 1.0) {
      temp_gamma = 1.0;
   } else if (temp_gamma < -1.0) {
      temp_gamma = -1.0;
   }
   gamma_com = Units::RadiansAngle(asin(temp_gamma));

   // Speed Control
   tas_com = weather.getAtmosphere()->CAS2TAS(guidance.m_ias_command, eqmState.enu_z);
   Units::Speed v_cas = weather.getAtmosphere()->TAS2CAS(eqmState.true_airspeed,
                                                         eqmState.enu_z);

   // Speed Error
   Units::Speed new_vel_error = tas_com - eqmState.true_airspeed;
   Units::Acceleration vel_dot_com = m_gain_velocity * new_vel_error;

   // Estimate kinetic forces for this state
   Units::Force lift, drag;
   estimateKineticForces(eqmState, lift, drag, flapConfig_new, weather);

   // Thrust to maintain speed
   // Nominal Thrust (no acceleration) at desired speed
   Units::Force thrust_nominal;
   thrust_nominal = m_ac_mass * vel_dot_com
          + drag - m_ac_mass * Units::ONE_G_ACCELERATION * sin(eqmState.gamma)
          - m_ac_mass * eqmState.true_airspeed
            * (m_dVwx_dh * cos(eqmState.psi) + m_dVwy_dh * sin(eqmState.psi))
            * sin(eqmState.gamma) * cos(eqmState.gamma);

   thrust_command = thrust_nominal;

   // Thrust Limits
   Units::Force max_thrust = Units::NewtonsForce(
         m_bada_calculator->getMaxThrust(eqmState.enu_z, flapConfig_new, "cruise"));
   Units::Force min_thrust = Units::NewtonsForce(
         m_bada_calculator->getMaxThrust(eqmState.enu_z, flapConfig_new, "descent"));

   // Check Configuration if min_thrust is Commanded
   if (thrust_command < min_thrust) {

      thrust_command = min_thrust;

      int mode_new = 0;
      m_bada_calculator->getConfigForDrag(v_cas,
                                      Units::MetersLength(eqmState.enu_z),
                                      m_alt_at_FAF,
                                      flapConfig_new,
                                      mode_new);

      flapConfig_new = mode_new;

      m_min_thrust_counter = m_min_thrust_counter + 1;

   } else {
      m_min_thrust_counter = 0.0;

      // Limit Thrust if thrust_command exceeds Max Thrust
      if (thrust_command > max_thrust) {
         thrust_command = max_thrust;
      }
   }

   // Use speed brakes, if necessary
   if (m_min_thrust_counter > 15.0 && new_vel_error < Units::KnotsSpeed(-5.0)) {
      if (flapConfig_new <= 2) {
         if (!m_is_speedbrake_on) {
            m_speedbrake_counter = 0.0;
            speed_brake_command = 0.5;
            m_is_speedbrake_on = true;
         } else {
            m_speedbrake_counter = m_speedbrake_counter + 1;
            speed_brake_command = 0.5;
         }
      } else {
         m_speedbrake_counter = 0.0;
         speed_brake_command = 0.0;
         m_is_speedbrake_on = false;
      }
   }

   // If no longer commanding Min Thrust
   if (m_is_speedbrake_on && m_speedbrake_counter <= 30.0) {
      m_speedbrake_counter = m_speedbrake_counter + 1;
      speed_brake_command = 0.5;
   } else if (m_is_speedbrake_on && m_speedbrake_counter > 30.0) {
      if (m_min_thrust_counter == 0) {
         m_speedbrake_counter = 0.0;
         speed_brake_command = 0.0;
         m_is_speedbrake_on = false;
      } else {
         m_speedbrake_counter = m_speedbrake_counter + 1;
         speed_brake_command = 0.5;
      }
   }
}

