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

#include "public/SpeedOnThrustControl.h"
#include "public/Environment.h"

using namespace aaesim::open_source;

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
   m_thrust_gain = CalculateThrustGain(); // use the parent's provided thrust gain calculation
   m_gain_velocity = Units::sqr(m_natural_frequency) / m_thrust_gain; // new velocity gain, roughly .117

   m_min_thrust_counter = 0.0;
   m_speedbrake_counter = 0.0;
   m_is_speedbrake_on = false;
}

void SpeedOnThrustControl::Initialize(std::shared_ptr<aaesim::BadaPerformanceCalculator> bada_calculator,
                                      const Units::Angle &max_bank_angle) {
   AircraftControl::Initialize(bada_calculator, max_bank_angle);
   m_min_thrust_counter = 0.0;
   m_speedbrake_counter = 0.0;
   m_is_speedbrake_on = false;
}

void SpeedOnThrustControl::DoVerticalControl(const Guidance &guidance,
                                             const EquationsOfMotionState &equations_of_motion_state,
                                             Units::Force &thrust_command,
                                             Units::Angle &gamma_command,
                                             Units::Speed &tas_command,
                                             double &speed_brake_command,
                                             aaesim::open_source::bada_utils::FlapConfiguration &new_flap_configuration,
                                             const WeatherTruth &weather) {
   const Units::Speed hdot_ref = guidance.m_vertical_speed;
   const Units::Length alt_ref = guidance.m_reference_altitude;
   const Units::Length error_alt = alt_ref - equations_of_motion_state.altitude_msl;

   double temp_gamma = -(hdot_ref + m_gain_altitude * error_alt) / equations_of_motion_state.true_airspeed; // calculate change in altitude
   // if fabs(change) > 1 set to 1 required for asin calculation
   if (temp_gamma > 1.0) {
      temp_gamma = 1.0;
   } else if (temp_gamma < -1.0) {
      temp_gamma = -1.0;
   }
   gamma_command = Units::RadiansAngle(asin(temp_gamma));

   // Speed Control
   tas_command = weather.CAS2TAS(guidance.m_ias_command, equations_of_motion_state.altitude_msl);
   Units::Speed calibrated_airspeed = weather.getAtmosphere()->TAS2CAS(equations_of_motion_state.true_airspeed,
                                                                       equations_of_motion_state.altitude_msl);

   // Speed Error
   Units::Speed error_tas = tas_command - equations_of_motion_state.true_airspeed;
   Units::Acceleration vel_dot_com = m_gain_velocity * error_tas;

   // Estimate kinetic forces for this state
   Units::Force lift, drag;
   ConfigureFlapsAndEstimateKineticForces(equations_of_motion_state, lift, drag, new_flap_configuration, weather);

   // Thrust to maintain speed
   // Nominal Thrust (no acceleration) at desired speed
   Units::Force thrust_nominal;
   thrust_nominal = m_ac_mass * vel_dot_com
          + drag - m_ac_mass * Units::ONE_G_ACCELERATION * sin(equations_of_motion_state.gamma)
          - m_ac_mass * equations_of_motion_state.true_airspeed
            * (m_dVwx_dh * cos(equations_of_motion_state.psi_enu) + m_dVwy_dh * sin(equations_of_motion_state.psi_enu))
            * sin(equations_of_motion_state.gamma) * cos(equations_of_motion_state.gamma);

   thrust_command = thrust_nominal;

   // Thrust Limits
   Units::Force max_thrust = Units::NewtonsForce(
      m_bada_calculator->GetMaxThrust(equations_of_motion_state.altitude_msl, new_flap_configuration, aaesim::open_source::bada_utils::EngineThrustMode::MAXIMUM_CRUISE, Units::ZERO_CELSIUS));
   Units::Force min_thrust = Units::NewtonsForce(
      m_bada_calculator->GetMaxThrust(equations_of_motion_state.altitude_msl, new_flap_configuration, aaesim::open_source::bada_utils::EngineThrustMode::DESCENT, Units::ZERO_CELSIUS));

   // Check Configuration if min_thrust is Commanded
   if (thrust_command < min_thrust) {

      thrust_command = min_thrust;

      aaesim::open_source::bada_utils::FlapConfiguration updated_flap_configuration = aaesim::open_source::bada_utils::FlapConfiguration::UNDEFINED;
      m_bada_calculator->GetConfigurationForIncreasedDrag(calibrated_airspeed,
                                                          Units::MetersLength(equations_of_motion_state.altitude_msl),
                                                          updated_flap_configuration);

      new_flap_configuration = updated_flap_configuration;

      m_min_thrust_counter = m_min_thrust_counter + 1;

   } else {
      m_min_thrust_counter = 0.0;

      // Limit Thrust if thrust_command exceeds Max Thrust
      if (thrust_command > max_thrust) {
         thrust_command = max_thrust;
      }
   }

   // Use speed brakes, if necessary
   if (m_min_thrust_counter > 15.0 && error_tas < Units::KnotsSpeed(-5.0)) {
      if (new_flap_configuration <= aaesim::open_source::bada_utils::FlapConfiguration::LANDING) {
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

   LOG4CPLUS_TRACE(m_logger, "altitude_error," << Units::FeetLength(error_alt) << "," <<
                             "is_level_flight," << m_is_level_flight << "," <<
                             "thrust_command," << Units::NewtonsForce(thrust_command) << "," <<
                             "dynamics_thrust," << Units::NewtonsForce(equations_of_motion_state.thrust) << "," <<
                             "max_thrust," << Units::NewtonsForce(max_thrust) << "," <<
                             "min_thrust," << Units::NewtonsForce(min_thrust) << "," <<
                             "speed_brake_command," << speed_brake_command << "," <<
                             "new_flap_configuration," << bada_utils::GetFlapConfigurationAsString(new_flap_configuration) << "," <<
                             "true_airspeed_error," << Units::KnotsSpeed(error_tas) << "," <<
                             "true_airspeed_command," << Units::KnotsSpeed(tas_command) << "," <<
                             "gamma_command," << Units::DegreesAngle(gamma_command) );
}

