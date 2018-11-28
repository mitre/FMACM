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
// Copyright 2018 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#include <cstring>
#include "framework/TestFrameworkDynamics.h"
#include "math/CustomMath.h"
#include "public/SimulationTime.h"
#include "public/AircraftCalculations.h"
#include "math/DMatrix.h"
#include "utility/CustomUnits.h"
#include <Area.h>
#include <Density.h>
#include <SignedAngle.h>
#include <public/CoreUtils.h>
#include "public/Scenario.h"

using namespace std;

const Units::Time TestFrameworkDynamics::pilot_delay_mean = Units::SecondsTime(0.0);
const Units::Time TestFrameworkDynamics::pilot_delay_std = Units::SecondsTime(0.0);


TestFrameworkDynamics::TestFrameworkDynamics() {
   m_fms = NULL;
   m_model_loaded = false;
   m_ac_type_name = "";
   m_max_bank_angle = Units::RadiansAngle(Units::DegreesAngle(30.0));

   m_max_thrust_percent = 1.0;
   m_min_thrust_percent = 1.0;

   m_alt_thresh = Units::FeetLength(500); // default altitude threshold is 500 feet
   m_speed_thresh = Units::KnotsSpeed(10);// default speed threshold of 10 Knots
   m_speed_management_type = "thrust";

   m_min_thrust_counter = 0.0;
   m_speed_brake_counter = 0.0;
   m_speed_brake_on = false;
   m_level_flight = true;

   m_mode_last = 0;

   m_weather = NULL;

   m_wind_velocity_parallel = Units::MetersPerSecondSpeed(0.0);
   m_wind_velocity_perpendicular = Units::MetersPerSecondSpeed(0.0);

   m_alt_at_faf = Units::FeetLength(-100.0);

   // Default flaps speed values

   m_state.flapConfig = 0;
   m_bada_with_calc.setFlapSpeeds("");
}

TestFrameworkDynamics::~TestFrameworkDynamics() {
}

// Aircraft update method that calculates the new aircraft state from the given command state
AircraftState TestFrameworkDynamics::Update(const AircraftState state_in,
      const Guidance guidance_in) {
   AircraftState result_state;
   Guidance pilot_delay;

   Units::Time dt0 = SimulationTime::get_simulation_time_step(); // gets simulation time step as difference in time
   Units::Time time = Units::SecondsTime(state_in.m_time) + dt0;
   SetWeatherFromTime(time);

   // add current command to the pilot delay buffer
   AddToPilotDelay(guidance_in, state_in.m_time);

   // get the aircraft guidance for the current time accounting for pilot delay
   m_prev_guidance = GetPilotDelayGuidance(m_prev_guidance, state_in.m_time);
   // TODO ? guidance_in = prev_guidance;

   if (InternalObserver::getInstance()->debugTrueWind()) {
      InternalObserver::getInstance()->setTrueWindHdrVals(state_in.m_time, state_in.m_id);
   }

   result_state = Integrate(guidance_in);

   return result_state;
}

// integrate method to integrate the command vector into the aircraft state
AircraftState TestFrameworkDynamics::Integrate(Guidance guidance_in) {
   AircraftState result_state;

   //first-order derivative of the state calculated by the EOM
   //The units are metric.
   InternalAircraftStateD dX;

   Units::SecondsTime dt = SimulationTime::get_simulation_time_step(); // gets simulation time step as difference in time

   //call EOM speed_on_thrust_control_dynamics model to calculate the change per second for the aircraft states
   if (m_speed_management_type == "thrust") {
      dX = SpeedOnThrustControlDynamics(guidance_in);
   } else if (m_speed_management_type == "pitch") {
      dX = SpeedOnPitchControlDynamics(guidance_in);
   }

   //Integrate the state:
   m_internal_aircraft_state.x += dX.dx * dt;
   m_internal_aircraft_state.y += dX.dy * dt;
   m_internal_aircraft_state.h += dX.dh * dt;
   m_internal_aircraft_state.V += dX.dV * dt;
   m_internal_aircraft_state.gamma += dX.dgamma * dt;
   m_internal_aircraft_state.psi += dX.dpsi * dt;
   m_internal_aircraft_state.T += dX.dT * dt;
   m_internal_aircraft_state.phi += dX.dphi * dt;
   m_internal_aircraft_state.speedBrake += dX.dspeedBrake * dt.value();
   m_internal_aircraft_state.flapConfig = dX.flapConfig;

   //% States
   m_state.x = Units::MetersLength(m_internal_aircraft_state.x).value();                 // east (m)
   m_state.y = Units::MetersLength(m_internal_aircraft_state.y).value();                 // north (m)
   m_state.h = Units::MetersLength(m_internal_aircraft_state.h).value();                 // altitude (m)
   m_state.V = Units::MetersPerSecondSpeed(m_internal_aircraft_state.V).value();         // true airspeed (m/s)
   m_state.gamma = Units::RadiansAngle(
         m_internal_aircraft_state.gamma).value();         // flight-path angle (rad): NOTE: for gamma, heading down is positive; heading up is negative
   m_state.psi = Units::RadiansAngle(
         m_internal_aircraft_state.psi).value();         // heading angle measured from east counter-clockwise (rad); NOTE: use mathematical angle, not aviation heading
   m_state.T = Units::NewtonsForce(m_internal_aircraft_state.T).value();            // thrust (N)
   m_state.phi = Units::RadiansAngle(m_internal_aircraft_state.phi).value();         // roll angle (rad)
   m_state.speed_brake = m_internal_aircraft_state.speedBrake;      // speed brake (% of deployment)-currently unused
   m_state.flapConfig = int(m_internal_aircraft_state.flapConfig + 0.1);   // current flap configuration

   // Use xdot and ydot from Dynamics to ensure winds are used
   Units::Speed xdot = dX.dx;  //ground speed (m/s)
   Units::Speed ydot = dX.dy;  //ground speed (m/s)

   //gwang 2013-10
   m_state.xd = Units::MetersPerSecondSpeed(xdot).value(); //ground speed x component (m/s)
   m_state.yd = Units::MetersPerSecondSpeed(ydot).value(); //ground speed y component (m/s)
   //end gwang

   // LAW: Check Thrust Limits and Limit Appropriately
   Units::Speed v_cas = m_weather_truth.getAtmosphere()->TAS2CAS(Units::MetersPerSecondSpeed(m_state.V),
         Units::MetersLength(
               m_state.h)); // current indicated airspeed in meters per second

   double cd0, cd2;
   int mode;
   double gear;
   Units::MetersLength state_h(m_state.h);
   m_bada_with_calc.getConfig(v_cas, state_h, m_alt_at_faf, m_state.flapConfig, cd0, cd2, gear, mode);
   Units::Force maxThrust = Units::NewtonsForce(m_bada_with_calc.getMaxThrust(state_h, mode, "cruise"));
   Units::Force minThrust = Units::NewtonsForce(m_bada_with_calc.getMaxThrust(state_h, mode, "descent"));

   if (m_state.T > Units::NewtonsForce(maxThrust).value()) {
      m_state.T = Units::NewtonsForce(maxThrust).value();
   } else if (m_state.T < Units::NewtonsForce(minThrust).value()) {
      m_state.T = Units::NewtonsForce(minThrust).value();
   }

   if (m_state.speed_brake > 0.5) {
      m_state.speed_brake = 0.5;
   } else if (m_state.speed_brake < 0.0) {
      m_state.speed_brake = 0.0;
   }

   // assign return values.
         result_state.m_x = m_state.x / FEET_TO_METERS; //(ft)
         result_state.m_y = m_state.y / FEET_TO_METERS; //(ft)
         result_state.m_z = m_state.h / FEET_TO_METERS; //(ft)
         result_state.SetPsi(m_state.psi); //(radian)
         result_state.m_xd = Units::FeetPerSecondSpeed(xdot).value(); //(ft/s)
         result_state.m_yd = Units::FeetPerSecondSpeed(ydot).value(); //(ft/s)
         result_state.m_zd = -m_state.V / FEET_TO_METERS * sin(m_state.gamma); //(ft/s) Note: for gamma, heading down is positive
         result_state.m_Vwx = Units::MetersPerSecondSpeed(m_weather->Vwx).value();
         result_state.m_Vwy = Units::MetersPerSecondSpeed(m_weather->Vwy).value();
         result_state.m_Vw_para = Units::MetersPerSecondSpeed(m_wind_velocity_parallel).value();
         result_state.m_Vw_perp = Units::MetersPerSecondSpeed(m_wind_velocity_perpendicular).value();

         return result_state;
}

// EOM speed_on_thrust_control_dynamics call to calculate the dX values

TestFrameworkDynamics::InternalAircraftStateD
TestFrameworkDynamics::SpeedOnThrustControlDynamics(Guidance guidance_in) {
   InternalAircraftStateD dX; //units: metric

   //	int m = Wind::GetFlightLevelLowerBound() + 1;
   //	int n = Wind::GetFlightLevelUpperBound() + 1;
   //	DMatrix wind_x(m, n, 1, 2);
   //	DMatrix wind_y(m, n, 1, 2);

   double lat = 0, lon = 0;   // dummy parameters

   //	Wind::interpolate_true_wind(false, lat, lon, state.x/FT_M, state.y/FT_M, state.h/FT_M, wind_x, wind_y);
   //
   //	if (InternalObserver::getInstance()->debugTrueWind()) {
   //		InternalObserver::getInstance()->writeTrueWind(trueWindCsvString(wind_x,wind_y));
   //	}


   // Control Gains used in tracking desired states and commanded inputs
   Units::InvertedLength k_xtrk = Units::PerMeterInvertedLength(5e-4);  // meters^-1
   double k_trk = 3;      // unitless
   Units::Frequency k_alt = Units::HertzFrequency(0.20); // 1/sec
   Units::Frequency k_gamma = Units::HertzFrequency(0.20); // 1/sec
   Units::Frequency k_phi = Units::HertzFrequency(0.40);   // 1/sec
   //Units::Frequency k_t = 0.70;     // 1/sec

   // new gain control values 2/20/2013
   double zeta = 0.88;
   Units::Frequency wn = Units::HertzFrequency(0.20);
   Units::Frequency k_t = 2 * zeta * wn; // new thrust gain, roughly .352
   Units::Frequency k_v = Units::sqr(wn) / k_t; // new velocity gain, roughly .117
   double k_i = 0.0;//0.005; // velocity error gain
   double k_speedBrake = 0.20;

   //% States:
   Units::Length x = m_internal_aircraft_state.x;           // aircraft position east coordinate (m)
   Units::Length y = m_internal_aircraft_state.y;           // aircraft position north coordinate (m)
   Units::Length h = m_internal_aircraft_state.h;           // aircraft altitude (m)
   Units::Speed V = m_internal_aircraft_state.V;           // true airspeed (m/s)
   Units::Angle gamma = m_internal_aircraft_state.gamma;       // flight-path angle (rad)
   Units::Angle psi = m_internal_aircraft_state.psi;         // heading angle measured from east counter-clockwise (rad)
   Units::Force T = m_internal_aircraft_state.T;           // thrust (N)
   Units::Angle phi = m_internal_aircraft_state.phi;         // roll angle (rad)
   double speedBrake = m_internal_aircraft_state.speedBrake;  // speed brake (% of deployment)
   int flapConfig = (int) (m_internal_aircraft_state.flapConfig + 0.1); // flap configuration

   double speedBrakeCom = 0.0;

   // winds and gradients are already set in weather

   // Commanded Track Angle
   Units::Angle trk = Units::RadiansAngle(guidance_in.psi); // GUIDANCE

   m_wind_velocity_parallel = m_weather->Vwx * cos(trk) + m_weather->Vwy * sin(trk);
   m_wind_velocity_perpendicular = -m_weather->Vwx * sin(trk) + m_weather->Vwy * cos(trk);

   Units::Speed W = sqrt(Units::sqr(m_weather->Vwx) + Units::sqr(m_weather->Vwy));
   Units::Speed gs = sqrt(Units::sqr(V * cos(gamma)) - Units::sqr(m_wind_velocity_perpendicular)) + m_wind_velocity_parallel;

   double temp = (Units::sqr(V * cos(gamma)) + Units::sqr(gs) - Units::sqr(W)) / (V * 2 * cos(gamma) * gs);

   // Limit temp so acos function doesn't give undefined value.

   if (temp > 1.0) {
      temp = 1.0;
   } else if (temp < -1.0) {
      temp = -1.0;
   }

   Units::Angle beta =
         Units::RadiansAngle(acos(temp)) * -1.0 *
               CoreUtils::SignOfValue(Units::MetersPerSecondSpeed(m_wind_velocity_perpendicular).value());

   //Lateral Control

   // Convert track guidance to heading using winds (beta is the Wind Correction Angle)
   Units::Angle headingCom = trk + beta;

   // Error in heading angle
   Units::SignedAngle e_trk = headingCom - psi;
   e_trk.normalize();

   // Along-path distance and Cross-track Error
   Units::Length e_xtrk = Units::MetersLength(0.0);
   double dynamic_cross = 1.0;

   // check if guidance has has cross track error and use it if so
   if (guidance_in.use_cross_track) {
      e_xtrk = Units::MetersLength(guidance_in.cross_track);
   }

   // Calculate commanded roll angle
   Units::Angle phi_com = -k_xtrk * e_xtrk * Units::ONE_RADIAN_ANGLE - k_trk * e_trk; // CONTROL
   double unlimited_phi_com = Units::RadiansAngle(phi_com).value();

   // Limit the commanded roll angle
   double sign_phi_com = CoreUtils::SignOfValue(unlimited_phi_com);
   if (phi_com * sign_phi_com > m_max_bank_angle) {
      phi_com = m_max_bank_angle * sign_phi_com;
   }

   InternalObserver::getInstance()->cross_output(Units::MetersLength(x).value(),
         Units::MetersLength(y).value(),
         dynamic_cross, guidance_in.cross_track, guidance_in.psi,
         unlimited_phi_com,
         Units::RadiansAngle(phi_com).value());

   //% Vertical Control
   Units::Speed hdot_ref = Units::FeetPerSecondSpeed(guidance_in.altitude_rate);

   //double 	alt_ref = 10000*FT_M;   // GUIDANCE
   Units::Length alt_ref = Units::FeetLength(guidance_in.reference_altitude);   // GUIDANCE

   Units::Length e_alt = alt_ref - h;

   double temp_gamma = -(hdot_ref + k_alt * e_alt) / V; // calculate change in altitude
   if (temp_gamma > 1.0) {
      temp_gamma = 1.0;
   } else if (temp_gamma < -1.0) {
      temp_gamma = -1.0;
   }
   Units::Angle gamma_com = Units::RadiansAngle(asin(temp_gamma)); // CONTROL new 2/20/2013

   //% Speed Control

   Units::Speed tas_com = m_weather_truth.getAtmosphere()->CAS2TAS(
         Units::FeetPerSecondSpeed(guidance_in.m_im_speed_command_ias),
         h);   // GUIDANCE true airspeed
   Units::Speed v_cas = m_weather_truth.getAtmosphere()->TAS2CAS(V, h); // current indicated airspeed in meters per second

   //% Speed Error
   Units::Speed new_vel_error = tas_com - V;
   Units::Acceleration vel_dot_com = k_v * new_vel_error;

   // Should we limit commanded acceleration?

   //% Thrust to maintain speed
   // Get temp, density, and pressure
   Units::Density rho;
   Units::Pressure P_tmp;
   m_weather_truth.getAtmosphere()->AirDensity(h, rho, P_tmp);
   // Don't bother converting P_tmp from kg/m^2 because we don't need it.

   // Get m_bada_calculator Configuration
   double cd0, cd2;
   int flapConfig_new;
   double gear;
   m_bada_with_calc.getConfig(v_cas, h, m_alt_at_faf, flapConfig, cd0, cd2, gear, flapConfig_new);

   Units::Mass ac_mass = Units::KilogramsMass(m_bada_with_calc.mAircraftMass);
   Units::Area wing_area = Units::MetersArea(m_bada_with_calc.aerodynamics.S);

   // Lift and Drag Coefficients
   //double 	cL = (2*m_bada_calculator.mass*G)/(rho*V^2*m_bada_calculator.S);
   // units of cL = kg * m / s^2 / (kg / m^3 * m^2 / s^2 * m^2) = unitless
   double cL = (2. * ac_mass * Units::ONE_G_ACCELERATION) / (rho * Units::sqr(V) * wing_area * cos(phi));
   //cD = cd0 + gear + cd2*cL^2;
   double cD = cd0 + gear + cd2 * pow(cL, 2);

   if (speedBrake != 0.0) {
      cD = (1.0 + 0.6 * speedBrake) * cD;
   }

   // Drag
   Units::Force D = 1. / 2. * rho * cD * Units::sqr(V) * wing_area;


   // Lift
   Units::Force L = 1. / 2. * rho * cL * Units::sqr(V) * wing_area;

   // Nominal Thrust (no acceleration) at desired speed
   Units::Force Tnom;
   Tnom = ac_mass * vel_dot_com
         + D - ac_mass * Units::ONE_G_ACCELERATION * sin(gamma)
   - ac_mass * V
   * (m_weather->dVwx_dh * cos(psi) + m_weather->dVwy_dh * sin(psi))
   * sin(gamma) * cos(gamma);


   Units::Force T_com = Tnom;

   // Thrust Limits
   Units::Force maxThrust = Units::NewtonsForce(
         m_bada_with_calc.getMaxThrust(h, flapConfig_new, "cruise"));
   Units::Force minThrust = Units::NewtonsForce(
         m_bada_with_calc.getMaxThrust(h, flapConfig_new, "descent"));
   speedBrakeCom = 0.0;

   // Check Configuration if minThrust is Commanded
   if (T_com < minThrust) {

      T_com = minThrust;

      int mode_new = 0;
      m_bada_with_calc.getConfigForDrag(v_cas, h, m_alt_at_faf, flapConfig_new, mode_new);

      flapConfig_new = mode_new;

      m_min_thrust_counter = m_min_thrust_counter + 1;

   } else {
      m_min_thrust_counter = 0.0;

      // Limit Thrust if T_com exceeds Max Thrust
      if (T_com > maxThrust) {
         T_com = maxThrust;
      }

   }

   // Use speed brakes, if necessary
   if (m_min_thrust_counter > 15.0 && new_vel_error < Units::KnotsSpeed(-5.0)) {
      if (flapConfig_new <= 2) {
         if (!m_speed_brake_on) {
            m_speed_brake_counter = 0.0;
            speedBrakeCom = 0.5;
            m_speed_brake_on = true;
         } else {
            m_speed_brake_counter = m_speed_brake_counter + 1;
            speedBrakeCom = 0.5;
         }
      } else {
         m_speed_brake_counter = 0.0;
         speedBrakeCom = 0.0;
         m_speed_brake_on = false;
      }
   }

   // If no longer commanding Min Thrust
   if (m_speed_brake_on && m_speed_brake_counter <= 30.0) {
      m_speed_brake_counter = m_speed_brake_counter + 1;
      speedBrakeCom = 0.5;
   } else if (m_speed_brake_on && m_speed_brake_counter > 30.0) {
      if (m_min_thrust_counter == 0) {
         m_speed_brake_counter = 0.0;
         speedBrakeCom = 0.0;
         m_speed_brake_on = false;
      } else {
         m_speed_brake_counter = m_speed_brake_counter + 1;
         speedBrakeCom = 0.5;
      }
   }

   // calculate the first-order derivative of the state vector:
   dX.dx = Units::MetersPerSecondSpeed(V * cos(gamma) * cos(psi) + m_weather->Vwx);
   dX.dy = Units::MetersPerSecondSpeed(V * cos(gamma) * sin(psi) + m_weather->Vwy);
   dX.dh = Units::MetersPerSecondSpeed(-V * sin(gamma));
   dX.dV = (T - D) / ac_mass + Units::ONE_G_ACCELERATION * sin(gamma)
   + V * (m_weather->dVwx_dh * cos(psi) + m_weather->dVwy_dh * sin(psi))
   * sin(gamma) * cos(gamma);
   dX.dgamma = k_gamma * (gamma_com - gamma) -
         (m_weather->dVwx_dh * cos(psi) + m_weather->dVwy_dh * sin(psi)) * pow(sin(gamma), 2) *
         Units::ONE_RADIAN_ANGLE;
   dX.dpsi = (-L * sin(phi) / (ac_mass * V * cos(gamma)) -
         (m_weather->dVwx_dh * sin(psi) - m_weather->dVwy_dh * cos(psi)) * tan(gamma)) * Units::ONE_RADIAN_ANGLE;
   dX.dT = k_t * (T_com - T);
   dX.dphi = k_phi * (phi_com - phi);
   dX.dspeedBrake = k_speedBrake * (speedBrakeCom - speedBrake);
   dX.flapConfig = flapConfig_new;

   return dX;
} // speed_on_thrust_control_dynamics


// sets the Dynamics FMS
void TestFrameworkDynamics::SetFms(TestFrameworkFMS *fms_in) {
   m_fms = fms_in;
}

// method to check if the model loaded properly
bool TestFrameworkDynamics::IsLoaded() {
   return m_model_loaded;
}

void TestFrameworkDynamics::Init(double mass_percentile,
      Units::Length altitude_at_final_approach_fix_in,
      Units::Length initial_altitude,
      Units::Speed initial_ias,
      double initial_mach,
      double start_time) {
   SetWeatherFromTime(Units::SecondsTime(start_time));

   m_alt_at_faf = Units::MetersLength(altitude_at_final_approach_fix_in);

   if (m_alt_at_faf < Units::MetersLength(0.0)) {
      std::cout << "WARNING:FAF altitude " << Units::FeetLength(m_alt_at_faf) << " < 0." << std::endl
            << "Probably was not initialized in precalculated trajectory." << std::endl;
   }

   m_bada_with_calc.getAircraftParameters(m_ac_type_name, mass_percentile);
   m_bada_with_calc.setFlapSpeeds(m_ac_type_name);

   double ias_;

   double xdot, ydot;

   //When initializing the state, initial values should be assigned to the X vector, because it is the one that
   //is used by the EOM function to calculate its first-order derivative.


   // Defining aircraft states
   // ----------------------------------------------------------------------------
   m_state.x = m_fms->m_waypoint_x[m_fms->m_next_waypoint_ix - 1] * FEET_TO_METERS;
   m_state.y = m_fms->m_waypoint_y[m_fms->m_next_waypoint_ix - 1] * FEET_TO_METERS;
   m_state.h = Units::MetersLength(initial_altitude).value();

   double lat, lon;
   lat = 0;
   lon = 0;

   m_state.psi = m_fms->GetPsi(m_fms->m_next_waypoint_ix);
   m_state.gamma = 0;
   m_state.phi = 0;


   // Determine whether aircraft is flying IAS or MACH

   double ias_at_waypoint = Units::FeetPerSecondSpeed(initial_ias).value(); //FPS
   double altitude = m_state.h / FEET_TO_METERS;

   if (initial_mach != 0.0) //may fly MACH
   {
      //may need to do mach-cas transition:
      double tas_from_ias =
            Units::FeetPerSecondSpeed(m_weather_truth.getAtmosphere()->CAS2TAS(
                  Units::FeetPerSecondSpeed(ias_at_waypoint),
                  Units::FeetLength(altitude))).value();

      double tas_from_mach = MachToTas(initial_mach, altitude); //FPS
      if (tas_from_ias <= tas_from_mach) //fly ias_at_waypoint
      {
         ias_ = ias_at_waypoint;
      } else //fly MACH
      {
         ias_ = MachToCas_MITRE(initial_mach, altitude);
      }
   } else //fly ias_at_waypoint
   {
      ias_ = ias_at_waypoint;
   }

   // Convert IAS to TAS

   m_state.V = Units::MetersPerSecondSpeed(m_weather_truth.getAtmosphere()->CAS2TAS(
         Units::FeetPerSecondSpeed(ias_),
         Units::MetersLength(m_state.h))).value();
   // Determine Groundspeed
   xdot = m_state.V * cos(m_state.psi) * cos(m_state.gamma)
                      + Units::MetersPerSecondSpeed(m_weather->Vwx).value(); // mps
   ydot = m_state.V * sin(m_state.psi) * cos(m_state.gamma)
                      + Units::MetersPerSecondSpeed(m_weather->Vwy).value(); // mps

   m_state.xd = xdot; // mps
   m_state.yd = ydot; // mps


   //Actually initialize the state X:
   //% States
   m_internal_aircraft_state.x = Units::MetersLength(m_state.x); // east (m)
   m_internal_aircraft_state.y = Units::MetersLength(m_state.y); // north (m)
   m_internal_aircraft_state.h = Units::MetersLength(m_state.h); // altitude (m)
   m_internal_aircraft_state.V = Units::MetersPerSecondSpeed(m_state.V); // true airspeed (m/s)
   m_internal_aircraft_state.gamma = Units::RadiansAngle(m_state.gamma);    // flight-path angle (rad); NOTE: for gamma, heading down is positve
   m_internal_aircraft_state.psi = Units::RadiansAngle(m_state.psi);      // heading angle measured from east counter-clockwise (rad)
   m_internal_aircraft_state.T = Units::NewtonsForce(0.0);            // thrust (N)
   m_internal_aircraft_state.phi = Units::RadiansAngle(m_state.phi);      // roll angle (rad)
   m_internal_aircraft_state.speedBrake = 0.0;            // speed brake (% of deployment)
   m_internal_aircraft_state.flapConfig = 0;            // flap config

   //%Calculate initial Aircraft Thrust
   // Get temp, density, and pressure
   Units::MetersLength state_h(m_state.h);
   Units::Density rho;
   Units::Pressure P_tmp;
   m_weather_truth.getAtmosphere()->AirDensity(state_h, rho, P_tmp);
   // Don't bother converting P_tmp from kg/m^2 because we don't need it.

   // Get m_bada_calculator Configuration
   double cd0, cd2;
   int mode;
   double gear;
   m_bada_with_calc.getConfig(m_weather_truth.getAtmosphere()->TAS2CAS(Units::MetersPerSecondSpeed(m_state.V), state_h),
         state_h,
         m_alt_at_faf, 0, cd0, cd2, gear, mode);
   Units::Mass ac_mass = Units::KilogramsMass(m_bada_with_calc.mAircraftMass);
   Units::Area wing_area = Units::MetersArea(m_bada_with_calc.aerodynamics.S);

   // Set values for flaps speed.

   m_state.flapConfig = mode;

   // Set lift and Drag Coefficients

   double cL = (2. * ac_mass * Units::ONE_G_ACCELERATION) / (rho * Units::sqr(m_internal_aircraft_state.V) * wing_area);
   double cD = cd0 + gear + cd2 * pow(cL, 2);
   Units::Force D = 1. / 2. * rho * cD * Units::sqr(m_internal_aircraft_state.V) * wing_area; // Drag
   Units::Force L = 1. / 2. * rho * cL * Units::sqr(m_internal_aircraft_state.V) * wing_area; // Lift
   // Nominal Thrust (no acceleration) at desired speed
   Units::Force Tnom = /*ac_mass*vel_dot_com + VELOCITY ERROR IS 0*/D - ac_mass * Units::ONE_G_ACCELERATION * sin(asin(
         0.0/*gamma 0.0*/));// - ac_mass*V*(dVwx_dh*cos(psi) + dVwy_dh*sin(psi))*sin(gamma)*cos(gamma); REMOVED WIND
   // new Thrust commands from model speed change MATLAB code 2/25
   Units::Force maxThrust = Units::NewtonsForce(m_bada_with_calc.getMaxThrust(state_h));
   Units::Force minThrust = Units::NewtonsForce(m_bada_with_calc.getMaxThrust(state_h, mode, "descent"));
   if (Tnom > maxThrust * m_max_thrust_percent) {
      Tnom = maxThrust * m_max_thrust_percent;
   } else if (Tnom < minThrust * m_min_thrust_percent) {
      Tnom = minThrust * m_min_thrust_percent;
   }

   m_internal_aircraft_state.T = Tnom; // sets the initial Thrust

   // initialize the previous Guidance to have a default until the first command is processed from pilot delay
   m_prev_guidance.m_im_speed_command_ias = ias_;
   m_prev_guidance.psi = m_state.psi;
   m_prev_guidance.reference_altitude = m_state.h / FEET_TO_METERS;
   m_prev_guidance.altitude_rate = 0;

   // True wind factors (m/s)
   m_wind_velocity_parallel = Units::MetersPerSecondSpeed(0.0);
   m_wind_velocity_perpendicular = Units::MetersPerSecondSpeed(0.0);

}


bool TestFrameworkDynamics::load(DecodedStream *input) {
   bool results;
   string env_csv_file = "";

   set_stream(input);

   // register the aircraft bada type
   register_var("ac_type", &m_ac_type_name, true);
   register_var("speed_management_type", &m_speed_management_type, true);
   register_var("env_csv_file", &env_csv_file, false);

   //do the actual reading:
   results = complete();

   if (m_ac_type_name.size() != 4) {
      cout << "ac_type was not correct. Found: " << m_ac_type_name << endl;
      exit(-1);
   }
   if (m_speed_management_type != "thrust" && m_speed_management_type != "pitch") {
      cout << "speed_management_type was not correct: " << m_speed_management_type << endl;
      exit(-1);
   }

   m_model_loaded = results;
   LoadEnvFile(env_csv_file);

   return results;
}

void TestFrameworkDynamics::LoadEnvFile(string env_csv_file) {
   m_weather_by_time.clear();
   m_weather_by_distance_to_go.clear();

   if (env_csv_file == "") {
      cout << "No env_csv_file specified; using default weather." << endl;
      // parameter missing, use default
      m_weather = new Weather();
      m_weather->Vwx = Units::MetersPerSecondSpeed(0);
      m_weather->Vwy = Units::MetersPerSecondSpeed(0);
      m_weather->dVwx_dh = Units::HertzFrequency(0);
      m_weather->dVwy_dh = Units::HertzFrequency(0);
      m_weather->temperature = Units::CelsiusTemperature(25);
      m_weather_by_time[Units::SecondsTime(0)] = m_weather;
      m_weather_by_distance_to_go[Units::MetersLength(0)] = m_weather;
      return;
   }

   FILE *env = fopen(env_csv_file.c_str(), "r");
   if (!env) {
      cout << "Could not open env_csv_file: " << env_csv_file << endl;
      exit(1);
   }

   char line[100];
   fgets(line, sizeof(line), env);   // skip the header line
   while (!feof(env)) {
      double time, distToGo, Vwx, Vwy, dVwx_dh, dVwy_dh, temperature;
      fgets(line, sizeof(line), env);
      if (strlen(line) > 6) {
         sscanf(line, "%lf,%lf,%lf,%lf,%lf,%lf,%lf",
               &time, &distToGo, &Vwx, &Vwy,
               &dVwx_dh, &dVwy_dh, &temperature);
         m_weather = new Weather();
         m_weather->Vwx = Units::MetersPerSecondSpeed(Vwx);
         m_weather->Vwy = Units::MetersPerSecondSpeed(Vwy);
         m_weather->dVwx_dh = Units::HertzFrequency(dVwx_dh);
         m_weather->dVwy_dh = Units::HertzFrequency(dVwy_dh);
         m_weather->temperature = Units::KelvinTemperature(temperature);
         m_weather_by_time[Units::SecondsTime(time)] = m_weather;
         m_weather_by_distance_to_go[Units::MetersLength(distToGo)] = m_weather;
      }
   }
   m_weather = NULL;
   fclose(env);
}

void TestFrameworkDynamics::SetWeatherFromTime(Units::Time time) {
   std::map<Units::Time, Weather *>::iterator it = m_weather_by_time.lower_bound(time);
   Units::Time t1;
   if (it == m_weather_by_time.end()) {
      std::map<Units::Time, Weather *>::reverse_iterator rit = m_weather_by_time.rbegin();
      t1 = rit->first;
   } else {
      t1 = it->first;
   }
   m_weather = m_weather_by_time[t1];
}

TestFrameworkDynamics::Weather TestFrameworkDynamics::GetWeatherFromTime(double time) {
   Units::Time t0 = Units::SecondsTime(time);
   std::map<Units::Time, Weather *>::iterator it = m_weather_by_time.lower_bound(t0);
   Units::Time t1;
   if (it == m_weather_by_time.end()) {
      std::map<Units::Time, Weather *>::reverse_iterator rit = m_weather_by_time.rbegin();
      t1 = rit->first;
   } else {
      t1 = it->first;
   }
   Weather w = *m_weather_by_time[t1];   // copy the object
   return w;
}

// helper method to add a new guidance command to the pilot delay buffer
void TestFrameworkDynamics::AddToPilotDelay(Guidance guidance_in,
      double time) {
   int delayed_time;

#ifdef _LINUX_
   delayed_time = (int) (round(time + Units::SecondsTime(
         Scenario::m_rand.RayleighSample(
               TestFrameworkDynamics::pilot_delay_mean,
               TestFrameworkDynamics::pilot_delay_std)).value()) + 0.1);
#else
   delayed_time = roundToInt(time + Units::SecondsTime(
         Scenario::mRand.rayleighSample(
               TestFrameworkDynamics::pilot_delay_mean,
               TestFrameworkDynamics::pilot_delay_std)).value());
#endif

   // if the delay time is less than the current time, set back to current time
   if (delayed_time < time) {
      delayed_time = (int) (time + 0.1);
   }

   m_pilot_delay_buffer.insert(pair<int, Guidance>(delayed_time, guidance_in));
}

// helper method to get the current command from the delay buffer
Guidance TestFrameworkDynamics::GetPilotDelayGuidance(Guidance prev_guidance,
      double time) {
   Guidance result = prev_guidance;
   int current_key = (int) time; // gets the key time for the current time

   map<int, Guidance>::iterator guide_it; // iterator to get the current Guidance

   guide_it = m_pilot_delay_buffer.find(current_key);

   if (guide_it != m_pilot_delay_buffer.end()) {
      result = guide_it->second;
   }

   return result;
}

// EOM new speed_on_pitch_control_dynamics speed_on_thrust_control_dynamics model
TestFrameworkDynamics::InternalAircraftStateD
TestFrameworkDynamics::SpeedOnPitchControlDynamics(Guidance guidance_in) {

   InternalAircraftStateD dX;

   //	int m = Wind::GetFlightLevelLowerBound() + 1;
   //	int n = Wind::GetFlightLevelUpperBound() + 1;
   //	DMatrix wind_x(m, n, 1, 2);
   //	DMatrix wind_y(m, n, 1, 2);

   //	double lat = 0, lon = 0;	// dummy parameters

   //	Wind::interpolate_true_wind(false, lat, lon, state.x/FT_M, state.y/FT_M, state.h/FT_M, wind_x, wind_y);

   //	if (InternalObserver::getInstance()->debugTrueWind()) {
   //		InternalObserver::getInstance()->writeTrueWind(trueWindCsvString(wind_x,wind_y));
   //	}

   // Control Gains used in tracking desired states and commanded inputs
   Units::InvertedLength k_xtrk = Units::PerMeterInvertedLength(5e-4);  // meters^-1
   double k_trk = 3;      // unitless
   Units::Frequency k_alt = Units::HertzFrequency(0.20); // 1/sec
   Units::Frequency k_gamma = Units::HertzFrequency(0.40); // 1/sec
   Units::Frequency k_phi = Units::HertzFrequency(0.40);   // 1/sec
   //Units::Frequency k_t = Units::HertzFrequency(0.390);     // 1/sec

   // new gain control values 2/20/2013
   double zeta = 0.88;
   Units::Frequency wn = Units::HertzFrequency(0.20);
   Units::Frequency k_t = 2 * zeta * wn; // new thrust gain, roughly .352
   Units::Frequency k_v = Units::sqr(wn) / k_t; // new velocity gain, roughly .117
   double k_i = 0.0;//0.005; // velocity error gain
   double k_speedBrake = 0.10;

   // read in the current aircraft States
   Units::Length x = m_internal_aircraft_state.x;           // east (m)
   Units::Length y = m_internal_aircraft_state.y;           // north (m)
   Units::Length h = m_internal_aircraft_state.h;           // altitude (m)
   Units::Speed V = m_internal_aircraft_state.V;           // true airspeed (m/s)
   Units::Angle gamma = m_internal_aircraft_state.gamma;       // flight-path angle (rad)
   Units::Angle psi = m_internal_aircraft_state.psi;         // heading angle measured from east counter-clockwise (rad)
   Units::Force T = m_internal_aircraft_state.T;           // thrust (N)
   Units::Angle phi = m_internal_aircraft_state.phi;         // roll angle (rad)
   double speedBrake = m_internal_aircraft_state.speedBrake; // speed brake (% of deployment)-currently unused
   int flapConfig = (int) (m_internal_aircraft_state.flapConfig + 0.1); // flap configuration

   double speedBrakeCom = 0.0;

   // Wind Segment
   // winds are already set as part of the weather object

   Units::Angle trk = Units::RadiansAngle(guidance_in.psi); // GUIDANCE

   m_wind_velocity_parallel = m_weather->Vwx * cos(trk) + m_weather->Vwy * sin(trk);
   m_wind_velocity_perpendicular = -m_weather->Vwx * sin(trk) + m_weather->Vwy * cos(trk);

   Units::Speed W = sqrt(Units::sqr(m_weather->Vwx) + Units::sqr(m_weather->Vwy));
   Units::Speed gs = sqrt(Units::sqr(V * cos(gamma)) - Units::sqr(m_wind_velocity_perpendicular)) + m_wind_velocity_parallel;

   double temp = (Units::sqr(V * cos(gamma)) + Units::sqr(gs) - Units::sqr(W)) / (V * 2 * cos(gamma) * gs);

   // Limit temp so the acos function doesn't give an undefined value.

   if (temp > 1.0) {
      temp = 1.0;
   } else if (temp < -1.0) {
      temp = -1.0;
   }

   Units::Angle beta =
         Units::RadiansAngle(acos(temp)) * -1.0 *
               CoreUtils::SignOfValue(Units::MetersPerSecondSpeed(m_wind_velocity_perpendicular).value());

   //Lateral Control

   // Convert track guidance to heading using winds
   Units::Angle headingCom = trk + beta;

   //% Lateral Control

   // Error in track angle limited to +-PI
   Units::SignedAngle e_trk = headingCom - psi;
   e_trk.normalize();

   Units::Length e_xtrk = Units::MetersLength(0);
   double dynamic_cross = 1.0;

   // check if guidance has has cross track error and use it if so
   if (guidance_in.use_cross_track) {
      e_xtrk = Units::MetersLength(guidance_in.cross_track);
   }

   // Calculate commanded roll angle
   Units::Angle phi_com = -k_xtrk * e_xtrk * Units::ONE_RADIAN_ANGLE - k_trk * e_trk; // CONTROL
   double unlimited_phi_com = Units::RadiansAngle(phi_com).value();

   // Limit the commanded roll angle
   double sign_phi_com = CoreUtils::SignOfValue(unlimited_phi_com);
   if (phi_com * sign_phi_com > m_max_bank_angle) {
      phi_com = m_max_bank_angle * sign_phi_com;
   }

   Units::Speed v_cas = m_weather_truth.getAtmosphere()->TAS2CAS(Units::MetersPerSecondSpeed(V), Units::MetersLength(
         h)); // current indicated airspeed in meters per second

   //% Thrust to maintain speed
   // Get temp, density, and pressure
   Units::Density rho;
   Units::Pressure P_tmp;
   m_weather_truth.getAtmosphere()->AirDensity(h, rho, P_tmp);
   // Don't bother converting P_tmp from kg/m^2 because we don't need it.

   // Get m_bada_calculator Configuration
   double cd0, cd2;
   int flapConfig_new;
   double gear;
   m_bada_with_calc.getConfig(v_cas, h, m_alt_at_faf, flapConfig, cd0, cd2, gear, flapConfig_new);

   Units::Mass ac_mass = Units::KilogramsMass(m_bada_with_calc.mAircraftMass);
   Units::Area wing_area = Units::MetersArea(m_bada_with_calc.aerodynamics.S);

   // Lift and Drag Calculations
   double cL = (2. * ac_mass * Units::ONE_G_ACCELERATION) / (rho * Units::sqr(V) * wing_area * cos(phi));
   double cD = cd0 + gear + cd2 * pow(cL, 2);

   if (speedBrake != 0.0) {
      cD = (1.0 + 0.6 * speedBrake) * cD;
   }

   Units::Force D = 1. / 2. * rho * cD * Units::sqr(V) * wing_area;
   Units::Force L = 1. / 2. * rho * cL * Units::sqr(V) * wing_area;

   Units::Speed ias_com = Units::FeetPerSecondSpeed(guidance_in.m_im_speed_command_ias);
   Units::Speed tas_com = m_weather_truth.getAtmosphere()->CAS2TAS(ias_com, h);

   Units::Force maxThrust = Units::NewtonsForce(
         m_bada_with_calc.getMaxThrust(h, flapConfig_new, "cruise"));
   Units::Force minThrust = Units::NewtonsForce(
         m_bada_with_calc.getMaxThrust(h, flapConfig_new, "descent"));

   //Speed Management Method check
   Units::Length alt_ref = Units::FeetLength(guidance_in.reference_altitude);
   Units::Length e_alt = h - alt_ref;
   Units::Speed h_dot = Units::FeetPerSecondSpeed(guidance_in.altitude_rate);
   Units::Force T_com = Units::NewtonsForce(0.0);
   Units::Angle gamma_com = Units::RadiansAngle(0.0);

   // LEVEL FLIGHT - manage speed with thrust and altitude with pitch
   if (m_level_flight) {
      Units::Speed ev = tas_com - V;
      Units::Acceleration vDotCom = k_v * ev;

      // calculate the Thrust Command
      T_com = ac_mass * vDotCom
            + D
            - ac_mass * Units::ONE_G_ACCELERATION * sin(gamma)
      - ac_mass * V * (m_weather->dVwx_dh * cos(psi) +
            m_weather->dVwy_dh * sin(psi))
            * sin(gamma) * cos(gamma);

      // command a level altitude
      gamma_com = Units::RadiansAngle(0.0);

      if (Units::MetersLength(h).value() - m_fms->m_constraints[m_fms->m_next_waypoint_ix].constraint_altLow > 200.0 * FEET_TO_METERS &&
            Units::MetersPerSecondSpeed(h_dot).value() != 0.0) {
         m_level_flight = false;
         T_com = minThrust;
      }

   }
   // DESCENDING FLIGHT - manage altitude with thrust, speed with pitch
   else {
      double esf = AircraftCalculations::ESFconstantCAS(V, h, m_weather->temperature);

      Units::Speed ev = tas_com - V;

      // adjust esf based on velocity error compared to the speed threshold
      if (ev <= -m_speed_thresh) {
         esf = 0.3;
      } else if (ev > -m_speed_thresh && ev < Units::ZERO_SPEED) {
         esf = (esf - 0.3) / m_speed_thresh * ev + esf;
      } else if (ev > Units::ZERO_SPEED && ev < m_speed_thresh) {
         esf = (1.7 - esf) / m_speed_thresh * ev + esf;
      } else if (ev >= m_speed_thresh) {
         esf = 1.7;
      }

      // descent rate
      Units::Speed dh_dt = ((T - D) * V) / (ac_mass * Units::ONE_G_ACCELERATION) * esf;

      gamma_com = Units::RadiansAngle(asin(-dh_dt / V));

      //if(gamma_com > 4.0*M_PI/180)
      //	gamma_com = 4.0*M_PI/180;

      if (e_alt < -m_alt_thresh) {
         T_com = 0.50 * maxThrust;
      } else if (e_alt > m_alt_thresh) {
         T_com = minThrust;
      } else {
         T_com = (minThrust - 0.50 * maxThrust) / (m_alt_thresh * 2) * e_alt + (0.50 * maxThrust + minThrust) / 2.0;
      }

      // Check if flight should level off
      if (Units::MetersLength(h).value() - m_fms->m_constraints[m_fms->m_next_waypoint_ix].constraint_altLow < 100.0 * FEET_TO_METERS) {
         m_level_flight = true;
      }

   }

   // limit thrust to max and min limits
   if (T_com > maxThrust) {
      T_com = maxThrust;
   } else if (T_com < minThrust) {
      T_com = minThrust;
   }

   // Determine if speed brake is needed
   Units::Length xEnd = Units::FeetLength(m_fms->m_waypoint_x[m_fms->m_number_of_waypoints - 1]);
   Units::Length yEnd = Units::FeetLength(m_fms->m_waypoint_y[m_fms->m_number_of_waypoints - 1]);

   Units::Length distToEnd = sqrt(Units::sqr(x - xEnd) + Units::sqr(y - yEnd));

   //if(T_com == minThrust && distToEnd < 15.0 * NM_M && (h - Fms->constraints[Fms->NextWp].constraint_altLow) > 250.0 * FT_M)
   if (T_com == minThrust) {
      //T_com = minThrust;

      if (e_alt > m_alt_thresh) {

         int mode_new = 0;
         m_bada_with_calc.getConfigForDrag(v_cas, h, m_alt_at_faf, flapConfig_new, mode_new);

         if (mode_new == flapConfig_new && mode_new <= 2) {
            speedBrakeCom = 0.5;
            m_speed_brake_on = true;
            m_speed_brake_counter = m_speed_brake_counter + 1;
         }

         flapConfig_new = mode_new;
      }
      m_min_thrust_counter = m_min_thrust_counter + 1;
   } else {
      m_min_thrust_counter = 0.0;
   }

   // If no longer commanding Min Thrust
   if (m_speed_brake_on && m_speed_brake_counter <= 30.0) {
      m_speed_brake_counter = m_speed_brake_counter + 1;
      speedBrakeCom = 0.5;
   } else if (m_speed_brake_on && m_speed_brake_counter > 30.0) {
      if (m_min_thrust_counter == 0) {
         m_speed_brake_counter = 0.0;
         speedBrakeCom = 0.0;
         m_speed_brake_on = false;
      } else {
         m_speed_brake_counter = m_speed_brake_counter + 1;
         speedBrakeCom = 0.5;
      }
   }


   //% calculate the firsr-order derivative of the state vector:
   //Wind speed needs to be checked!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
   dX.dx = Units::MetersPerSecondSpeed(V * cos(gamma) * cos(psi) + m_weather->Vwx);
   dX.dy = Units::MetersPerSecondSpeed(V * cos(gamma) * sin(psi) + m_weather->Vwy);
   dX.dh = Units::MetersPerSecondSpeed(-V * sin(gamma));
   dX.dV = (T - D) / ac_mass + Units::ONE_G_ACCELERATION * sin(gamma)
   + V * (m_weather->dVwx_dh * cos(psi) + m_weather->dVwy_dh * sin(psi))
   * sin(gamma) * cos(gamma);
   dX.dgamma = k_gamma * (gamma_com - gamma) -
         (m_weather->dVwx_dh * cos(psi) + m_weather->dVwy_dh * sin(psi)) * pow(sin(gamma), 2) *
         Units::ONE_RADIAN_ANGLE;
   dX.dpsi = (-L * sin(phi) / (ac_mass * V * cos(gamma))
         - (m_weather->dVwx_dh * sin(psi) - m_weather->dVwy_dh * cos(psi)) * tan(gamma)) * Units::ONE_RADIAN_ANGLE;
   dX.dT = k_t * (T_com - T);
   dX.dphi = k_phi * (phi_com - phi);
   dX.dspeedBrake = k_speedBrake * (speedBrakeCom - speedBrake);
   dX.flapConfig = flapConfig_new;

   return dX;
} // speed_on_pitch_control_dynamics

const Units::Speed &TestFrameworkDynamics::GetWindVelocityX() const {
   return m_wind_velocity_x;
}

const Units::Speed &TestFrameworkDynamics::GetWindVelocityY() const {
   return m_wind_velocity_y;
}

void TestFrameworkDynamics::SetWeatherTruth(const WeatherTruth &weather_truth) {
   m_weather_truth = weather_truth;
}
