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

#include <iomanip>
#include "public/CoreUtils.h"
#include "public/ThreeDOFDynamics.h"
#include "public/Wind.h"
#include "public/SimulationTime.h"
#include "public/AircraftCalculations.h"
#include "utility/Logging.h"
#include <nlohmann/json.hpp>

using json = nlohmann::json;
using namespace std;
using namespace aaesim::open_source;

log4cplus::Logger ThreeDOFDynamics::m_logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("ThreeDOFDynamics"));

ThreeDOFDynamics::ThreeDOFDynamics() {
   m_max_thrust_percent = 1.0;
   m_min_thrust_percent = 1.0;
   m_wind_velocity_east = Units::MetersPerSecondSpeed(0.0);
   m_wind_velocity_north = Units::MetersPerSecondSpeed(0.0);
   m_dynamics_state.flap_configuration = aaesim::open_source::bada_utils::FlapConfiguration::UNDEFINED;
}

AircraftState ThreeDOFDynamics::Update(const Guidance &guidance,
                                       const shared_ptr<AircraftControl> &aircraft_control) {
   return Integrate(guidance, aircraft_control);
}

AircraftState ThreeDOFDynamics::Integrate(const Guidance &guidance,
                                          const shared_ptr<AircraftControl> &aircraft_control) {
   const Units::SecondsTime dt = SimulationTime::get_simulation_time_step();

   Units::Frequency dVwx_dh, dVwy_dh;
   CalculateEnvironmentalWind(m_true_weather.east_west, m_true_weather.north_south, dVwx_dh, dVwy_dh);

   ControlCommands control_commands = aircraft_control->CalculateControlCommands(guidance,
                                                                                 m_equations_of_motion_state,
                                                                                 m_true_weather);

   bool perform_takeoff_roll_logic = control_commands.getFlapMode()==bada_utils::TAKEOFF &&
                                     control_commands.getTrueAirspeed() < m_bada_calculator->GetAerodynamicsInformation().take_off.V_stall;
   EquationsOfMotionStateDeriv state_deriv;
   if (perform_takeoff_roll_logic) {
      state_deriv = StatePropagationOnRunway(control_commands, guidance);
   } else {
      // First-order derivative of the state calculated by the EOM
      state_deriv = StatePropagation(dVwx_dh,
                                     dVwy_dh,
                                     aircraft_control->GetGammaGain(),
                                     aircraft_control->GetThrustGain(),
                                     aircraft_control->GetPhiGain(),
                                     aircraft_control->GetSpeedBrakeGain(),
                                     control_commands);
   }

   //Integrate the state
   m_equations_of_motion_state.enu_x += state_deriv.enu_velocity_x * dt;
   m_equations_of_motion_state.enu_y += state_deriv.enu_velocity_y * dt;
   m_equations_of_motion_state.altitude_msl += state_deriv.enu_velocity_z * dt;
   m_equations_of_motion_state.true_airspeed += state_deriv.true_airspeed_deriv * dt;
   m_equations_of_motion_state.gamma += state_deriv.gamma_deriv * dt;
   m_equations_of_motion_state.psi_enu += state_deriv.heading_deriv * dt;
   m_equations_of_motion_state.thrust += state_deriv.thrust_deriv * dt;
   m_equations_of_motion_state.phi += state_deriv.roll_rate * dt;
   m_equations_of_motion_state.speedBrake += state_deriv.speed_brake_deriv * dt.value();
   m_equations_of_motion_state.flapConfig = state_deriv.flap_configuration;

   // States
   m_dynamics_state.x = m_equations_of_motion_state.enu_x;
   m_dynamics_state.y = m_equations_of_motion_state.enu_y;
   m_dynamics_state.h = m_equations_of_motion_state.altitude_msl;
   m_dynamics_state.v_true_airspeed = m_equations_of_motion_state.true_airspeed;
   m_dynamics_state.gamma = m_equations_of_motion_state.gamma; // flight-path angle NOTE: for gamma, down is positive; up is negative
   m_dynamics_state.psi = m_equations_of_motion_state.psi_enu; // heading angle measured from east counter-clockwise (rad)
   m_dynamics_state.thrust = m_equations_of_motion_state.thrust;
   m_dynamics_state.phi = m_equations_of_motion_state.phi;
   m_dynamics_state.speed_brake = m_equations_of_motion_state.speedBrake;      // speed brake (% of deployment)
   m_dynamics_state.flap_configuration = m_equations_of_motion_state.flapConfig;
   m_equations_of_motion_state_derivative = state_deriv;

   // Use xdot and ydot from Dynamics to ensure winds are used
   Units::Speed xdot = state_deriv.enu_velocity_x;  //ground speed (m/s)
   Units::Speed ydot = state_deriv.enu_velocity_y;  //ground speed (m/s)

   m_dynamics_state.xd = xdot;
   m_dynamics_state.yd = ydot;

   // Check Thrust Limits and Limit Appropriately
   m_dynamics_state.v_indicated_airspeed = m_true_weather.getAtmosphere()->TAS2CAS(Units::MetersPerSecondSpeed(m_dynamics_state.v_true_airspeed),
                                                                                   Units::MetersLength(m_dynamics_state.h));

   aaesim::open_source::bada_utils::FlapConfiguration mode = m_bada_calculator->GetCurrentFlapConfiguration();

   Units::Force max_thrust, min_thrust;
   switch (mode) {
      case aaesim::open_source::bada_utils::FlapConfiguration::CRUISE:
      case aaesim::open_source::bada_utils::FlapConfiguration::APPROACH:
      case aaesim::open_source::bada_utils::FlapConfiguration::LANDING:
      case aaesim::open_source::bada_utils::FlapConfiguration::GEAR_DOWN:
         max_thrust = Units::NewtonsForce(
            m_bada_calculator->GetMaxThrust(Units::MetersLength(m_dynamics_state.h), mode, aaesim::open_source::bada_utils::EngineThrustMode::MAXIMUM_CRUISE, Units::ZERO_CELSIUS));
         break;

      case aaesim::open_source::bada_utils::FlapConfiguration::TAKEOFF:
      case aaesim::open_source::bada_utils::FlapConfiguration::INITIAL_CLIMB:
         max_thrust = Units::NewtonsForce(
            m_bada_calculator->GetMaxThrust(Units::MetersLength(m_dynamics_state.h), mode, aaesim::open_source::bada_utils::EngineThrustMode::MAXIMUM_CLIMB, Units::ZERO_CELSIUS));
         break;

      case aaesim::open_source::bada_utils::FlapConfiguration::UNDEFINED:
      default:
         throw std::logic_error("Design Error: should never get here");
   }
   min_thrust = Units::NewtonsForce(
      m_bada_calculator->GetMaxThrust(Units::MetersLength(m_dynamics_state.h), mode, aaesim::open_source::bada_utils::EngineThrustMode::DESCENT, Units::ZERO_CELSIUS));

   if (m_dynamics_state.thrust > max_thrust) {
      m_dynamics_state.thrust = max_thrust;
   } else if (m_dynamics_state.thrust < min_thrust) {
      m_dynamics_state.thrust = min_thrust;
   }

   if (m_dynamics_state.speed_brake > 0.5) {
      m_dynamics_state.speed_brake = 0.5;
   } else if (m_dynamics_state.speed_brake < 0.0) {
      m_dynamics_state.speed_brake = 0.0;
   }

   // Update wind values for output
   Units::Angle trk = Units::RadiansAngle(m_dynamics_state.psi);
   Units::Speed Vw_para = m_wind_velocity_east * cos(trk) + m_wind_velocity_north * sin(trk);
   Units::Speed Vw_perp = -m_wind_velocity_east * sin(trk) + m_wind_velocity_north * cos(trk);

   // assign return values
   AircraftState result_state;
   result_state.m_x = Units::FeetLength(m_dynamics_state.x).value() ;
   result_state.m_y = Units::FeetLength(m_dynamics_state.y).value();
   result_state.m_z = Units::FeetLength(m_dynamics_state.h).value();
   result_state.SetPsi(m_dynamics_state.psi);
   result_state.m_xd = Units::FeetPerSecondSpeed(xdot).value();
   result_state.m_yd = Units::FeetPerSecondSpeed(ydot).value();
   result_state.SetZd(-Units::FeetPerSecondSpeed(m_dynamics_state.v_true_airspeed).value() * sin(m_dynamics_state.gamma));
   result_state.m_gamma = Units::RadiansAngle(m_dynamics_state.gamma).value();
   result_state.m_Vwx = Units::MetersPerSecondSpeed(m_wind_velocity_east).value();
   result_state.m_Vwy = Units::MetersPerSecondSpeed(m_wind_velocity_north).value();
   result_state.m_Vw_para = Units::MetersPerSecondSpeed(Vw_para).value();
   result_state.m_Vw_perp = Units::MetersPerSecondSpeed(Vw_perp).value();
   result_state.m_Vwx_dh = dVwx_dh;
   result_state.m_Vwy_dh = dVwy_dh;
   result_state.m_sensed_temperature = m_true_weather.GetTemperature();
   result_state.m_sensed_density = m_true_weather.GetDensity();
   result_state.m_sensed_pressure = m_true_weather.GetPressure();
   result_state.m_dynamics_state = m_dynamics_state;

   return result_state;
}

EquationsOfMotionStateDeriv ThreeDOFDynamics::StatePropagation(Units::Frequency dVwx_dh,
                                                               Units::Frequency dVwy_dh,
                                                               Units::Frequency k_gamma,
                                                               Units::Frequency k_t,
                                                               Units::Frequency k_phi,
                                                               double k_speedBrake,
                                                               ControlCommands commands) {
   // Aircraft Configuration
   const Units::Mass ac_mass = m_bada_calculator->GetAircraftMass();

   // States
   const Units::Speed true_airspeed = m_equations_of_motion_state.true_airspeed;
   const Units::Angle gamma = m_equations_of_motion_state.gamma;     // flight-path angle
   const Units::Angle psi = m_equations_of_motion_state.psi_enu;         // heading angle measured from east counter-clockwise
   const Units::Force thrust = m_equations_of_motion_state.thrust;
   const Units::Angle phi = m_equations_of_motion_state.phi;         // roll angle
   const double speedBrake = m_equations_of_motion_state.speedBrake; // speed brake (% of deployment)

   Units::Force drag, lift;
   CalculateKineticForces(lift, drag);

   // calculate the first-order derivative of the state vector
   EquationsOfMotionStateDeriv dX;
   dX.enu_velocity_x = Units::MetersPerSecondSpeed(true_airspeed * cos(gamma) * cos(psi) + m_wind_velocity_east);
   dX.enu_velocity_y = Units::MetersPerSecondSpeed(true_airspeed * cos(gamma) * sin(psi) + m_wind_velocity_north);
   dX.enu_velocity_z = Units::MetersPerSecondSpeed(-true_airspeed * sin(gamma));
   dX.true_airspeed_deriv = (thrust - drag) / ac_mass + Units::ONE_G_ACCELERATION * sin(gamma)
                            + true_airspeed * (dVwx_dh * cos(psi) + dVwy_dh * sin(psi))
                              * sin(gamma) * cos(gamma);
   dX.gamma_deriv = k_gamma * (commands.getGamma() - gamma) -
                    (dVwx_dh * cos(psi) + dVwy_dh * sin(psi)) * pow(sin(gamma), 2) * Units::ONE_RADIAN_ANGLE;
   dX.heading_deriv = (-lift * sin(phi) / (ac_mass * true_airspeed * cos(gamma)) -
                       (dVwx_dh * sin(psi) - dVwy_dh * cos(psi)) * tan(gamma)) * Units::ONE_RADIAN_ANGLE;
   dX.thrust_deriv = k_t * (commands.getThrust() - thrust);
   dX.roll_rate = k_phi * (commands.getPhi() - phi);
   dX.speed_brake_deriv = k_speedBrake * (commands.getSpeedBrake() - speedBrake);
   dX.flap_configuration = commands.getFlapMode();

   return dX;
}

void ThreeDOFDynamics::CalculateKineticForces(Units::Force &lift,
                                              Units::Force &drag) {
   // Aircraft Configuration
   const Units::Mass ac_mass = m_bada_calculator->GetAircraftMass();
   const Units::Area wing_area = m_bada_calculator->GetAerodynamicsInformation().S;

   // States important for this method
   const Units::Length altitude_msl = m_equations_of_motion_state.altitude_msl;
   const Units::Speed true_airspeed = m_equations_of_motion_state.true_airspeed;
   const Units::Angle phi = m_equations_of_motion_state.phi; // roll angle
   const double speed_brake_setting = m_equations_of_motion_state.speedBrake;  // speed brake (% of deployment)

   // Get temp, density, and pressure
   Units::KilogramsMeterDensity rho(m_true_weather.GetDensity());
   Units::PascalsPressure pressure(m_true_weather.GetPressure());

   // Get aerodynamic configuration
   double cd0, cd2, gear;
   aaesim::open_source::bada_utils::FlapConfiguration
      updated_flap_setting = aaesim::open_source::bada_utils::FlapConfiguration::UNDEFINED;
   m_dynamics_state.v_indicated_airspeed = m_true_weather.getAtmosphere()->TAS2CAS(true_airspeed, pressure, rho);
   m_bada_calculator->GetDragCoefficients(m_dynamics_state.v_indicated_airspeed,
                                          altitude_msl,
                                          m_dynamics_state.flap_configuration,
                                          cd0,
                                          cd2,
                                          gear,
                                          updated_flap_setting);

   // Lift and Drag Coefficients
   const double cL = (2. * ac_mass * Units::ONE_G_ACCELERATION) / (rho * Units::sqr(true_airspeed) * wing_area * cos(phi));
   double cD = cd0 + gear + cd2 * pow(cL, 2); 
   if (speed_brake_setting != 0.0) {
      cD = (1.0 + 0.6 * speed_brake_setting) * cD;
   }

   // Drag & Lift
   drag = 1. / 2. * rho * cD * Units::sqr(true_airspeed) * wing_area;
   lift = 1. / 2. * rho * cL * Units::sqr(true_airspeed) * wing_area;

   if (m_logger.getLogLevel() == log4cplus::TRACE_LOG_LEVEL) {
      json j;
      j["mass_kg"] = Units::KilogramsMass(ac_mass).value();
      j["altitude_msl_ft"] = Units::FeetLength(altitude_msl).value();
      j["true_airspeed_kts"] = Units::KnotsSpeed(true_airspeed).value();
      j["rho_kgm3"] = Units::KilogramsMeterDensity(rho).value();
      j["gear"] = gear;
      j["speed_brake_setting"] = speed_brake_setting;
      j["cD"] = cD;
      j["drag_newtons"] = Units::NewtonsForce(drag).value();
      j["cL"] = cL;
      j["lift_newtons"] = Units::NewtonsForce(lift).value();
      LOG4CPLUS_TRACE(m_logger, j.dump());   
   }
}

Units::SignedRadiansAngle ThreeDOFDynamics::CalculateTrimmedPsiForWind(Units::SignedAngle ground_track_enu) {
   // Get wind information
   WindStack windstackx(1, 5);
   WindStack windstacky(1, 5);
   Units::Frequency dvwxdh, dvwydh;
   CalculateEnvironmentalWind(windstackx, windstacky, dvwxdh, dvwydh);

   const double gamma = Units::RadiansAngle(m_equations_of_motion_state.gamma).value();
   const double trkRad = Units::RadiansAngle(AircraftCalculations::Convert0to2Pi(ground_track_enu)).value();
   Units::MetersPerSecondSpeed vwpara = Units::MetersPerSecondSpeed(
         Units::MetersPerSecondSpeed(m_wind_velocity_east).value() * cos(trkRad) +
         Units::MetersPerSecondSpeed(m_wind_velocity_north).value() * sin(trkRad));
   Units::MetersPerSecondSpeed vwperp = Units::MetersPerSecondSpeed(
         -Units::MetersPerSecondSpeed(m_wind_velocity_east).value() * sin(trkRad) +
         Units::MetersPerSecondSpeed(m_wind_velocity_north).value() * cos(trkRad));

   Units::MetersPerSecondSpeed w = sqrt(Units::sqr(Units::MetersPerSecondSpeed(m_wind_velocity_east)) +
                                        Units::sqr(Units::MetersPerSecondSpeed(m_wind_velocity_north)));

   const Units::MetersPerSecondSpeed v = m_equations_of_motion_state.true_airspeed;
   Units::MetersPerSecondSpeed gs = Units::MetersPerSecondSpeed(sqrt(
         pow(v.value() * cos(gamma), 2) -
         pow(vwperp.value(), 2)) + vwpara.value());

   double numerator = (pow(v.value() * cos(gamma), 2) +
                       pow(gs.value(), 2) -
                       pow(w.value(), 2));

   double denominator = (2.0 * v.value() * cos(gamma) *
                         gs.value());

   double temp = numerator / denominator;
   if (temp > 1.0) { // Limit temp so acos function doesn't give undefined value.
      temp = 1.0;
   } else if (temp < -1.0) {
      temp = -1.0;
   }

   // Wind correction angle
   Units::Angle beta = Units::RadiansAngle(acos(temp) * -1.0 *
                                           CoreUtils::SignOfValue(vwperp.value()));

   return ground_track_enu + beta;
}
void ThreeDOFDynamics::Initialize(std::shared_ptr<const BadaPerformanceCalculator> aircraft_performance,
                                  const Waypoint &initial_waypoint,
                                  std::shared_ptr<TangentPlaneSequence> tangent_plane_sequence,
                                  Units::Length initial_altitude_msl,
                                  Units::Speed initial_true_airspeed,
                                  Units::Angle initial_ground_course_enu,
                                  double initial_mass_fraction,
                                  const WeatherTruth &true_weather) {
   m_true_weather = true_weather;
   m_bada_calculator = aircraft_performance;

   m_dynamics_state.h = initial_altitude_msl;
   m_dynamics_state.gamma = Units::RadiansAngle(0);
   m_dynamics_state.phi = Units::RadiansAngle(0);
   m_dynamics_state.v_true_airspeed = initial_true_airspeed;
   m_dynamics_state.flap_configuration = m_bada_calculator->GetCurrentFlapConfiguration();

   // Actually initialize the state
   m_equations_of_motion_state.altitude_msl = Units::MetersLength(m_dynamics_state.h); // altitude (m)
   m_equations_of_motion_state.true_airspeed = Units::MetersPerSecondSpeed(m_dynamics_state.v_true_airspeed); // true airspeed (m/s)
   m_equations_of_motion_state.gamma = Units::RadiansAngle(m_dynamics_state.gamma); // flight-path angle (rad); NOTE: for gamma, heading down is positve
   m_equations_of_motion_state.thrust = Units::NewtonsForce(0.0); // thrust (N)
   m_equations_of_motion_state.phi = Units::RadiansAngle(m_dynamics_state.phi); // roll angle (rad)
   m_equations_of_motion_state.speedBrake = 0.0; // speed brake (% of deployment)
   m_equations_of_motion_state.flapConfig = m_bada_calculator->GetCurrentFlapConfiguration();

   if (m_dynamics_state.flap_configuration == bada_utils::FlapConfiguration::TAKEOFF) {
      // On the ground. Don't trim aircraft.
      m_dynamics_state.psi = initial_ground_course_enu;
      m_equations_of_motion_state.psi_enu = m_dynamics_state.psi;
      m_dynamics_state.xd = m_dynamics_state.v_true_airspeed * cos(m_dynamics_state.psi); 
      m_dynamics_state.yd = m_dynamics_state.v_true_airspeed * sin(m_dynamics_state.psi); 

      Units::Force takeoff_max_thrust = m_bada_calculator->GetMaxThrust(Units::MetersLength(m_dynamics_state.h), aaesim::open_source::bada_utils::FlapConfiguration::TAKEOFF, aaesim::open_source::bada_utils::EngineThrustMode::MAXIMUM_CLIMB, Units::ZERO_CELSIUS);
      m_equations_of_motion_state.thrust = takeoff_max_thrust;
   } else {
      // Now that the initial state has been determined, still need to trim laterally for wind
      m_equations_of_motion_state.psi_enu = CalculateTrimmedPsiForWind(initial_ground_course_enu);
      m_dynamics_state.psi = m_equations_of_motion_state.psi_enu;
      m_dynamics_state.xd = m_dynamics_state.v_true_airspeed * cos(m_dynamics_state.psi) * cos(m_dynamics_state.gamma) +
         m_wind_velocity_east;
      m_dynamics_state.yd = m_dynamics_state.v_true_airspeed * sin(m_dynamics_state.psi) * cos(m_dynamics_state.gamma) +
         m_wind_velocity_north;

      // Calculate initial Aircraft Thrust
      Units::Mass ac_mass = m_bada_calculator->GetAircraftMass();
      Units::Force drag, lift;
      CalculateKineticForces(lift, drag);
      Units::Force equilibrium_thrust_required = drag - ac_mass * Units::ONE_G_ACCELERATION * sin(asin(0.0));
      const Units::Force max_thrust = m_bada_calculator->GetMaxThrust(Units::MetersLength(m_dynamics_state.h), aaesim::open_source::bada_utils::FlapConfiguration::CRUISE, aaesim::open_source::bada_utils::EngineThrustMode::MAXIMUM_CRUISE, Units::ZERO_CELSIUS);
      const Units::Force min_thrust = m_bada_calculator->GetMaxThrust(Units::MetersLength(m_dynamics_state.h), m_dynamics_state.flap_configuration, aaesim::open_source::bada_utils::EngineThrustMode::DESCENT, Units::ZERO_CELSIUS);
      if (equilibrium_thrust_required > max_thrust * m_max_thrust_percent) {
         equilibrium_thrust_required = max_thrust * m_max_thrust_percent;
      } else if (equilibrium_thrust_required < min_thrust * m_min_thrust_percent) {
         equilibrium_thrust_required = min_thrust * m_min_thrust_percent;
      }
      m_equations_of_motion_state.thrust = equilibrium_thrust_required;
   }
}

EquationsOfMotionStateDeriv ThreeDOFDynamics::StatePropagationOnRunway(ControlCommands commands, const Guidance &guidance) {
   const double wind_factor = 1.25;
   const Units::SignedAngle takeoff_roll_psi_enu = guidance.m_enu_track_angle;
   const Units::Mass ac_mass = m_bada_calculator->GetAircraftMass();
   const Units::Speed true_airspeed = m_equations_of_motion_state.true_airspeed;
   const Units::Force thrust = m_bada_calculator->GetMaxThrust(m_equations_of_motion_state.altitude_msl,commands.getFlapMode(),bada_utils::EngineThrustMode::MAXIMUM_CLIMB, Units::ZERO_CELSIUS);
   const Units::Speed wind_magnitude = Units::sqrt(Units::sqr(m_wind_velocity_east)+Units::sqr(m_wind_velocity_north));
   Units::Speed wind_east_parallel_to_track = m_wind_velocity_east * cos(takeoff_roll_psi_enu);
   Units::Speed wind_north_parallel_to_track = m_wind_velocity_north * sin(takeoff_roll_psi_enu);
   if (true_airspeed < wind_magnitude*wind_factor) {
      // ignore wind; don't allow aircraft to be blown around on runway
      wind_east_parallel_to_track = Units::zero();
      wind_north_parallel_to_track = Units::zero();
   }

   EquationsOfMotionStateDeriv dX;
   dX.enu_velocity_x = true_airspeed * cos(takeoff_roll_psi_enu) + wind_east_parallel_to_track;
   dX.enu_velocity_y = true_airspeed * sin(takeoff_roll_psi_enu) + wind_north_parallel_to_track;
   dX.enu_velocity_z = Units::zero();
   dX.true_airspeed_deriv = thrust/ac_mass;
   dX.gamma_deriv = Units::zero();
   auto derived_psi_enu = Units::arctan2(Units::MetersPerSecondSpeed(dX.enu_velocity_y).value(), Units::MetersPerSecondSpeed(dX.enu_velocity_x).value());
   auto delta_psi_enu = derived_psi_enu-m_equations_of_motion_state.psi_enu;
   dX.heading_deriv = delta_psi_enu/SimulationTime::get_simulation_time_step();
   dX.thrust_deriv = Units::zero();
   dX.roll_rate = Units::zero();
   dX.speed_brake_deriv = 0;
   dX.flap_configuration = commands.getFlapMode();
   return dX;
}

