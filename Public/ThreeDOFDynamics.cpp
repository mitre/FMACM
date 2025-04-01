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

#include "public/ThreeDOFDynamics.h"

#include <iomanip>
#include <nlohmann/json.hpp>

#include "public/CoreUtils.h"
#include "public/Logging.h"

using json = nlohmann::json;
using namespace std;
using namespace aaesim::open_source;

AircraftState ThreeDOFDynamics::Update(const int unique_acid, const aaesim::open_source::SimulationTime &simtime,
                                       const Guidance &guidance, const shared_ptr<AircraftControl> &aircraft_control) {
   auto dynamics_state = Integrate(guidance, aircraft_control);
   m_dynamics_history.insert(std::make_pair(simtime.GetCurrentSimulationTime(), dynamics_state));

   LatLonDerivative position_rate;
   m_position_estimator->ComputePosition(simtime, m_equations_of_motion_state, m_equations_of_motion_state_derivative,
                                         m_last_resolved_position, position_rate);
   Units::Angle trk = Units::RadiansAngle(dynamics_state.psi);
   Units::Speed Vw_para = m_wind_velocity_east * cos(trk) + m_wind_velocity_north * sin(trk);
   Units::Speed Vw_perp = -m_wind_velocity_east * sin(trk) + m_wind_velocity_north * cos(trk);
   Units::Temperature outside_air_temperature = m_true_weather_operator->GetTemperature();
   return AircraftState::Builder(unique_acid, simtime.GetCurrentSimulationTime())
         .Position(m_equations_of_motion_state.enu_x, m_equations_of_motion_state.enu_y)
         ->AltitudeMsl(dynamics_state.h)
         ->Psi(dynamics_state.psi)
         ->GroundSpeed(dynamics_state.xd, dynamics_state.yd)
         ->AltitudeRate(-dynamics_state.v_true_airspeed * Units::sin(dynamics_state.gamma))
         ->FlightPathAngle(dynamics_state.gamma)
         ->SensedWindComponents(m_wind_velocity_east, m_wind_velocity_north)
         ->SensedWindsParallel(Vw_para)
         ->SensedWindsPerpendicular(Vw_perp)
         ->VerticalWindDerivatives(m_true_weather_operator->GetWindSpeedVerticalDerivativeEast(),
                                   m_true_weather_operator->GetWindSpeedVerticalDerivativeNorth())
         ->SensedTemperature(outside_air_temperature)
         ->SensedDensity(m_true_weather_operator->GetDensity())
         ->SensedPressure(m_true_weather_operator->GetPressure())
         ->DynamicsState(dynamics_state)
         ->Latitude(m_last_resolved_position.latitude)
         ->Longitude(m_last_resolved_position.longitude)
         ->LatitudeRate(position_rate.latitude_time_derivative)
         ->LongitudeRate(position_rate.longitude_time_derivative)
         ->Build();
}

DynamicsState ThreeDOFDynamics::Integrate(const Guidance &guidance,
                                          const shared_ptr<AircraftControl> &aircraft_control) {
   const Units::SecondsTime dt = SimulationTime::GetSimulationTimeStep();
   UpdateTrueWeatherConditions();
   ControlCommands control_commands =
         aircraft_control->CalculateControlCommands(guidance, m_equations_of_motion_state, m_true_weather_operator);
   bool perform_takeoff_roll_logic =
         control_commands.getFlapMode() == bada_utils::TAKEOFF &&
         control_commands.getTrueAirspeed() < m_bada_calculator->GetAerodynamicsInformation().take_off.V_stall;
   if (perform_takeoff_roll_logic) {
      m_equations_of_motion_state_derivative = StatePropagationOnRunway(control_commands, guidance);
   } else {
      // First-order derivative of the state calculated by the EOM
      m_equations_of_motion_state_derivative =
            StatePropagation(m_true_weather_operator->GetWindSpeedVerticalDerivativeEast(),
                             m_true_weather_operator->GetWindSpeedVerticalDerivativeNorth(),
                             aircraft_control->GetGammaGain(), aircraft_control->GetThrustGain(),
                             aircraft_control->GetPhiGain(), aircraft_control->GetSpeedBrakeGain(), control_commands);
   }

   // Integrate the state
   m_equations_of_motion_state.enu_x += m_equations_of_motion_state_derivative.enu_velocity_x * dt;
   m_equations_of_motion_state.enu_y += m_equations_of_motion_state_derivative.enu_velocity_y * dt;
   m_equations_of_motion_state.altitude_msl += m_equations_of_motion_state_derivative.enu_velocity_z * dt;
   m_equations_of_motion_state.true_airspeed += m_equations_of_motion_state_derivative.true_airspeed_deriv * dt;
   m_equations_of_motion_state.gamma += m_equations_of_motion_state_derivative.gamma_deriv * dt;
   m_equations_of_motion_state.psi_enu += m_equations_of_motion_state_derivative.heading_deriv * dt;
   m_equations_of_motion_state.thrust += m_equations_of_motion_state_derivative.thrust_deriv * dt;
   m_equations_of_motion_state.phi += m_equations_of_motion_state_derivative.roll_rate * dt;
   m_equations_of_motion_state.speed_brake_percentage +=
         m_equations_of_motion_state_derivative.speed_brake_deriv * dt.value();
   m_equations_of_motion_state.flap_configuration = m_equations_of_motion_state_derivative.flap_configuration;
   return ComputeDynamicsState(m_equations_of_motion_state, m_equations_of_motion_state_derivative);
}

EquationsOfMotionStateDeriv ThreeDOFDynamics::StatePropagation(Units::Frequency dVwx_dh, Units::Frequency dVwy_dh,
                                                               Units::Frequency k_gamma, Units::Frequency k_t,
                                                               Units::Frequency k_phi, double k_speedBrake,
                                                               ControlCommands commands) {
   // Aircraft Configuration
   const Units::Mass ac_mass = m_bada_calculator->GetAircraftMass();

   // States
   const Units::Speed true_airspeed = m_equations_of_motion_state.true_airspeed;
   const Units::Angle gamma = m_equations_of_motion_state.gamma;
   const Units::Angle psi = m_equations_of_motion_state.psi_enu;
   const Units::Force thrust = m_equations_of_motion_state.thrust;
   const Units::Angle phi = m_equations_of_motion_state.phi;
   const double speed_brake_percentage = m_equations_of_motion_state.speed_brake_percentage;

   Units::Force drag, lift;
   CalculateKineticForces(lift, drag);

   // calculate the first-order derivative of the state vector
   EquationsOfMotionStateDeriv dX;
   dX.enu_velocity_x = Units::MetersPerSecondSpeed(true_airspeed * cos(gamma) * cos(psi) + m_wind_velocity_east);
   dX.enu_velocity_y = Units::MetersPerSecondSpeed(true_airspeed * cos(gamma) * sin(psi) + m_wind_velocity_north);
   dX.enu_velocity_z = Units::MetersPerSecondSpeed(-true_airspeed * sin(gamma));
   dX.true_airspeed_deriv = (thrust - drag) / ac_mass + Units::ONE_G_ACCELERATION * sin(gamma) +
                            true_airspeed * (dVwx_dh * cos(psi) + dVwy_dh * sin(psi)) * sin(gamma) * cos(gamma);
   dX.gamma_deriv = k_gamma * (commands.getGamma() - gamma) -
                    (dVwx_dh * cos(psi) + dVwy_dh * sin(psi)) * pow(sin(gamma), 2) * Units::ONE_RADIAN_ANGLE;
   dX.heading_deriv = (-lift * sin(phi) / (ac_mass * true_airspeed * cos(gamma)) -
                       (dVwx_dh * sin(psi) - dVwy_dh * cos(psi)) * tan(gamma)) *
                      Units::ONE_RADIAN_ANGLE;
   dX.thrust_deriv = k_t * (commands.getThrust() - thrust);
   dX.roll_rate = k_phi * (commands.getPhi() - phi);
   dX.speed_brake_deriv = k_speedBrake * (commands.getSpeedBrake() - speed_brake_percentage);
   dX.flap_configuration = commands.getFlapMode();

   return dX;
}

void ThreeDOFDynamics::CalculateKineticForces(Units::Force &lift, Units::Force &drag) {
   // Aircraft Configuration
   const Units::Mass ac_mass = m_bada_calculator->GetAircraftMass();
   const Units::Area wing_area = m_bada_calculator->GetAerodynamicsInformation().S;

   // States important for this method
   const Units::Length altitude_msl = m_equations_of_motion_state.altitude_msl;
   const Units::Speed true_airspeed = m_equations_of_motion_state.true_airspeed;
   const Units::Angle phi = m_equations_of_motion_state.phi;  // roll angle
   const double speed_brake_setting =
         m_equations_of_motion_state.speed_brake_percentage;  // speed brake (% of deployment)

   // Get temp, density, and pressure
   Units::KilogramsMeterDensity rho(m_true_weather_operator->GetDensity());
   Units::PascalsPressure pressure(m_true_weather_operator->GetPressure());

   // Get aerodynamic configuration
   double cd0, cd2, gear;
   m_bada_calculator->GetCurrentDragCoefficients(cd0, cd2, gear);

   // Lift and Drag Coefficients
   const double cL =
         (2. * ac_mass * Units::ONE_G_ACCELERATION) / (rho * Units::sqr(true_airspeed) * wing_area * cos(phi));
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
      j["cd0"] = cd0;
      j["cd2"] = cd2;
      j["cD"] = cD;
      j["drag_newtons"] = Units::NewtonsForce(drag).value();
      j["cL"] = cL;
      j["lift_newtons"] = Units::NewtonsForce(lift).value();
      j["speed_brake_setting"] = speed_brake_setting;
      j["updated_flap_setting"] = aaesim::open_source::bada_utils::GetFlapConfigurationAsString(
            m_bada_calculator->GetCurrentFlapConfiguration());
      LOG4CPLUS_TRACE(m_logger, j.dump());
   }
}

Units::SignedRadiansAngle ThreeDOFDynamics::CalculateTrimmedPsiForWind(Units::SignedAngle ground_track_enu) {
   UpdateTrueWeatherConditions();
   const double gamma = Units::RadiansAngle(m_equations_of_motion_state.gamma).value();
   const double trkRad = Units::RadiansAngle(Units::ToUnsigned(ground_track_enu)).value();
   Units::MetersPerSecondSpeed vwpara =
         Units::MetersPerSecondSpeed(Units::MetersPerSecondSpeed(m_wind_velocity_east).value() * cos(trkRad) +
                                     Units::MetersPerSecondSpeed(m_wind_velocity_north).value() * sin(trkRad));
   Units::MetersPerSecondSpeed vwperp =
         Units::MetersPerSecondSpeed(-Units::MetersPerSecondSpeed(m_wind_velocity_east).value() * sin(trkRad) +
                                     Units::MetersPerSecondSpeed(m_wind_velocity_north).value() * cos(trkRad));

   Units::MetersPerSecondSpeed w = sqrt(Units::sqr(Units::MetersPerSecondSpeed(m_wind_velocity_east)) +
                                        Units::sqr(Units::MetersPerSecondSpeed(m_wind_velocity_north)));

   const Units::MetersPerSecondSpeed v = m_equations_of_motion_state.true_airspeed;
   Units::MetersPerSecondSpeed gs =
         Units::MetersPerSecondSpeed(sqrt(pow(v.value() * cos(gamma), 2) - pow(vwperp.value(), 2)) + vwpara.value());

   double numerator = (pow(v.value() * cos(gamma), 2) + pow(gs.value(), 2) - pow(w.value(), 2));

   double denominator = (2.0 * v.value() * cos(gamma) * gs.value());

   double temp = numerator / denominator;
   if (temp > 1.0) {  // Limit temp so acos function doesn't give undefined value.
      temp = 1.0;
   } else if (temp < -1.0) {
      temp = -1.0;
   }

   // Wind correction angle
   Units::Angle beta = Units::RadiansAngle(acos(temp) * -1.0 * CoreUtils::SignOfValue(vwperp.value()));

   return ground_track_enu + beta;
}

void ThreeDOFDynamics::Initialize(
      std::shared_ptr<const aaesim::open_source::FixedMassAircraftPerformance> aircraft_performance,
      const EarthModel::GeodeticPosition &initial_position, const EarthModel::LocalPositionEnu &initial_position_enu,
      Units::Length initial_altitude_msl, Units::Speed initial_true_airspeed, Units::Angle initial_ground_course_enu,
      double initial_mass_fraction,
      std::shared_ptr<aaesim::open_source::EllipsoidalPositionEstimator> position_estimator,
      std::shared_ptr<aaesim::open_source::TrueWeatherOperator> true_weather_operator) {
   m_bada_calculator = aircraft_performance;
   m_position_estimator = position_estimator;
   m_true_weather_operator = true_weather_operator;

   m_last_resolved_position.latitude = initial_position.latitude;
   m_last_resolved_position.longitude = initial_position.longitude;
   m_last_resolved_position.altitude = initial_altitude_msl;

   DynamicsState initial_dynamics_state{};
   initial_dynamics_state.h = initial_altitude_msl;
   initial_dynamics_state.gamma = Units::RadiansAngle(0);
   initial_dynamics_state.phi = Units::RadiansAngle(0);
   initial_dynamics_state.v_true_airspeed = initial_true_airspeed;
   initial_dynamics_state.flap_configuration = m_bada_calculator->GetCurrentFlapConfiguration();
   initial_dynamics_state.mach =
         m_true_weather_operator->GetTrueWeather()->TAS2Mach(initial_true_airspeed, initial_altitude_msl);
   initial_dynamics_state.v_indicated_airspeed =
         m_true_weather_operator->GetTrueWeather()->TAS2CAS(initial_true_airspeed, initial_altitude_msl);

   // initialize the state
   m_equations_of_motion_state.enu_x = initial_position_enu.x;
   m_equations_of_motion_state.enu_y = initial_position_enu.y;
   m_equations_of_motion_state.altitude_msl = initial_dynamics_state.h;
   m_equations_of_motion_state.true_airspeed = initial_dynamics_state.v_true_airspeed;
   m_equations_of_motion_state.gamma = initial_dynamics_state.gamma;
   m_equations_of_motion_state.thrust = Units::ZERO_FORCE;
   m_equations_of_motion_state.phi = initial_dynamics_state.phi;
   m_equations_of_motion_state.speed_brake_percentage = 0.0;
   m_equations_of_motion_state.flap_configuration = m_bada_calculator->GetCurrentFlapConfiguration();

   if (initial_dynamics_state.flap_configuration == bada_utils::FlapConfiguration::TAKEOFF) {
      // On the ground. Don't trim aircraft.
      initial_dynamics_state.psi = initial_ground_course_enu;
      m_equations_of_motion_state.psi_enu = initial_dynamics_state.psi;
      initial_dynamics_state.xd = initial_dynamics_state.v_true_airspeed * cos(initial_dynamics_state.psi);
      initial_dynamics_state.yd = initial_dynamics_state.v_true_airspeed * sin(initial_dynamics_state.psi);

      Units::Force takeoff_max_thrust = m_bada_calculator->GetMaxThrust(
            Units::MetersLength(initial_dynamics_state.h), aaesim::open_source::bada_utils::FlapConfiguration::TAKEOFF,
            aaesim::open_source::bada_utils::EngineThrustMode::MAXIMUM_CLIMB, Units::ZERO_CELSIUS);
      m_equations_of_motion_state.thrust = takeoff_max_thrust;
   } else {
      // Now that the initial state has been determined, still need to trim laterally for wind
      m_equations_of_motion_state.psi_enu = CalculateTrimmedPsiForWind(initial_ground_course_enu);
      initial_dynamics_state.psi = m_equations_of_motion_state.psi_enu;
      initial_dynamics_state.xd = initial_dynamics_state.v_true_airspeed * cos(initial_dynamics_state.psi) *
                                        cos(initial_dynamics_state.gamma) +
                                  m_wind_velocity_east;
      initial_dynamics_state.yd = initial_dynamics_state.v_true_airspeed * sin(initial_dynamics_state.psi) *
                                        cos(initial_dynamics_state.gamma) +
                                  m_wind_velocity_north;

      // Calculate initial Aircraft Thrust
      Units::Mass ac_mass = m_bada_calculator->GetAircraftMass();
      Units::Force drag, lift;
      CalculateKineticForces(lift, drag);
      Units::Force equilibrium_thrust_required = drag - ac_mass * Units::ONE_G_ACCELERATION * sin(asin(0.0));
      const Units::Force max_thrust = m_bada_calculator->GetMaxThrust(
            Units::MetersLength(initial_dynamics_state.h), aaesim::open_source::bada_utils::FlapConfiguration::CRUISE,
            aaesim::open_source::bada_utils::EngineThrustMode::MAXIMUM_CRUISE, Units::ZERO_CELSIUS);
      const Units::Force min_thrust = m_bada_calculator->GetMaxThrust(
            Units::MetersLength(initial_dynamics_state.h), initial_dynamics_state.flap_configuration,
            aaesim::open_source::bada_utils::EngineThrustMode::DESCENT, Units::ZERO_CELSIUS);
      if (equilibrium_thrust_required > max_thrust * m_max_thrust_percent) {
         equilibrium_thrust_required = max_thrust * m_max_thrust_percent;
      } else if (equilibrium_thrust_required < min_thrust * m_min_thrust_percent) {
         equilibrium_thrust_required = min_thrust * m_min_thrust_percent;
      }
      m_equations_of_motion_state.thrust = equilibrium_thrust_required;
   }
   m_dynamics_history.insert(std::make_pair(Units::SecondsTime{-1}, initial_dynamics_state));
}

EquationsOfMotionStateDeriv ThreeDOFDynamics::StatePropagationOnRunway(ControlCommands commands,
                                                                       const Guidance &guidance) {
   const double wind_factor = 1.25;
   const Units::SignedAngle takeoff_roll_psi_enu = guidance.m_enu_track_angle;
   const Units::Mass ac_mass = m_bada_calculator->GetAircraftMass();
   const Units::Speed true_airspeed = m_equations_of_motion_state.true_airspeed;
   const Units::Force thrust =
         m_bada_calculator->GetMaxThrust(m_equations_of_motion_state.altitude_msl, commands.getFlapMode(),
                                         bada_utils::EngineThrustMode::MAXIMUM_CLIMB, Units::ZERO_CELSIUS);
   const Units::Speed wind_magnitude =
         Units::sqrt(Units::sqr(m_wind_velocity_east) + Units::sqr(m_wind_velocity_north));
   Units::Speed wind_east_parallel_to_track = m_wind_velocity_east * cos(takeoff_roll_psi_enu);
   Units::Speed wind_north_parallel_to_track = m_wind_velocity_north * sin(takeoff_roll_psi_enu);
   if (true_airspeed < wind_magnitude * wind_factor) {
      // ignore wind; don't allow aircraft to be blown around on runway
      wind_east_parallel_to_track = Units::zero();
      wind_north_parallel_to_track = Units::zero();
   }

   EquationsOfMotionStateDeriv dX;
   dX.enu_velocity_x = true_airspeed * cos(takeoff_roll_psi_enu) + wind_east_parallel_to_track;
   dX.enu_velocity_y = true_airspeed * sin(takeoff_roll_psi_enu) + wind_north_parallel_to_track;
   dX.enu_velocity_z = Units::zero();
   dX.true_airspeed_deriv = thrust / ac_mass;
   dX.gamma_deriv = Units::zero();
   auto derived_psi_enu = Units::arctan2(Units::MetersPerSecondSpeed(dX.enu_velocity_y).value(),
                                         Units::MetersPerSecondSpeed(dX.enu_velocity_x).value());
   auto delta_psi_enu = derived_psi_enu - m_equations_of_motion_state.psi_enu;
   dX.heading_deriv = delta_psi_enu / SimulationTime::GetSimulationTimeStep();
   dX.thrust_deriv = Units::zero();
   dX.roll_rate = Units::zero();
   dX.speed_brake_deriv = 0;
   dX.flap_configuration = commands.getFlapMode();
   return dX;
}

DynamicsState ThreeDOFDynamics::ComputeDynamicsState(
      const EquationsOfMotionState &equations_of_motion_state,
      const EquationsOfMotionStateDeriv &equations_of_motion_state_derivative) {
   DynamicsState dynamics_state;

   dynamics_state.h = m_equations_of_motion_state.altitude_msl;
   dynamics_state.v_true_airspeed = m_equations_of_motion_state.true_airspeed;
   dynamics_state.v_indicated_airspeed =
         m_true_weather_operator->GetTrueWeather()->TAS2CAS(dynamics_state.v_true_airspeed, dynamics_state.h);
   dynamics_state.mach = m_true_weather_operator->GetTrueWeather()->TAS2Mach(m_equations_of_motion_state.true_airspeed,
                                                                             m_equations_of_motion_state.altitude_msl);
   dynamics_state.gamma = m_equations_of_motion_state.gamma;
   dynamics_state.psi = m_equations_of_motion_state.psi_enu;
   dynamics_state.thrust = m_equations_of_motion_state.thrust;
   dynamics_state.phi = m_equations_of_motion_state.phi;
   dynamics_state.speed_brake = m_equations_of_motion_state.speed_brake_percentage;
   dynamics_state.flap_configuration = m_equations_of_motion_state.flap_configuration;
   dynamics_state.current_mass = m_bada_calculator->GetAircraftMass();
   dynamics_state.xd = equations_of_motion_state_derivative.enu_velocity_x;
   dynamics_state.yd = equations_of_motion_state_derivative.enu_velocity_y;

   // Check Thrust Limits and Limit Appropriately

   aaesim::open_source::bada_utils::FlapConfiguration mode = m_bada_calculator->GetCurrentFlapConfiguration();
   Units::Force max_thrust, min_thrust;
   switch (mode) {
      case aaesim::open_source::bada_utils::FlapConfiguration::CRUISE:
      case aaesim::open_source::bada_utils::FlapConfiguration::APPROACH:
      case aaesim::open_source::bada_utils::FlapConfiguration::LANDING:
      case aaesim::open_source::bada_utils::FlapConfiguration::GEAR_DOWN:
         max_thrust = Units::NewtonsForce(m_bada_calculator->GetMaxThrust(
               Units::MetersLength(dynamics_state.h), mode,
               aaesim::open_source::bada_utils::EngineThrustMode::MAXIMUM_CRUISE, Units::ZERO_CELSIUS));
         break;

      case aaesim::open_source::bada_utils::FlapConfiguration::TAKEOFF:
      case aaesim::open_source::bada_utils::FlapConfiguration::INITIAL_CLIMB:
         max_thrust = Units::NewtonsForce(m_bada_calculator->GetMaxThrust(
               Units::MetersLength(dynamics_state.h), mode,
               aaesim::open_source::bada_utils::EngineThrustMode::MAXIMUM_CLIMB, Units::ZERO_CELSIUS));
         break;

      case aaesim::open_source::bada_utils::FlapConfiguration::UNDEFINED:
      default:
         throw std::logic_error("Design Error: should never get here");
   }
   min_thrust = Units::NewtonsForce(m_bada_calculator->GetMaxThrust(
         Units::MetersLength(dynamics_state.h), mode, aaesim::open_source::bada_utils::EngineThrustMode::DESCENT,
         Units::ZERO_CELSIUS));

   if (dynamics_state.thrust > max_thrust) {
      dynamics_state.thrust = max_thrust;
   } else if (dynamics_state.thrust < min_thrust) {
      dynamics_state.thrust = min_thrust;
   }

   if (dynamics_state.speed_brake > 0.5) {
      dynamics_state.speed_brake = 0.5;
   } else if (dynamics_state.speed_brake < 0.0) {
      dynamics_state.speed_brake = 0.0;
   }

   dynamics_state.true_temperature =
         Units::AbsCelsiusTemperature(Units::AbsKelvinTemperature(m_true_weather_operator->GetTemperature().value()));
   return dynamics_state;
}

void ThreeDOFDynamics::UpdateTrueWeatherConditions() {
   m_true_weather_operator->CalculateEnvironmentalWind(m_last_resolved_position,
                                                       m_equations_of_motion_state.altitude_msl);
   m_wind_velocity_east = m_true_weather_operator->GetWindSpeedEast();
   m_wind_velocity_north = m_true_weather_operator->GetWindSpeedNorth();
}
