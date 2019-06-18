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

#include <public/CoreUtils.h>
#include "public/ThreeDOFDynamics.h"
#include "public/Wind.h"
#include "public/SimulationTime.h"
#include "public/AircraftCalculations.h"
#include "utility/Logging.h"

using namespace std;

log4cplus::Logger ThreeDOFDynamics::m_logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("ThreeDOFDynamics"));

ThreeDOFDynamics::ThreeDOFDynamics() {
   m_model_loaded = false;
   m_ac_type = "B753";
   m_max_thrust_percent = 1.0;
   m_min_thrust_percent = 1.0;
   m_wind_velocity_x = Units::MetersPerSecondSpeed(0.0);
   m_wind_velocity_y = Units::MetersPerSecondSpeed(0.0);
   m_altitude_msl_at_final_waypoint = Units::FeetLength(-100.0);
   m_dynamics_state.m_flap_configuration = 0;
   m_bada_calculator.setFlapSpeeds("");
}

AircraftState ThreeDOFDynamics::Update(const AircraftState &aircraft_state,
                                       const Guidance &guidance,
                                       const shared_ptr<AircraftControl> &aircraft_control) {

   LOG4CPLUS_TRACE(m_logger, "AC state pos (" <<
                                              Units::FeetLength(aircraft_state.m_x) << "," <<
                                              Units::FeetLength(aircraft_state.m_y) << "), dynamics state (" <<
                                              m_dynamics_state.x << "," <<
                                              m_dynamics_state.y << ")");

   InternalObserver::getInstance()->cross_entry.time = aircraft_state.m_time;

   if (InternalObserver::getInstance()->debugTrueWind()) {
      InternalObserver::getInstance()->setTrueWindHdrVals(aircraft_state.m_time, aircraft_state.m_id);
   }

   return Integrate(guidance, aircraft_control);
}

AircraftState ThreeDOFDynamics::Integrate(const Guidance &guidance,
                                          const shared_ptr<AircraftControl> &aircraft_control) {
   AircraftState result_state;

   //first-order derivative of the state calculated by the EOM
   EquationsOfMotionStateDeriv state_derivative;

   const Units::SecondsTime dt = SimulationTime::get_simulation_time_step();

   // Wind declarations
   Units::Frequency dVwx_dh, dVwy_dh;
   CalculateEnvironmentalWind(m_true_weather.east_west, m_true_weather.north_south, dVwx_dh, dVwy_dh);

   ControlCommands controlCommands = aircraft_control->CalculateControlCommands(guidance,
                                                                                m_equations_of_motion_state,
                                                                                m_true_weather);

   // Propagate the state to calculate the change per second for the aircraft states
   state_derivative = StatePropagation(dVwx_dh,
                                       dVwy_dh,
                                       aircraft_control->getGammaGain(),
                                       aircraft_control->getThrustGain(),
                                       aircraft_control->getPhiGain(),
                                       aircraft_control->getSpeedBrakeGain(),
                                       controlCommands);

   //Integrate the state
   m_equations_of_motion_state.enu_x += state_derivative.enu_velocity_x * dt;
   m_equations_of_motion_state.enu_y += state_derivative.enu_velocity_y * dt;
   m_equations_of_motion_state.enu_z += state_derivative.enu_velocity_z * dt;
   m_equations_of_motion_state.true_airspeed += state_derivative.true_airspeed_deriv * dt;
   m_equations_of_motion_state.gamma += state_derivative.gamma_deriv * dt;
   m_equations_of_motion_state.psi += state_derivative.heading_deriv * dt;
   m_equations_of_motion_state.thrust += state_derivative.thrust_deriv * dt;
   m_equations_of_motion_state.phi += state_derivative.roll_rate * dt;
   m_equations_of_motion_state.speedBrake += state_derivative.speed_brake_deriv * dt.value();
   m_equations_of_motion_state.flapConfig = state_derivative.flap_configuration;

   // States
   m_dynamics_state.x = m_equations_of_motion_state.enu_x;
   m_dynamics_state.y = m_equations_of_motion_state.enu_y;
   m_dynamics_state.h = m_equations_of_motion_state.enu_z;
   m_dynamics_state.V = m_equations_of_motion_state.true_airspeed;
   m_dynamics_state.gamma = m_equations_of_motion_state.gamma; // flight-path angle NOTE: for gamma, down is
   // positive; up is negative
   m_dynamics_state.psi = m_equations_of_motion_state.psi; // heading angle measured from east counter-clockwise (rad)
   m_dynamics_state.thrust = m_equations_of_motion_state.thrust;
   m_dynamics_state.phi = m_equations_of_motion_state.phi;
   m_dynamics_state.speed_brake = m_equations_of_motion_state.speedBrake;      // speed brake (% of deployment)
   m_dynamics_state.m_flap_configuration = static_cast<int>(m_equations_of_motion_state.flapConfig + 0.1);

   // Use xdot and ydot from Dynamics to ensure winds are used
   Units::Speed xdot = state_derivative.enu_velocity_x;  //ground speed (m/s)
   Units::Speed ydot = state_derivative.enu_velocity_y;  //ground speed (m/s)

   m_dynamics_state.xd = xdot;
   m_dynamics_state.yd = ydot;

   // Check Thrust Limits and Limit Appropriately
   m_dynamics_state.v_cas = m_true_weather.getAtmosphere()->TAS2CAS(Units::MetersPerSecondSpeed(m_dynamics_state.V),
                                                                    Units::MetersLength(m_dynamics_state.h));

   double cd0, cd2;
   int mode;
   double gear;
   m_bada_calculator.getConfig(m_dynamics_state.v_cas,
                               Units::MetersLength(m_dynamics_state.h),
                               m_altitude_msl_at_final_waypoint,
                               m_dynamics_state.m_flap_configuration,
                               cd0, cd2, gear, mode);
   Units::Force max_thrust = Units::NewtonsForce(
         m_bada_calculator.getMaxThrust(Units::MetersLength(m_dynamics_state.h), mode, "cruise"));
   Units::Force min_thrust = Units::NewtonsForce(
         m_bada_calculator.getMaxThrust(Units::MetersLength(m_dynamics_state.h), mode, "descent"));

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
   Units::Speed Vw_para = m_wind_velocity_x * cos(trk) + m_wind_velocity_y * sin(trk);
   Units::Speed Vw_perp = -m_wind_velocity_x * sin(trk) + m_wind_velocity_y * cos(trk);

   // assign return values.
   result_state.m_x = Units::FeetLength(m_dynamics_state.x).value() ;
   result_state.m_y = Units::FeetLength(m_dynamics_state.y).value();
   result_state.m_z = Units::FeetLength(m_dynamics_state.h).value();
   result_state.SetPsi(m_dynamics_state.psi);
   result_state.m_xd = Units::FeetPerSecondSpeed(xdot).value();
   result_state.m_yd = Units::FeetPerSecondSpeed(ydot).value();
   result_state.SetZd(-Units::FeetPerSecondSpeed(m_dynamics_state.V).value() * sin(m_dynamics_state.gamma));
   // Note: for gamma, heading down is positive
   result_state.m_gamma = Units::RadiansAngle(m_dynamics_state.gamma).value();
   result_state.m_Vwx = Units::MetersPerSecondSpeed(m_wind_velocity_x).value();
   result_state.m_Vwy = Units::MetersPerSecondSpeed(m_wind_velocity_y).value();
   result_state.m_Vw_para = Units::MetersPerSecondSpeed(Vw_para).value();
   result_state.m_Vw_perp = Units::MetersPerSecondSpeed(Vw_perp).value();
   result_state.m_Vwx_dh = dVwx_dh;
   result_state.m_Vwy_dh = dVwy_dh;

   return result_state;
}

void ThreeDOFDynamics::Initialize(const double mass_percentile,
                                  Units::Length altitude_msl_at_faf,
                                  Units::Speed initial_tas,
                                  std::shared_ptr<TangentPlaneSequence> tangent_plane_sequence,
                                  const EarthModel::LocalPositionEnu &initial_enu_position,
                                  const Units::Angle initial_heading,
                                  const WeatherTruth &true_weather) {
   this->m_tangent_plane_sequence = std::move(tangent_plane_sequence);
   m_true_weather = true_weather;
   m_altitude_msl_at_final_waypoint = Units::MetersLength(altitude_msl_at_faf);

   if (m_altitude_msl_at_final_waypoint < Units::MetersLength(0.0)) {
      LOG4CPLUS_WARN(m_logger, "WARNING: FAF altitude " << Units::FeetLength(m_altitude_msl_at_final_waypoint) << " < 0." << endl
                                                        << "Probably was not initialized in precalculated trajectory.");
   }

   m_bada_calculator.getAircraftParameters(m_ac_type, mass_percentile);
   m_bada_calculator.setFlapSpeeds(m_ac_type);

   m_dynamics_state.x = initial_enu_position.x;
   m_dynamics_state.y = initial_enu_position.y;
   m_dynamics_state.h = initial_enu_position.z;
   m_dynamics_state.psi = initial_heading;  // Using psi from the next waypoint.
   m_dynamics_state.gamma = Units::RadiansAngle(0);
   m_dynamics_state.phi = Units::RadiansAngle(0);
   m_dynamics_state.V = initial_tas;

   // Actually initialize the state
   m_equations_of_motion_state.enu_x = Units::MetersLength(m_dynamics_state.x); // east (m)
   m_equations_of_motion_state.enu_y = Units::MetersLength(m_dynamics_state.y); // north (m)
   m_equations_of_motion_state.enu_z = Units::MetersLength(m_dynamics_state.h); // altitude (m)
   m_equations_of_motion_state.true_airspeed = Units::MetersPerSecondSpeed(m_dynamics_state.V); // true airspeed (m/s)
   m_equations_of_motion_state.gamma = Units::RadiansAngle(m_dynamics_state.gamma); // flight-path angle (rad); NOTE:
   // for gamma, heading down is positve
   m_equations_of_motion_state.thrust = Units::NewtonsForce(0.0); // thrust (N)
   m_equations_of_motion_state.phi = Units::RadiansAngle(m_dynamics_state.phi); // roll angle (rad)
   m_equations_of_motion_state.speedBrake = 0.0; // speed brake (% of deployment)
   m_equations_of_motion_state.flapConfig = 0;   // flap config

   // Now that the initial state has been determined, still need to trim laterally for wind
   m_equations_of_motion_state.psi = CalculateTrimmedPsiForWind(m_dynamics_state.psi); // this also sets Vwx & Vwy class members
   m_dynamics_state.psi = m_equations_of_motion_state.psi; // heading angle measured from east counter-clockwise (rad)

   // Determine Groundspeed
   m_dynamics_state.xd = m_dynamics_state.V * cos(m_dynamics_state.psi) * cos(m_dynamics_state.gamma) +
                         m_wind_velocity_x;
   m_dynamics_state.yd = m_dynamics_state.V * sin(m_dynamics_state.psi) * cos(m_dynamics_state.gamma) +
                         m_wind_velocity_y;

   // Calculate initial Aircraft Thrust
   double cd0, cd2;
   int mode;
   double gear;
   m_bada_calculator.getConfig(m_true_weather.getAtmosphere()->TAS2CAS(Units::MetersPerSecondSpeed(m_dynamics_state.V),
                                                                       Units::MetersLength(m_dynamics_state.h)),
                               Units::MetersLength(m_dynamics_state.h),
                               m_altitude_msl_at_final_waypoint,
                               0, cd0, cd2, gear, mode);
   Units::Mass ac_mass = m_bada_calculator.mAircraftMass;

   Units::Force D, L;
   CalculateKineticForces(L, D);

   Units::Force Tnom = D - ac_mass * Units::ONE_G_ACCELERATION * sin(asin(0.0));
   Units::Force maxThrust = Units::NewtonsForce(m_bada_calculator.getMaxThrust(Units::MetersLength(m_dynamics_state.h)));
   Units::Force minThrust = Units::NewtonsForce(
         m_bada_calculator.getMaxThrust(Units::MetersLength(m_dynamics_state.h), mode, "descent"));
   if (Tnom > maxThrust * m_max_thrust_percent) {
      Tnom = maxThrust * m_max_thrust_percent;
   } else if (Tnom < minThrust * m_min_thrust_percent) {
      Tnom = minThrust * m_min_thrust_percent;
   }

   m_equations_of_motion_state.thrust = Tnom;
}

bool ThreeDOFDynamics::load(DecodedStream *input) {
   bool results;

   set_stream(input);

   // register the aircraft bada type
   LoaderDeprecatedMetaInfo acTypeInfo = {true, "Please use: Aircraft -> ac_type"};
   register_var("ac_type", &m_ac_type, acTypeInfo);

   LoaderDeprecatedMetaInfo bankangleTypeInfo = {true, "Please use: fms -> max_bank_angle"};
   Units::DegreesAngle maxbankangleload(-1.0);
   register_var("max_bank_angle", &maxbankangleload, bankangleTypeInfo);

   LoaderDeprecatedMetaInfo speed_management_type_info = {true, "Please use: aircraft -> speed_management_type in FMACM"};
   string dummy_speed_management_type;
   register_var("speed_management_type", &dummy_speed_management_type, speed_management_type_info);

   LoaderDeprecatedMetaInfo env_csv_file_info = {true, "Please use: aircraft -> env_csv_file in FMACM"};
   string dummy_env_csv_file;
   register_var("env_csv_file", &dummy_env_csv_file, env_csv_file_info);

   //do the actual reading:
   results = complete();

   // done
   m_model_loaded = results;

   return results;
}

const string &ThreeDOFDynamics::GetBadaAcType() const {
   return m_ac_type;
}

void ThreeDOFDynamics::SetBadaAcType(const string &acType) {
   m_ac_type = acType;
}

string ThreeDOFDynamics::GetTrueWindAsCsvString(const WindStack &wind_x,
                                                const WindStack &wind_y) {
   // Forms string of wind_x, wind_y data.  String includes a header as well
   // the data under it.
   //
   // wind_x,wind_y:x,y data in array where first column is altitude (feet) and the
   //               second column is the wind (knots).
   //
   // returns string of all values, row by row with altitude (meters) and wind
   //               (meters/second).

   string str = "AltX(m),windX(mps),AltY(m),windY(mps)\n";

   char *txt = new char[101];

   for (int i = wind_x.GetMinRow(); i <= wind_x.GetMaxRow(); i++) {
      sprintf(txt, "%lf,%lf,%lf,%lf\n",
              Units::MetersLength(wind_x.GetAltitude(i)).value(),
              Units::MetersPerSecondSpeed(wind_x.GetSpeed(i)).value(),
              Units::MetersLength(wind_y.GetAltitude(i)).value(),
              Units::MetersPerSecondSpeed(wind_y.GetSpeed(i)).value());
      str += txt;
   }

   delete[] txt;

   return str;
}

EquationsOfMotionStateDeriv ThreeDOFDynamics::StatePropagation(const Units::Frequency dVwx_dh,
                                                               const Units::Frequency dVwy_dh,
                                                               const Units::Frequency k_gamma,
                                                               const Units::Frequency k_t,
                                                               const Units::Frequency k_phi,
                                                               const double k_speedBrake,
                                                               const ControlCommands commands) {

   // Aircraft Configuration
   const Units::Mass ac_mass = m_bada_calculator.mAircraftMass;

   // States
   const Units::Speed V = m_equations_of_motion_state.true_airspeed; // true airspeed (m/s)
   const Units::Angle gamma = m_equations_of_motion_state.gamma;     // flight-path angle (rad)
   const Units::Angle psi = m_equations_of_motion_state.psi;         // heading angle measured from east counter-clockwise (rad)
   const Units::Force thrust = m_equations_of_motion_state.thrust;   // thrust (N)
   const Units::Angle phi = m_equations_of_motion_state.phi;         // roll angle (rad)
   const double speedBrake = m_equations_of_motion_state.speedBrake; // speed brake (% of deployment)

   Units::Force drag, lift;
   CalculateKineticForces(lift, drag);

   // calculate the first-order derivative of the state vector
   EquationsOfMotionStateDeriv dX;
   dX.enu_velocity_x = Units::MetersPerSecondSpeed(V * cos(gamma) * cos(psi) + m_wind_velocity_x);
   dX.enu_velocity_y = Units::MetersPerSecondSpeed(V * cos(gamma) * sin(psi) + m_wind_velocity_y);
   dX.enu_velocity_z = Units::MetersPerSecondSpeed(-V * sin(gamma));
   dX.true_airspeed_deriv = (thrust - drag) / ac_mass + Units::ONE_G_ACCELERATION * sin(gamma)
                            + V * (dVwx_dh * cos(psi) + dVwy_dh * sin(psi))
                              * sin(gamma) * cos(gamma);
   dX.gamma_deriv = k_gamma * (commands.getGamma() - gamma) -
                    (dVwx_dh * cos(psi) + dVwy_dh * sin(psi)) * pow(sin(gamma), 2) * Units::ONE_RADIAN_ANGLE;
   dX.heading_deriv = (-lift * sin(phi) / (ac_mass * V * cos(gamma)) -
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
   const Units::Mass ac_mass = m_bada_calculator.mAircraftMass;
   const Units::Area wing_area = m_bada_calculator.aerodynamics.S;
   int flapConfig = (int) (m_equations_of_motion_state.flapConfig + 0.1); // flap configuration

   // States important for this method
   const Units::Length h = m_equations_of_motion_state.enu_z;           // aircraft altitude (m)
   const Units::Speed V = m_equations_of_motion_state.true_airspeed;           // true airspeed (m/s)
   const Units::Angle phi = m_equations_of_motion_state.phi;         // roll angle (rad)
   double speedBrake = m_equations_of_motion_state.speedBrake;  // speed brake (% of deployment)

   // Calculate forces
   // Get temp, density, and pressure
   Units::KilogramsMeterDensity rho(m_true_weather.GetDensity());
   Units::PascalsPressure pressure(m_true_weather.GetPressure());

   // Get m_bada_calculator Configuration
   double cd0, cd2, gear;
   int flapConfig_new;
   m_dynamics_state.v_cas = m_true_weather.getAtmosphere()->TAS2CAS(V, pressure, rho); // current indicated airspeed in meters per second
   m_bada_calculator.getConfig(m_dynamics_state.v_cas,
                               h, m_altitude_msl_at_final_waypoint,
                               flapConfig,
                               cd0, cd2, gear, flapConfig_new);

   // Lift and Drag Coefficients
   const double cL = (2. * ac_mass * Units::ONE_G_ACCELERATION) / (rho * Units::sqr(V) * wing_area * cos(phi));
   double cD = cd0 + gear + cd2 * pow(cL, 2); // not const because it an be modified below
   if (speedBrake != 0.0) {
      cD = (1.0 + 0.6 * speedBrake) * cD;
   }

   // Drag & Lift
   drag = 1. / 2. * rho * cD * Units::sqr(V) * wing_area;
   lift = 1. / 2. * rho * cL * Units::sqr(V) * wing_area;
}

void ThreeDOFDynamics::CalculateEnvironmentalWind(WindStack &windstack_x,
                                                  WindStack &windstack_y,
                                                  Units::Frequency &dVwx_dh,
                                                  Units::Frequency &dVwy_dh) {

   EarthModel::GeodeticPosition geoPosition;

   // do lat/lon conversion only if using wind
   if (Wind::UseWind()) {
      EarthModel::LocalPositionEnu localPosition;
      localPosition.x = m_equations_of_motion_state.enu_x;
      localPosition.y = m_equations_of_motion_state.enu_y;
      localPosition.z = m_equations_of_motion_state.enu_z;
      m_tangent_plane_sequence->convertLocalToGeodetic(localPosition, geoPosition);
   }

   m_true_weather.LoadConditionsAt(geoPosition.latitude, geoPosition.longitude, m_equations_of_motion_state.enu_z);
   windstack_x = m_true_weather.east_west;
   windstack_y = m_true_weather.north_south;


   if (InternalObserver::getInstance()->debugTrueWind()) {
      InternalObserver::getInstance()->writeTrueWind(ThreeDOFDynamics::GetTrueWindAsCsvString(windstack_x, windstack_y));
   }

   // Get Winds and Wind Gradients at altitude
   m_true_weather.getAtmosphere()->CalculateWindGradientAtAltitude(m_equations_of_motion_state.enu_z, windstack_x,
                                                                   m_wind_velocity_x, dVwx_dh);
   m_true_weather.getAtmosphere()->CalculateWindGradientAtAltitude(m_equations_of_motion_state.enu_z, windstack_y,
                                                                   m_wind_velocity_y, dVwy_dh);
}

Units::UnsignedRadiansAngle ThreeDOFDynamics::CalculateTrimmedPsiForWind(Units::Angle trk) {
   // Get wind information

   WindStack windstackx(1, 5);
   WindStack windstacky(1, 5);
   Units::Frequency dvwxdh, dvwydh;
   CalculateEnvironmentalWind(windstackx, windstacky, dvwxdh, dvwydh);

   const double gamma = Units::RadiansAngle(m_equations_of_motion_state.gamma).value();
   const double trkRad = Units::RadiansAngle(AircraftCalculations::Convert0to2Pi(trk)).value();
   Units::MetersPerSecondSpeed vwpara = Units::MetersPerSecondSpeed(
         Units::MetersPerSecondSpeed(m_wind_velocity_x).value() * cos(trkRad) +
         Units::MetersPerSecondSpeed(m_wind_velocity_y).value() * sin(trkRad));
   Units::MetersPerSecondSpeed vwperp = Units::MetersPerSecondSpeed(
         -Units::MetersPerSecondSpeed(m_wind_velocity_x).value() * sin(trkRad) +
         Units::MetersPerSecondSpeed(m_wind_velocity_y).value() * cos(trkRad));

   Units::MetersPerSecondSpeed w = sqrt(Units::sqr(Units::MetersPerSecondSpeed(m_wind_velocity_x)) +
                                        Units::sqr(Units::MetersPerSecondSpeed(m_wind_velocity_y)));

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

   return AircraftCalculations::Convert0to2Pi(trk + beta);
}

