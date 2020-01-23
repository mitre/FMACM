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
// Copyright 2020 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#include <public/CoreUtils.h>
#include "public/AircraftControl.h"
#include "public/Environment.h"
#include "public/InternalObserver.h"

log4cplus::Logger AircraftControl::m_logger = log4cplus::Logger::getInstance("AircraftControl");

AircraftControl::AircraftControl()
      : m_speed_brake_gain(0.0),
        m_bada_calculator(nullptr) {
}

void AircraftControl::Initialize(BadaWithCalc &aircraftPerformance,
                                 const Units::Length &altAtFAF,
                                 const Units::Angle &maxBankAngle,
                                 const PrecalcWaypoint &finalWaypoint) {
   m_max_bank_angle = maxBankAngle;
   m_alt_at_FAF = altAtFAF;
   m_bada_calculator = &aircraftPerformance;

   m_ac_mass = m_bada_calculator->mAircraftMass;
   m_wing_area = m_bada_calculator->aerodynamics.S;
   m_final_waypoint = finalWaypoint;
}

ControlCommands AircraftControl::CalculateControlCommands(const Guidance &guidance,
                                                          const EquationsOfMotionState &eqmState,
                                                          const WeatherTruth &wind) {
   const Units::Angle phi = Units::ZERO_ANGLE;
   const Units::Force thrust = Units::ZERO_FORCE;
   const Units::Angle gamma = Units::ZERO_ANGLE;
   const Units::Speed trueAirspeed = Units::ZERO_SPEED;
   const double speedBrake = 0;
   const int flapMode = 0;

   return ControlCommands(phi, thrust, gamma, trueAirspeed, speedBrake, flapMode);
}

Units::Frequency AircraftControl::calculateThrustGain() {
   const double zeta = 0.88;
   m_natural_frequency = Units::HertzFrequency(0.20);
   const Units::Frequency thrustGain = 2 * zeta * m_natural_frequency; // new thrust gain, roughly .352
   return thrustGain;
}

void AircraftControl::estimateKineticForces(const EquationsOfMotionState &eqmState,
                                            Units::Force &lift,
                                            Units::Force &drag,
                                            int &newFlapConfiguration,
                                            const WeatherTruth &weather) {
   Units::Speed v_cas = weather.getAtmosphere()->TAS2CAS(Units::MetersPerSecondSpeed(eqmState.true_airspeed), Units::MetersLength(
         eqmState.enu_z)); // current indicated airspeed in meters per second

   // Get temp, density, and pressure
   Units::KilogramsMeterDensity rho;
   Units::Pressure P_tmp;
   weather.getAtmosphere()->AirDensity(eqmState.enu_z, rho, P_tmp);
   // Don't bother converting P_tmp from kg/m^2 because we don't need it.

   // Get AC Configuration
   double cd0, cd2;
   double gear;
   m_bada_calculator->getConfig(v_cas,
                            eqmState.enu_z,
                            m_alt_at_FAF,
                            eqmState.flapConfig + 0.1,
                            cd0,
                            cd2,
                            gear,
                            newFlapConfiguration);

   // Lift and Drag Estimate Calculations
   double cL =
         (2. * m_ac_mass * Units::ONE_G_ACCELERATION) / (rho * Units::sqr(eqmState.true_airspeed) * m_wing_area * cos(eqmState.phi));
   double cD = cd0 + gear + cd2 * pow(cL, 2);

   if (eqmState.speedBrake != 0.0) {
      cD = (1.0 + 0.6 * eqmState.speedBrake) * cD;
   }

   drag = 1. / 2. * rho * cD * Units::sqr(eqmState.true_airspeed) * m_wing_area;
   lift = 1. / 2. * rho * cL * Units::sqr(eqmState.true_airspeed) * m_wing_area;

}

/**
 * This default implementation of calculateSensedWind provides "perfect" knowledge by
 * calculating and providing the true environmental winds.
 *
 * Override this method to provide alternate implemenations.
 */
void AircraftControl::calculateSensedWind(const WeatherTruth &wind,
                                          const Units::MetersLength &altitude) {
   // Get Winds and Wind Gradients at altitude
   wind.getAtmosphere()->CalculateWindGradientAtAltitude(altitude, wind.east_west, m_Vwx, m_dVwx_dh);
   wind.getAtmosphere()->CalculateWindGradientAtAltitude(altitude, wind.north_south, m_Vwy, m_dVwy_dh);

}

Units::Angle AircraftControl::doLateralControl(const Guidance &guidance,
                                               const EquationsOfMotionState &eqmState) {
   const Units::InvertedLength k_xtrk = Units::PerMeterInvertedLength(5e-4);  // meters^-1
   const double k_trk = 3;      // unitless

   // States
   const Units::Length x = eqmState.enu_x;           // aircraft position east coordinate (m)
   const Units::Length y = eqmState.enu_y;           // aircraft position north coordinate (m)
   const Units::Speed V = eqmState.true_airspeed;           // true airspeed (m/s)
   const Units::Angle gamma = eqmState.gamma;       // flight-path angle (rad)
   const Units::Angle psi = eqmState.psi;         // heading angle measured from east counter-clockwise (rad)

   // Commanded Track Angle
   Units::Angle trk = guidance.m_track_angle;

   Units::Speed Vw_para = m_Vwx * cos(trk) + m_Vwy * sin(trk);
   Units::Speed Vw_perp = -m_Vwx * sin(trk) + m_Vwy * cos(trk);

   Units::Speed W = sqrt(Units::sqr(m_Vwx) + Units::sqr(m_Vwy));
   Units::Speed gs = sqrt(Units::sqr(V * cos(gamma)) - Units::sqr(Vw_perp)) + Vw_para;

   double temp = (Units::sqr(V * cos(gamma)) + Units::sqr(gs) - Units::sqr(W)) / (V * 2 * cos(gamma) * gs);

   // Limit temp so acos function doesn't give undefined value.
   if (temp > 1.0) {
      temp = 1.0;
   } else if (temp < -1.0) {
      temp = -1.0;
   }

   Units::Angle beta =
         Units::RadiansAngle(acos(temp)) * -1.0 * CoreUtils::SignOfValue(Units::MetersPerSecondSpeed(Vw_perp).value());

   // Convert track guidance to heading using winds (beta is the Wind Correction Angle)
   Units::Angle headingCom = trk + beta;

   // Error in heading angle
   //double 	e_trk = subtract_headings(headingCom, psi);
   Units::SignedAngle e_trk = headingCom - psi;
   e_trk.normalize();

   // Along-path distance and Cross-track Error
   Units::Length e_xtrk = Units::MetersLength(0.0);
   double dynamic_cross = 1.0;

   // check if guidance has has cross track error and use it if so
   if (guidance.m_use_cross_track) {
      if (guidance.m_reference_bank_angle != Units::ZERO_ANGLE) {
         e_xtrk = guidance.m_cross_track_error - (guidance.m_reference_bank_angle / k_xtrk);
      } else {
         e_xtrk = guidance.m_cross_track_error;
      }
   }

   // Calculate commanded roll angle
   // We had to add a conversion from unitless to radians in the formula for phi_com.
   Units::Angle phi_com = -k_xtrk * e_xtrk * Units::ONE_RADIAN_ANGLE - k_trk * e_trk; // CONTROL
   double unlimited_phi_com = Units::RadiansAngle(phi_com).value();

   // Limit the commanded roll angle
   double sign_phi_com = CoreUtils::SignOfValue(unlimited_phi_com);
   if (phi_com * sign_phi_com > m_max_bank_angle) {
      phi_com = m_max_bank_angle * sign_phi_com;
   }

   InternalObserver::getInstance()->cross_output(Units::MetersLength(x).value(),
                                                 Units::MetersLength(y).value(),
                                                 dynamic_cross,
                                                 Units::MetersLength(guidance.m_cross_track_error).value(),
                                                 Units::RadiansAngle(guidance.m_track_angle).value(), unlimited_phi_com,
                                                 Units::RadiansAngle(phi_com).value());

   return phi_com;
}


