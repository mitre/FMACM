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

#include "public/USStandardAtmosphere1976.h"
#include "public/Logging.h"

const Units::KelvinTemperature TEMPERATURE_TOLERANCE(0.1);

/** Lowest altitude supported in US Standard Atmosphere */
const Units::MetersLength MINIMUM_ALTITUDE(-5000);

/** Sea-level constants, from US Standard Atmosphere table */
const Units::KelvinTemperature T0(288.15);

/** Bottom-of-tropopause constants, from US Standard Atmosphere table */
const Units::MetersLength H_TROP(11000);
const Units::KelvinTemperature T_TROP(216.65);
const Units::PascalsPressure P_TROP(22632);
const Units::KilogramsMeterDensity RHO_TROP(0.36392);

log4cplus::Logger USStandardAtmosphere1976::m_logger =
      log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("USStandardAtmosphere1976"));

const double USStandardAtmosphere1976::P_T_EXPONENT(std::log(P_TROP / P0_ISA) / std::log(T_TROP / T0));
const double USStandardAtmosphere1976::RHO_T_EXPONENT(std::log(RHO_TROP / RHO0_ISA) / std::log(T_TROP / T0));

USStandardAtmosphere1976::USStandardAtmosphere1976() { Atmosphere::SetTemperatureOffset(Units::CelsiusTemperature(0)); }
USStandardAtmosphere1976::USStandardAtmosphere1976(const Units::Temperature temperature_offset) {
   Atmosphere::SetTemperatureOffset(temperature_offset);
}

Atmosphere *USStandardAtmosphere1976::Clone() const { return new USStandardAtmosphere1976(); }

void USStandardAtmosphere1976::SetTemperatureOffset(const Units::Temperature temperature_offset) {
   if (Units::abs(temperature_offset) < TEMPERATURE_TOLERANCE) {
      LOG4CPLUS_WARN(m_logger,
                     "Temperature offset is not supported in USStandardAtmosphere1976, but the specified value of "
                           << Units::KelvinTemperature(temperature_offset) << " is within tolerance.");
   } else {
      std::string error("Temperature offset is not supported in USStandardAtmosphere1976.");
      LOG4CPLUS_FATAL(m_logger, error);
      throw std::runtime_error(error);
   }
}

void USStandardAtmosphere1976::CalibrateTemperatureAtAltitude(const Units::KelvinTemperature temperature,
                                                              const Units::Length altitude) {

   Units::KelvinTemperature difference = GetTemperature(altitude) - temperature;
   if (Units::abs(difference) <= TEMPERATURE_TOLERANCE) {
      LOG4CPLUS_WARN(m_logger,
                     "Calibration is not supported in USStandardAtmosphere1976, but the specified difference of "
                           << difference << " is within tolerance.");
   } else {
      std::string error("Calibration is not supported in USStandardAtmosphere1976.");
      LOG4CPLUS_FATAL(m_logger, error);
      throw std::runtime_error(error);
   }
}

void USStandardAtmosphere1976::AirDensity(const Units::Length h, Units::Density &rho, Units::Pressure &P) const {

   Units::KelvinTemperature T = GetTemperature(h);
   if (h < H_TROP) {
      // troposphere
      rho = RHO0_ISA * std::pow(T / T0, RHO_T_EXPONENT);
      P = P0_ISA * std::pow(T / T0, P_T_EXPONENT);
   } else {
      // tropopause
      const double factor(exp(-Units::ONE_G_ACCELERATION / (R * T_TROP) * (h - H_TROP)));
      P = GetTropopausePressure() * factor;
      rho = GetTropopauseDensity() * factor;
   }
   AirDensity_Log(h, T, P, rho);
}

Units::KelvinTemperature USStandardAtmosphere1976::GetTemperature(const Units::Length altitude_msl) const {

   if (altitude_msl < MINIMUM_ALTITUDE) {
      std::ostringstream s;
      s << "Lowest supported altitude is " << MINIMUM_ALTITUDE;
      std::string error(s.str());
      LOG4CPLUS_FATAL(m_logger, error);
      throw std::runtime_error(error);
   }

   Units::KelvinTemperature result;

   if (altitude_msl < H_TROP) {
      result = T0 + altitude_msl * K_T;
   } else {
      result = T_TROP;
   }
   return result;
}

Units::KelvinTemperature USStandardAtmosphere1976::GetSeaLevelTemperature() const { return T0; }

Units::Density USStandardAtmosphere1976::GetSeaLevelDensity() const { return RHO0_ISA; }

Units::MetersLength USStandardAtmosphere1976::GetTropopauseHeight() const { return H_TROP; }

Units::Density USStandardAtmosphere1976::GetTropopauseDensity() const { return RHO_TROP; }

Units::Pressure USStandardAtmosphere1976::GetTropopausePressure() const { return P_TROP; }

Units::Speed USStandardAtmosphere1976::CAS2TAS(const Units::Speed vcas, const Units::Pressure p,
                                               const Units::Density rho) const {
   // https://ppla.education/navcomp/Calculation_of_TAS_from_CAS-Correction_of_Density_Error/
   // CAS = TAS √relative density
   double relative_density(rho / GetSeaLevelDensity());
   Units::Speed vtas = vcas / sqrt(relative_density);
   return vtas;
}

Units::Speed USStandardAtmosphere1976::TAS2CAS(const Units::Speed vtas, const Units::Pressure p,
                                               const Units::Density rho) const {
   // https://ppla.education/navcomp/Calculation_of_TAS_from_CAS-Correction_of_Density_Error/
   // CAS = TAS √relative density
   double relative_density(rho / GetSeaLevelDensity());
   Units::Speed vcas = vtas * sqrt(relative_density);
   return vcas;
}

Units::Speed USStandardAtmosphere1976::SpeedOfSound(Units::KelvinTemperature temperature) const {
   // from https://en.wikipedia.org/wiki/Speed_of_sound#Speed_of_sound_in_ideal_gases_and_air
   Units::KnotsSpeed speed_of_sound = sqrt(GAMMA * R * temperature);
   return speed_of_sound;
}

double USStandardAtmosphere1976::ESFconstantCAS(const Units::Speed true_airspeed, const Units::Length altitude_msl,
                                                const Units::KelvinTemperature temperature) const {
   std::string error("USStandardAtmosphere1976::ESFconstantCAS is not implemented.");
   LOG4CPLUS_FATAL(m_logger, error);
   throw std::runtime_error(error);
}

Units::Length USStandardAtmosphere1976::GetMachIASTransition(const Units::Speed ias, const double mach) const {

   // Find the altitude at which CAS2TAS matches Mach2TAS (mach * SpeedOfSound)
   Units::MetersLength h(H_TROP);  // Only altitude at which the derivative is discontinuous, start there

   // Use Newton's Method to find the zero for
   // f(h) = mach * SpeedOfSound(h) - CAS2TAS(h)
   Units::KnotsSpeed f = mach * Atmosphere::SpeedOfSound(h) - Atmosphere::CAS2TAS(ias, h);
   const bool below_trop(f < Units::zero());

   int max_iterations(100);
   for (int i = 1; i <= max_iterations; ++i) {
      Units::HertzFrequency f_prime(0);
      if (below_trop) {
         f_prime = mach * K_T * sqrt(GAMMA * R / (T0 + h * K_T));
         double temp = 1 + h * K_T / T0;
         f_prime -= ias * K_T / T0 * (-RHO_T_EXPONENT / 2) * std::pow(temp, -RHO_T_EXPONENT / 2 - 1);
      } else {
         f_prime = Units::HertzFrequency(0);
         const Units::MetersSecondAcceleration G(Units::ONE_G_ACCELERATION);
         const double density_ratio(RHO0_ISA / RHO_TROP);
         f_prime -= ias * sqrt(density_ratio) * G / 2 / (R * T_TROP) * std::exp(G / 2 / (R * T_TROP) * (h - H_TROP));
      }

      h -= f / f_prime;
      f = mach * Atmosphere::SpeedOfSound(h) - Atmosphere::CAS2TAS(ias, h);

      static const auto CONVERGENCE_TOLERANCE{Units::KnotsSpeed(1e-4)};
      if (Units::abs(f) < CONVERGENCE_TOLERANCE) break;
   }

   return h;
}
