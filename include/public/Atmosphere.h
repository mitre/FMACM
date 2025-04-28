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

#pragma once

#include "utility/CustomUnits.h"
#include <scalar/Length.h>
#include <scalar/Speed.h>
#include <scalar/Temperature.h>
#include <scalar/Density.h>
#include <scalar/Pressure.h>
#include "public/WindStack.h"
#include <log4cplus/logger.h>

// Standard air pressure at sea level
const Units::PascalsPressure P0_ISA(101325.);

// Standard Density at Sea Level
// BADA_37_USER_MANUAL eq. 3.2-8
const Units::KilogramsMeterDensity RHO0_ISA(1.225);

// Speed of Sound at sea level
const Units::MetersPerSecondSpeed A0(340.29);

// Isentropic expansion coefficient for air
constexpr double GAMMA = 1.4;

// mu
constexpr double MU = ((GAMMA - 1) / (GAMMA));

// Real gas constant (m^2/K-s^2)
const Units::MetersSecondsKelvinGasConstant R(287.05287);

// Temperature gradient (K/m)
const Units::KelvinPerMeter K_T(-0.0065);

class Atmosphere {
  public:
   enum AtmosphereType { UNKNOWN = 0, BADA37, BADA312, BADA316 };

   Atmosphere() {}

   virtual ~Atmosphere() = default;

   virtual Atmosphere *Clone() const = 0;

   virtual void SetTemperatureOffset(const Units::Temperature temperature_offset) {
      m_temperature_offset = temperature_offset;
   }

   Units::CelsiusTemperature GetTemperatureOffset() const { return m_temperature_offset; }

   /**
    * Calculate the temperature offset which will yield the specified temperature
    * at the specified altitude and then call SetTemperatureOffset.
    */
   virtual void CalibrateTemperatureAtAltitude(const Units::KelvinTemperature temperature,
                                               const Units::Length altitude) = 0;

   virtual Units::KelvinTemperature GetTemperature(const Units::Length altitude_msl) const = 0;

   virtual void AirDensity(const Units::Length h, Units::Density &rho, Units::Pressure &P) const = 0;

   virtual Units::Speed CAS2TAS(const Units::Speed vcas, const Units::Pressure p, const Units::Density rho) const = 0;

   Units::Speed CAS2TAS(const Units::Speed vcas, const Units::Length alt) const {
      // Get the air density
      Units::KilogramsMeterDensity rho;
      Units::Pressure p;

      AirDensity(alt, rho, p);

      Units::Speed vtas = CAS2TAS(vcas, p, rho);
      return vtas;
   }

   virtual Units::Speed TAS2CAS(const Units::Speed vtas, const Units::Pressure p, const Units::Density rho) const = 0;

   Units::Speed TAS2CAS(const Units::Speed vtas, const Units::Length alt) const {
      // Get the air density
      Units::KilogramsMeterDensity rho;
      Units::Pressure p;

      AirDensity(alt, rho, p);

      Units::Speed vcas = TAS2CAS(vtas, p, rho);
      return vcas;
   }

   virtual Units::Length GetMachIASTransition(const Units::Speed ias, const double mach) const = 0;

   Units::Speed MachToIAS(const double mach, const Units::Length alt) const {
      Units::Speed tas = mach * SpeedOfSound(alt);

      return TAS2CAS(tas, alt);
   }

   double IASToMach(const Units::Speed ias, const Units::Length alt) const {
      Units::Speed tas = CAS2TAS(ias, alt);

      return tas / SpeedOfSound(alt);
   }

   virtual Units::Speed SpeedOfSound(Units::KelvinTemperature temperature) const = 0;

   Units::Speed SpeedOfSound(Units::Length altitude) const {
      Units::KelvinTemperature temperature = GetTemperature(altitude);
      return SpeedOfSound(temperature);
   }

   virtual Units::KelvinTemperature GetSeaLevelTemperature() const = 0;

   virtual Units::MetersLength GetTropopauseHeight() const = 0;
   virtual Units::Pressure GetTropopausePressure() const = 0;

   /**
    * Calculate the Calibrated Airspeed BADA Energy Share Factor
    */
   virtual double ESFconstantCAS(const Units::Speed true_airspeed, const Units::Length altitude_msl,
                                 const Units::KelvinTemperature temperature) const = 0;

  protected:
   Units::Temperature m_temperature_offset;

   void AirDensity_Log(const Units::MetersLength h, const Units::KelvinTemperature t, const Units::PascalsPressure p,
                       const Units::KilogramsMeterDensity rho) const;

  private:
   static log4cplus::Logger m_logger;
};
