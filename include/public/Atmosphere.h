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

#include <scalar/Length.h>
#include <scalar/Speed.h>
#include <scalar/Temperature.h>
#include <scalar/Density.h>
#include "utility/CustomUnits.h"
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

// exponent for eq. 3.2-15, about 5.25583
const double P_T_EXPONENT(-Units::ONE_G_ACCELERATION / (K_T * R));

// exponent for eq. 3.2-6, about 4.25583
const double RHO_T_EXPONENT(P_T_EXPONENT - 1);

class Atmosphere {
  public:
   Atmosphere();

   virtual ~Atmosphere();

   virtual Units::KelvinTemperature GetTemperature(const Units::Length altitude_msl) const = 0;

   void AirDensity(const Units::Length h, Units::Density &rho, Units::Pressure &P) const;

   void CalculateWindGradientAtAltitude(const Units::Length altitude_in,
                                        const aaesim::open_source::WindStack &wind_stack, Units::Speed &wind_speed,
                                        Units::Frequency &wind_gradient) const;

   static Units::Speed CAS2TAS(const Units::Speed vcas, const Units::Pressure p, const Units::Density rho);

   Units::Speed CAS2TAS(const Units::Speed vcas, const Units::Length alt) const;

   static Units::Speed TAS2CAS(const Units::Speed vtas, const Units::Pressure p, const Units::Density rho);

   Units::Speed TAS2CAS(const Units::Speed vtas, const Units::Length alt) const;

   virtual Units::Length GetMachIASTransition(const Units::Speed &ias, const double &mach) const;

   Units::Speed MachToIAS(const double mach, const Units::Length alt) const;

   static Units::Speed SpeedOfSound(Units::KelvinTemperature temperature);
   Units::Speed SpeedOfSound(Units::Length altitude) const;

   virtual Units::KelvinTemperature GetSeaLevelTemperature() const = 0;
   virtual Units::Density GetSeaLevelDensity() const = 0;

   virtual Units::MetersLength GetTropopauseHeight() const = 0;
   virtual Units::Density GetTropopauseDensity() const = 0;
   virtual Units::Pressure GetTropopausePressure() const = 0;

   /**
    * Calculate the Calibrated Airspeed BADA Energy Share Factor
    */
   double ESFconstantCAS(const Units::Speed true_airspeed, const Units::Length altitude_msl,
                         const Units::KelvinTemperature temperature);

  private:
   static log4cplus::Logger m_logger;
};
