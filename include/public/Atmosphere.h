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

#pragma once

#include "Length.h"
#include "Speed.h"
#include "Temperature.h"
#include "Density.h"
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

class Atmosphere
{
public:
   Atmosphere();

   virtual ~Atmosphere();

   virtual Units::KelvinTemperature GetTemperature(const Units::Length altitude_msl) const = 0;

   void AirDensity(const Units::Length h,
                   Units::Density &rho,
                   Units::Pressure &P) const;

   void CalculateWindGradientAtAltitude(const Units::Length altitude_in,
                                        const WindStack &wind_stack,
                                        Units::Speed &wind_speed,
                                        Units::Frequency &wind_gradient) const;

   static Units::Speed CAS2TAS(const Units::Speed vcas, const Units::Pressure p, const Units::Density rho);

   Units::Speed CAS2TAS(const Units::Speed vcas,
                        const Units::Length alt) const;

   static Units::Speed TAS2CAS(const Units::Speed vtas, const Units::Pressure p, const Units::Density rho);

   Units::Speed TAS2CAS(const Units::Speed vtas,
                        const Units::Length alt) const;

   virtual Units::Length GetMachIASTransition(const Units::Speed &ias,
                                      const double &mach) const;

   Units::Speed MachToIAS(const double mach,
                          const Units::Length alt) const;

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
   double ESFconstantCAS(const Units::Speed true_airspeed,
                                const Units::Length altitude_msl,
                                const Units::KelvinTemperature temperature);



private:
   static log4cplus::Logger m_logger;
};
