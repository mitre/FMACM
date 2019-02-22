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

// atmosphere constants
// Standard Temperature at Sea Level
const Units::KelvinTemperature T0(288.15);

// Standard Density at Sea Level
const Units::KilogramsMeterDensity RHO0(1.225);

// Height of the tropopause
const Units::MetersLength H_TROP(11000.);

// Temperature of the tropopause
const Units::KelvinTemperature T_TROP(216.65);

// Density of the tropopause at 11,000 meters (kg/m^3)
const Units::KilogramsMeterDensity RHO_TROP(0.3639228);

// Pressure of the tropopause at 11,000 meters (Pa)
const Units::PascalsPressure P_TROP(22632.0);

// Standard air pressure at sea level
const Units::PascalsPressure P0(101325.);

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


class Atmosphere
{
public:
   Atmosphere();

   virtual ~Atmosphere();

   virtual Units::KelvinTemperature GetTemperature(const Units::Length altitude_msl) const = 0;

   void AirDensity(const Units::Length h,
                   Units::Density &rho,
                   Units::Pressure &P) const;

   void CalcWindGrad(const Units::Length h_star,
                     const WindStack &wind,
                     Units::Speed &w_dir,
                     Units::Frequency &w_dir_grad) const;

   Units::Speed CAS2TAS(const Units::Speed vcas,
                        const Units::Length alt) const;

   Units::Speed TAS2CAS(const Units::Speed vtas,
                        const Units::Length alt) const;

   // method to calculate the MACH to IAS transition
   Units::Length GetMachIASTransition(const Units::Speed &ias,
                                      const double &mach) const;

   // method to convert mach to ias-ias in meters per second.
   Units::Speed MachToIAS(const double mach,
                          const Units::Length alt) const;
};
