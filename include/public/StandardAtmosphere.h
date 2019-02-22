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

/**
 * StandardAtmosphere is an implementation of the ISA
 * (International Standard Atmosphere) concept, which
 * defines straightforward approximations for calculating
 * pressure, temperature, density, and viscosity as a
 * function of altitude.
 *
 * https://en.wikipedia.org/wiki/International_Standard_Atmosphere
 */

#pragma once

#include <public/Atmosphere.h>

class StandardAtmosphere : public Atmosphere
{
public:
   StandardAtmosphere(const Units::Temperature temperatureOffset);

   virtual ~StandardAtmosphere();

   /**
    * Calculates the temperature at a given altitude MSL.
    *
    * Note:  Only the Troposphere and Tropopause layers are implemented.
    * Calculations above 65,000 feet would require adding Stratosphere.
    *
    * @param altitude_msl
    */
   Units::KelvinTemperature GetTemperature(const Units::Length altitude_msl) const;

   Units::Temperature GetTemperatureOffset() const;

private:
   const Units::Temperature m_temperature_offset;
};

