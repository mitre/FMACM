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
   static StandardAtmosphere *MakeInstance(const Units::KelvinTemperature temperature, const Units::Length altitude);

   static StandardAtmosphere *MakeInstanceFromTemperatureOffset(Units::CelsiusTemperature temperature_offset);

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

   Units::KelvinTemperature GetSeaLevelTemperature() const;
   Units::Density GetSeaLevelDensity() const;
   Units::MetersLength GetTropopauseHeight() const;
   Units::Density GetTropopauseDensity() const;
   Units::Pressure GetTropopausePressure() const;

protected:
   void SetTemperatureOffset(Units::Temperature temperature_offset);

private:
   static log4cplus::Logger m_logger;
   Units::Temperature m_temperature_offset;
   Units::MetersLength m_tropopause_height;
   Units::Temperature m_sea_level_temperature;
   Units::Density m_sea_level_density;
   Units::Density m_tropopause_density;
   Units::Pressure m_tropopause_pressure;
};

