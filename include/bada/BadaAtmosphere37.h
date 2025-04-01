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

/**
 * BadaAtmosphere37 is an implementation of the ISA
 * (International Standard Atmosphere) concept, as
 * extended by the BADA 3.7 User Manual.
 *
 * https://en.wikipedia.org/wiki/International_Standard_Atmosphere
 */

#pragma once

#include <bada/BadaAtmosphereCommon.h>

namespace aaesim {
namespace bada {

class BadaAtmosphere37 final : public BadaAtmosphereCommon {
  public:
   BadaAtmosphere37(const Units::Temperature temperatureOffset);

   virtual ~BadaAtmosphere37() = default;

   Atmosphere *Clone() const;

   void SetTemperatureOffset(Units::Temperature temperature_offset);

   void CalibrateTemperatureAtAltitude(const Units::KelvinTemperature temperature, const Units::Length altitude);

   void AirDensity(const Units::Length h, Units::Density &rho, Units::Pressure &P) const;

   /**
    * Calculates the temperature at a given altitude MSL.
    *
    * Note:  Only the Troposphere and Tropopause layers are implemented.
    * Calculations above 65,000 feet would require adding Stratosphere.
    *
    * @param altitude_msl
    */
   Units::KelvinTemperature GetTemperature(const Units::Length altitude_msl) const;

   Units::KelvinTemperature GetSeaLevelTemperature() const;
   Units::Density GetSeaLevelDensity() const;
   Units::MetersLength GetTropopauseHeight() const;
   Units::Density GetTropopauseDensity() const;
   Units::Pressure GetTropopausePressure() const;

  private:
   static log4cplus::Logger m_logger;
   Units::MetersLength m_tropopause_height;
   Units::Temperature m_sea_level_temperature;
   Units::Density m_sea_level_density;
   Units::Density m_tropopause_density;
   Units::Pressure m_tropopause_pressure;
};

}  // namespace bada
}  // namespace aaesim
