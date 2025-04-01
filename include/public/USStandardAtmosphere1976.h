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
 * USStandardAtmosphere1976 implements the United States Standard Atmosphere as revised in 1976.
 *
 * It uses the same value for sea-level temperature, pressure, and density as ISA, so we will
 * use
 */

#pragma once

#include <public/Atmosphere.h>

class USStandardAtmosphere1976 final : public Atmosphere {
  public:
   USStandardAtmosphere1976();

   virtual ~USStandardAtmosphere1976() = default;

   Atmosphere *Clone() const;

   void SetTemperatureOffset(const Units::Temperature temperature_offset);

   void CalibrateTemperatureAtAltitude(const Units::KelvinTemperature temperature, const Units::Length altitude);

   void AirDensity(const Units::Length h, Units::Density &rho, Units::Pressure &P) const;

   Units::KelvinTemperature GetTemperature(const Units::Length altitude_msl) const;

   Units::KelvinTemperature GetSeaLevelTemperature() const;

   Units::Density GetSeaLevelDensity() const;

   Units::MetersLength GetTropopauseHeight() const;

   Units::Density GetTropopauseDensity() const;

   Units::Pressure GetTropopausePressure() const;

   Units::Speed CAS2TAS(const Units::Speed vcas, const Units::Pressure p, const Units::Density rho) const;

   Units::Speed TAS2CAS(const Units::Speed vtas, const Units::Pressure p, const Units::Density rho) const;

   Units::Speed SpeedOfSound(Units::KelvinTemperature temperature) const;

   double ESFconstantCAS(const Units::Speed true_airspeed, const Units::Length altitude_msl,
                         const Units::KelvinTemperature temperature) const;

   Units::Length GetMachIASTransition(const Units::Speed ias, const double mach) const;

  private:
   static log4cplus::Logger m_logger;

   static const double P_T_EXPONENT;
   static const double RHO_T_EXPONENT;
};
