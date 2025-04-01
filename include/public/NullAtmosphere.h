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
 * NullAtmosphere is a placeholder class which throws logic_error for most
 * methods.  It should be replaced with an actual implementation of Atmosphere
 * before use.
 */

#pragma once

#include <public/Atmosphere.h>

class NullAtmosphere final : public Atmosphere {
  public:
   NullAtmosphere() {}

   NullAtmosphere(const Units::Temperature temperature_offset) {}

   virtual ~NullAtmosphere() = default;

   Atmosphere *Clone() const { throw std::logic_error("NullAtmosphere algorithms are not implemented."); }

   void CalibrateTemperatureAtAltitude(const Units::KelvinTemperature temperature, const Units::Length altitude) {
      throw std::logic_error("NullAtmosphere algorithms are not implemented.");
   }

   void AirDensity(const Units::Length h, Units::Density &rho, Units::Pressure &P) const {
      throw std::logic_error("NullAtmosphere algorithms are not implemented.");
   }

   Units::KelvinTemperature GetTemperature(const Units::Length altitude_msl) const {
      throw std::logic_error("NullAtmosphere algorithms are not implemented.");
   }

   Units::KelvinTemperature GetSeaLevelTemperature() const {
      throw std::logic_error("NullAtmosphere algorithms are not implemented.");
   }

   Units::Density GetSeaLevelDensity() const {
      throw std::logic_error("NullAtmosphere algorithms are not implemented.");
   }

   Units::MetersLength GetTropopauseHeight() const {
      throw std::logic_error("NullAtmosphere algorithms are not implemented.");
   }

   Units::Density GetTropopauseDensity() const {
      throw std::logic_error("NullAtmosphere algorithms are not implemented.");
   }

   Units::Pressure GetTropopausePressure() const {
      throw std::logic_error("NullAtmosphere algorithms are not implemented.");
   }

   Units::Speed CAS2TAS(const Units::Speed vcas, const Units::Pressure p, const Units::Density rho) const {
      throw std::logic_error("NullAtmosphere algorithms are not implemented.");
   }

   Units::Speed TAS2CAS(const Units::Speed vtas, const Units::Pressure p, const Units::Density rho) const {
      throw std::logic_error("NullAtmosphere algorithms are not implemented.");
   }

   Units::Speed SpeedOfSound(Units::KelvinTemperature temperature) const {
      throw std::logic_error("NullAtmosphere algorithms are not implemented.");
   }

   double ESFconstantCAS(const Units::Speed true_airspeed, const Units::Length altitude_msl,
                         const Units::KelvinTemperature temperature) const {
      throw std::logic_error("NullAtmosphere algorithms are not implemented.");
   }

   Units::Length GetMachIASTransition(const Units::Speed ias, const double mach) const {
      throw std::logic_error("NullAtmosphere algorithms are not implemented.");
   }
};
