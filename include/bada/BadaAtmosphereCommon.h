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

#include "public/Atmosphere.h"

/**
 * Functions and members shared between BadaAtmosphere versions.
 */

// exponent for eq. 3.2-15, about 5.25583
const double P_T_EXPONENT(-Units::ONE_G_ACCELERATION / (K_T * R));

// exponent for eq. 3.2-6, about 4.25583
const double RHO_T_EXPONENT(P_T_EXPONENT - 1);

namespace aaesim {
namespace bada {

class BadaAtmosphereCommon : public Atmosphere {
  private:
   static log4cplus::Logger m_logger;

  public:
   Units::Speed CAS2TAS(const Units::Speed vcas, const Units::Pressure p, const Units::Density rho) const;

   Units::Speed TAS2CAS(const Units::Speed vtas, const Units::Pressure p, const Units::Density rho) const;

   Units::Speed SpeedOfSound(Units::KelvinTemperature temperature) const;

   double ESFconstantCAS(const Units::Speed true_airspeed, const Units::Length altitude_msl,
                         const Units::KelvinTemperature temperature) const;

   Units::Length GetMachIASTransition(const Units::Speed ias, const double mach) const;
};

}  // namespace bada
}  // namespace aaesim
