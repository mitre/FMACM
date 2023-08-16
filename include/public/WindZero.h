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

#include "public/Wind.h"
#include "public/StandardAtmosphere.h"

namespace aaesim {
namespace open_source {
class WindZero : public Wind {
  public:
   WindZero();

   virtual ~WindZero();

   void InterpolateWind(Units::Angle latitude_in, Units::Angle longitude_in, Units::Length alt, Units::Speed &u,
                        Units::Speed &v) override;

   void InterpolateWindScalar(Units::Angle lat_in, Units::Angle lon_in, Units::Length altitude, Units::Speed &east_west,
                              Units::Speed &north_south) override;

   void InterpolateWindMatrix(Units::Angle lat_in, Units::Angle lon_in, Units::Length alt_in,
                              aaesim::open_source::WindStack &east_west,
                              aaesim::open_source::WindStack &north_south) override;

   Units::KelvinTemperature InterpolateTemperature(Units::Angle latitude_in, Units::Angle longitude_in,
                                                   Units::Length alt) override;

   Units::Pressure InterpolatePressure(Units::Angle latitude_in, Units::Angle longitude_in, Units::Length alt) override;

  private:
   StandardAtmosphere m_standard_atmosphere;
};
}  // namespace open_source
}  // namespace aaesim
