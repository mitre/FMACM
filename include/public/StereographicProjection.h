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

#pragma once

#include <scalar/Angle.h>
#include <scalar/Length.h>

class StereographicProjection {
  public:
   StereographicProjection(void);

   ~StereographicProjection(void);

   static void init(Units::Angle lat, Units::Angle lon, Units::Length earthRadius);

   static void xy_to_ll(const Units::Length x, const Units::Length y, Units::Angle &lat2, Units::Angle &lon2);

   static void ll_to_xy(const Units::Angle lat2, Units::Angle lon2, Units::Length &x, Units::Length &y);

  private:
   static double toConformalSin(double x);

   /* Raw parameters for the NAS conversion */
   static Units::RadiansAngle latTPT; /* North latitude of tangency point (radians) */
   static Units::RadiansAngle lonTPT; /* **WEST** longitude of tangency point (radians) */
   static Units::FeetLength eRadius;  // earth radius at tangent point (lon1, lat1), feet

   /* Convienience parameters calculated from raw parameters */
   static double sin_latTPT;  /* sin of latitude of tangency point */
   static double sin_clatTPT; /* sin of conformal latitude of tangency point */
   static double cos_clatTPT; /* cos of conformal latitude of tangency point */

   /* Convienience parameters used only for the reverse NAS projection */
   static double cos_gamma;
   static double sin_gamma;
};
