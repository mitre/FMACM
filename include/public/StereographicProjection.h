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
// Copyright 2020 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#pragma once

#include <Angle.h>
#include <Length.h>

class StereographicProjection
{
public:

   StereographicProjection(void);

   ~StereographicProjection(void);

   static void init(Units::Angle lat,
                    Units::Angle lon,
                    Units::Length earthRadius);

   static void xy_to_ll(const Units::Length x,
                        const Units::Length y,
                        Units::Angle &lat2,
                        Units::Angle &lon2);

   static void ll_to_xy(
         const Units::Angle lat2,
         Units::Angle lon2,
         Units::Length &x,
         Units::Length &y);

private:

   static double toConformalSin(double x);

   /* Raw parameters for the NAS conversion */
   static Units::RadiansAngle latTPT;  /* North latitude of tangency point (radians) */
   static Units::RadiansAngle lonTPT;  /* **WEST** longitude of tangency point (radians) */
   static Units::FeetLength eRadius; //earth radius at tangent point (lon1, lat1), feet

   /* Convienience parameters calculated from raw parameters */
   static double sin_latTPT; /* sin of latitude of tangency point */
   static double sin_clatTPT; /* sin of conformal latitude of tangency point */
   static double cos_clatTPT; /* cos of conformal latitude of tangency point */

   /* Convienience parameters used only for the reverse NAS projection */
   static double cos_gamma;
   static double sin_gamma;
};
