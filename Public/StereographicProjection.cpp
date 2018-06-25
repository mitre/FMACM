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
// Copyright 2018 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#include "public/StereographicProjection.h"
#include "utility/constants.h"

using namespace aaesim::constants;

Units::RadiansAngle StereographicProjection::latTPT(0);  /* North latitude of tangency point (radians) */
Units::RadiansAngle StereographicProjection::lonTPT(0);  /* **WEST** longitude of tangency point (radians) */

Units::FeetLength StereographicProjection::eRadius(0); //earth radius at tangent point (lon1, lat1), feet
   /* Convienience parameters calculated from raw parameters */
double StereographicProjection::sin_latTPT = 0; /* sin of latitude of tangency point */
double StereographicProjection::sin_clatTPT = 0; /* sin of conformal latitude of tangency point */
double StereographicProjection::cos_clatTPT = 0; /* cos of conformal latitude of tangency point */

   /* Convienience parameters used only for the reverse NAS projection */
double StereographicProjection::cos_gamma = 0;
double StereographicProjection::sin_gamma = 0;


StereographicProjection::StereographicProjection(void)
{
}

StereographicProjection::~StereographicProjection(void)
{
}

void StereographicProjection::init(Units::Angle lat, Units::Angle lon, Units::Length earthRadius)
{
  latTPT = lat;
  lonTPT = -lon;

  eRadius = earthRadius;

  //beginning of init
  double gamma;

  sin_latTPT   = sin(latTPT);

  /* Calculate sin and cos of conformal latitude instead of geodetic    */
  sin_clatTPT = toConformalSin(sin_latTPT);
  if (sin_clatTPT > 1.0) sin_clatTPT = 1.0;
  else if (sin_clatTPT < -1.0) sin_clatTPT = -1.0;

  cos_clatTPT = 1 - sin_clatTPT*sin_clatTPT;
  if (cos_clatTPT > 0.0) cos_clatTPT = sqrt(cos_clatTPT);
  else cos_clatTPT = 0.0;
  /* ** ADDED FOR SOUTHERN HEMISPHERE */
  if (latTPT.value() < 0.0) cos_clatTPT = -cos_clatTPT;

  gamma = PI/2.0 - asin(sin_clatTPT);
  sin_gamma = sin(gamma);
  cos_gamma = cos(gamma);

}


//This function contains the logic for converting xy_vector (x, y) to lat2_lon2  with lat1_lon1  as the tangent point, where
//Input:
//Position lat_lon1 = (lon1, lat1), lon1 and lat1 in radian; It is the tangent point.
//Vector xy_vector = (x, y) in feet
//Output:
//Position lat2_lon2 = (*lon2, *lat2), *lon2, *lat2 in radian; 
//Processing:
//Use lat1_lon1 as the tangent point and to convert xy_vector to  lat2_lon2.
//A typical value of eRadius is 3440.1344*NM2FT (feet)

void StereographicProjection::xy_to_ll(
		const Units::Length x, const Units::Length y,
		Units::Angle &lat2, Units::Angle &lon2)

/* Here is the real reverse conversion from NAS coordinates to lat,long. */
/* Based on the routine CNV_XYLL in the AERA PL1 software, written by David */
/* Chaloux (PSI).  */
/* This non-iterative approach was concieved by David Chaloux and is */
/* documented in the memo F048-M-362 "Changes to the EnRoute Coordinate */
/* Conversion Routines".  The approach has the advantage over previous */
/* iterative algorithms and the equations in NAS-MD-312 in that it works */
/* over a much larger area of the globe. */
{
  Units::RadiansAngle alpha;  /* The angle between the point of tangency, the
                    XY position, and the center of the earth.*/
  Units::RadiansAngle beta;  /* The angle between the point of tangency, the
                   XY position, and the Longitudinal line at the
                   point of tangency.*/
  Units::RadiansAngle delta;  /* The latitude component measured from the
                    North Pole.*/
  Units::RadiansAngle epsilon;  /* The longitude component measured from the
                      point of tangency.*/
  double sin_eps;  /* Used in calculating Epsilon.*/
  double cos_eps;  /* Used in calculating Epsilon.*/
  double sin_alpha;
  double cos_alpha;
  double sin_delta;
  double cos_delta;
  double dlatc;	// it's an angle, but used in some other calculations too

  double sin_phi;


  /* Find alpha - the angle between the point of tangency,
     the specified (X,Y) position, and the center of the earth.*/

  /* First find the angle between point of tangency,
     the specified position, and the point opposite the point
     of tangency on the globe.*/

  alpha = Units::RadiansAngle(asin(sqrt(x * x + y * y)/
                    (sqrt(x * x + y * y + (4.*eRadius*eRadius) ))));

  /* Now convert to what we originally wanted (angle with center
     of earth) by multiplying by two.*/

  alpha = alpha * 2.0;

  cos_alpha = cos(alpha);
  sin_alpha = sin(alpha);

  /* Now find Beta, the angle between the point of tangency,
     the XY position, and the Longitudinal line at the point
     of tangency.*/

  if (x == Units::FeetLength(0.0) && y == Units::FeetLength(0.0))
    beta = Units::RadiansAngle(0.0);
  else
    beta = Units::arctan2(Units::FeetLength(x).value(), Units::FeetLength(y).value());

  /* Find Delta, the latitude component measured from the
     North Pole.*/

  cos_delta = cos_alpha*cos_gamma + sin_alpha*sin_gamma*cos(beta);

  /* Be sure that roundoffs haven't gotten you slightly larger or
     smaller than allowable limits.*/

  if (cos_delta > 1.0)
    cos_delta = 1.0;

  if (cos_delta < -1.0)
    cos_delta = -1.0;

  delta = Units::RadiansAngle(acos(cos_delta));
  sin_delta = sin(delta);

  dlatc = PI/2. - delta.value();  /* The conformal latitude of the X,Y point*/

  /* Find Epsilon, The longitude component measured from the
     point of tangency.*/

  sin_eps = sin_delta;   /* Catches both 0. and Pi case*/

  if (sin_delta != 0.0)
    sin_eps = sin_alpha * sin(beta) / sin_delta;

  /*Again make sure that we are within allowable limits. Must do
    this because of roundoff errors*/

  if (sin_eps > 1.0)
    sin_eps = 1.0;
  else if (sin_eps < -1.0)
    sin_eps = -1.0;

  /* Note: The following can fail in a couple of cases. If the
     point of tangency is at one of the poles or if the XY position
     to be converted is at one of the poles it will fail. The case
     where the XY location is at one of the poles is tested here.
     Point of tangency at the pole should be avoided.*/

  if (sin_delta > 0.0)
    cos_eps = (cos_alpha - cos_gamma*cos_delta)/
      (sin_gamma*sin_delta);
  else
    cos_eps = 0.0;

  /*Again make sure that roundoff doesn't bite you.*/

  if (cos_eps < -1.0)
    cos_eps = -1.0;
  else if (cos_eps > 1.0)
    cos_eps = 1.0;

  epsilon = Units::arctan2(sin_eps, cos_eps);

  /*Now find the actual longitude*/

  lon2 = lonTPT - epsilon;

  /* Now, convert the latitude which is in conformal coordinates
     to geodetic coordinates. This method of doing that is not the
     method used by Chaloux, but uses a technique outlined by
     NAS-MD-312.  However, the calculation is performed twice.
     First to derive an initial estimate, then second to refine the 
     estimate.  This technique was found to have the lowest worst
     case errors. */

  sin_phi = sin(dlatc);

  dlatc = sin_phi / (GEOD_CONST_A + GEOD_CONST_B * sin_phi * sin_phi);

  /*Perform the calculation 1 more time for more accuracy*/

  dlatc = sin_phi / (GEOD_CONST_A + GEOD_CONST_B * dlatc * dlatc);

  if (dlatc > 1.0)
    dlatc = 1.0;
  else if (dlatc < -1.0)
    dlatc = -1.0;
   
  lat2 = Units::RadiansAngle(asin(dlatc)); /* WILL NOT WORK FOR THE SOUTHERN HEMISPHERE!!! */
  lon2 = -lon2; /* Convert back to east longitude */
}


//-------------------------------------------------------------------------------
//This function is mainly for the purpose of testing the above function.
//This function contains the logic for the converting lat2_lon2 to xy_vector with lat1_lon1 as the tangent point, where
//Input:
//Position lat1_lon1 = (lon1, lat1), lon1 and lat1 in radian; It is the tangent point.
//Position position2 = (lon2, lat2), lon2, lat2 in radian.
//
//Output:
//Vector Vector = (*x, *y) in feet
//Processing:
//Use position1 as the tangent point and convert position2 to Vector.
//A typical value of eRadius is 3440.1344*NM2FT (feet)
void StereographicProjection::ll_to_xy(
		const Units::Angle lat2, Units::Angle lon2,
		Units::Length &x, Units::Length &y)

/* Here is the real conversion to NAS coordinates.*/
/* Based on the routine CNV_LLXY from the AERA PL1 software and*/
/* NAS-MD-312 Appendix D.*/
{
  double sin_lat;
  Units::RadiansAngle dlong;
  double cos_dlong, sin_dlong;
  double sin_PHI, cos_PHI;
  double denom;


  lon2 = -lon2; /* Convert from east longitude to west longitude*/

  sin_lat = sin(lat2);
  dlong = lonTPT - lon2;  /* delta longitude from point of tangency.*/
  cos_dlong = cos(dlong);
  sin_dlong = sin(dlong);

  /* Convert to conformal latitude instead of geodetic*/
  sin_PHI  = toConformalSin(sin_lat);

  if (sin_PHI > 1.0) sin_PHI = 1.0;
  else if (sin_PHI < -1.) sin_PHI = -1.0;

  /* Determine the cos_PHI. This is a faster and more accurate*/
  /* method than cos_PHI = cos(Asin(sin_PHI)).*/
  cos_PHI = 1 - sin_PHI*sin_PHI;
  if (cos_PHI > 0.0) cos_PHI = sqrt(cos_PHI);
  else cos_PHI = 0.0;
  /* ADDED FOR THE SOUTHERN HEMISPHERE*/
  if (lat2 < Units::RadiansAngle(0.0)) cos_PHI = -cos_PHI;

  denom = 1 + sin_PHI*sin_clatTPT + 
    cos_PHI*cos_clatTPT*cos_dlong;

  x = 2.0*eRadius*sin_dlong*cos_PHI/denom;
  y = 2.0*eRadius*(sin_PHI*cos_clatTPT - cos_PHI*sin_clatTPT*cos_dlong)/denom;

}

double StereographicProjection::toConformalSin(double x) {
  return x * (GEOD_CONST_A  + GEOD_CONST_B *(x)*(x));
}