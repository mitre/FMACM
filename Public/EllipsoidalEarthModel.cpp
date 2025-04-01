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

/*
 * EllipsoidalEarthModel.cpp
 *
 *  Created on: Jun 30, 2015
 *      Author: klewis
 */

#include "public/EllipsoidalEarthModel.h"
#include <iomanip>

using namespace aaesim::open_source;

void EllipsoidalEarthModel::ConvertGeodeticToAbsolute(const EarthModel::GeodeticPosition &geo,
                                                      EarthModel::AbsolutePositionEcef &ecef) const {
   const double sinLat = sin(geo.latitude);
   const double cosLat = cos(geo.latitude);
   const Units::Length N = WGS84_SEMIMAJOR_AXIS / sqrt(1.0 - WGS84_ECCENTRICITY_SQUARED * sinLat * sinLat);
   ecef.x = N * cosLat * cos(geo.longitude);
   ecef.y = N * cosLat * sin(geo.longitude);
   ecef.z = N * (1.0 - WGS84_ECCENTRICITY_SQUARED) * sinLat;
}

void EllipsoidalEarthModel::ConvertAbsoluteToGeodetic(const EarthModel::AbsolutePositionEcef &ecef,
                                                      EarthModel::GeodeticPosition &geo) const {

   // Convert ECEF to Geodetic
   Units::Length z = ecef.z;

   // Ferrari's Solution (Wikipedia)
   double zeta = (1 - WGS84_ECCENTRICITY_SQUARED) * z * z / m_semi_major_radius_squared;
   Units::Length p = sqrt(ecef.x * ecef.x + ecef.y * ecef.y);
   double s = (m_eccentricity_4 * zeta * p * p) / (m_semi_major_radius_squared * 4);
   double rho = (p * p / m_semi_major_radius_squared + zeta - m_eccentricity_4) / 6;
   double rhocubed = rho * rho * rho;
   double t = pow(rhocubed + s + sqrt(s * (s + 2 * rhocubed)), 0.333333333333);
   double u = rho + t + (rho * rho) / t;
   double v = sqrt(u * u + m_eccentricity_4 * zeta);
   double w = WGS84_ECCENTRICITY_SQUARED * (u + v - zeta) / (2 * v);
   double kappa = 1 + (WGS84_ECCENTRICITY_SQUARED * (sqrt(u + v + w * w) + w)) / (u + v);

   // Now solve for lat & lon
   double latRadians = atan(kappa * z / p);
   double lonRadians = atan2(Units::MetersLength(ecef.y).value(), Units::MetersLength(ecef.x).value());
   geo.latitude = Units::SignedRadiansAngle(latRadians);
   geo.longitude = Units::SignedRadiansAngle(lonRadians);
   geo.altitude = Units::MetersLength(0);
}

std::shared_ptr<LocalTangentPlane> EllipsoidalEarthModel::MakeEnuConverter(
      const GeodeticPosition &pointOfTangencyGeo, const LocalPositionEnu &pointOfTangencyEnu) const {

   EarthModel::AbsolutePositionEcef ecef;
   ConvertGeodeticToAbsolute(pointOfTangencyGeo, ecef);

   std::shared_ptr<LocalTangentPlane> converter = std::make_shared<LocalTangentPlane>(this, ecef, pointOfTangencyEnu);
   converter->InitializeRotationForGeodeticOrigin();
   // rotate around the -y axis to the specified latitude
   converter->RotateEnuFrame(0, -1, 0, pointOfTangencyGeo.latitude);
   // rotate around the z axis to the specified longitude
   converter->RotateEnuFrame(0, 0, 1, pointOfTangencyGeo.longitude);
   return converter;
}
