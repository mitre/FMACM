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

/*
 * EllipsoidalEarthModel.cpp
 *
 *  Created on: Jun 30, 2015
 *      Author: klewis
 */

#include "public/EllipsoidalEarthModel.h"
#include <iomanip>

log4cplus::Logger EllipsoidalEarthModel::logger = log4cplus::Logger::getInstance(
      LOG4CPLUS_TEXT("EllipsoidalEarthModel"));

const Units::Length EllipsoidalEarthModel::mWGS84SemiMajorRadius = Units::MetersLength(6378137.0);
const double EllipsoidalEarthModel::mWGS84EccentricitySquared = 6.69437999014e-3;

EllipsoidalEarthModel::EllipsoidalEarthModel()
      :
      semiMajorRadius(mWGS84SemiMajorRadius),
      semiMajorRadius2(mWGS84SemiMajorRadius * mWGS84SemiMajorRadius),
      eccentricity2(mWGS84EccentricitySquared),
      eccentricity4(mWGS84EccentricitySquared * mWGS84EccentricitySquared) {

}

EllipsoidalEarthModel::~EllipsoidalEarthModel() {
}

void EllipsoidalEarthModel::convertGeodeticToAbsolute(
      const EarthModel::GeodeticPosition &geo,
      EarthModel::AbsolutePositionEcef &ecef) const {

   // FIXME incoming altitude is ignored
   const double sinLat = sin(geo.latitude);
   const double cosLat = cos(geo.latitude);
   const Units::Length N = this->semiMajorRadius /
                           sqrt(1.0 - this->eccentricity2 * sinLat * sinLat);
   ecef.x = N * cosLat * cos(geo.longitude);
   ecef.y = N * cosLat * sin(geo.longitude);
   ecef.z = N * (1.0 - this->eccentricity2) * sinLat;
   LOG4CPLUS_TRACE(logger, "(" << std::setprecision(15) <<
                               Units::DegreesAngle(geo.latitude) << "," << Units::DegreesAngle(geo.longitude)
                               << ") --> (" <<
                               Units::MetersLength(ecef.x) << "," << Units::MetersLength(ecef.y) << ","
                               << Units::MetersLength(ecef.z)
                               << ")");
}

void EllipsoidalEarthModel::convertAbsoluteToGeodetic(
      const EarthModel::AbsolutePositionEcef &ecef,
      EarthModel::GeodeticPosition &geo) const {

   // Convert ECEF to Geodetic
   Units::Length z = ecef.z;

   // Ferrari's Solution (Wikipedia)
   double zeta = (1 - eccentricity2) * z * z / semiMajorRadius2;
   Units::Length p = sqrt(ecef.x * ecef.x + ecef.y * ecef.y);
   double s = (eccentricity4 * zeta * p * p) / (semiMajorRadius2 * 4);
   double rho = (p * p / semiMajorRadius2 + zeta - eccentricity4) / 6;
   double rhocubed = rho * rho * rho;
   double t = pow(rhocubed + s + sqrt(s * (s + 2 * rhocubed)), 0.333333333333);
   double u = rho + t + (rho * rho) / t;
   double v = sqrt(u * u + eccentricity4 * zeta);
   double w = mWGS84EccentricitySquared * (u + v - zeta) / (2 * v);
   double kappa = 1 + (mWGS84EccentricitySquared * (sqrt(u + v + w * w) + w)) / (u + v);

   // Now solve for lat & lon
   double latRadians = atan(kappa * z / p);
   double lonRadians = atan2(Units::MetersLength(ecef.y).value(),
                             Units::MetersLength(ecef.x).value());
   geo.latitude = Units::SignedRadiansAngle(latRadians);
   geo.longitude = Units::SignedRadiansAngle(lonRadians);
   geo.altitude = Units::MetersLength(0);   // FIXME

   LOG4CPLUS_TRACE(logger, "(" << std::setprecision(15) <<
                               Units::MetersLength(ecef.x) << "," << Units::MetersLength(ecef.y) << ","
                               << Units::MetersLength(ecef.z)
                               << ") --> (" <<
                               Units::DegreesAngle(geo.latitude) << "," << Units::DegreesAngle(geo.longitude)
                               << ")");
}

std::shared_ptr<LocalTangentPlane> EllipsoidalEarthModel::makeEnuConverter(
      const GeodeticPosition &pointOfTangencyGeo,
      const LocalPositionEnu &pointOfTangencyEnu) const {
   EarthModel::AbsolutePositionEcef ecef;
   convertGeodeticToAbsolute(pointOfTangencyGeo, ecef);

   std::shared_ptr<LocalTangentPlane> converter =
         std::shared_ptr<LocalTangentPlane>(new LocalTangentPlane(this, ecef, pointOfTangencyEnu));
   converter->initializeRotationForGeodeticOrigin();
   // rotate around the -y axis to the specified latitude
   converter->rotateEnuFrame(0, -1, 0, pointOfTangencyGeo.latitude);
   // rotate around the z axis to the specified longitude
   converter->rotateEnuFrame(0, 0, 1, pointOfTangencyGeo.longitude);
   return converter;
}
