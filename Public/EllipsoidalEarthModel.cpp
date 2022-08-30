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

/*
 * EllipsoidalEarthModel.cpp
 *
 *  Created on: Jun 30, 2015
 *      Author: klewis
 */

#include "public/EllipsoidalEarthModel.h"
#include <iomanip>

log4cplus::Logger EllipsoidalEarthModel::m_logger = log4cplus::Logger::getInstance(
      LOG4CPLUS_TEXT("EllipsoidalEarthModel"));

/*
 * Developers Note:
 * These earth model constants must be a numerical match to what is in geolib_idealab. However, that
 * library is not directly callable from this class because this is public code and that library is MITRE-private.
 * Do not change these values without also consulting the geolib_idealab library.
 */
const Units::Length EllipsoidalEarthModel::m_wgs84_semimajor_radius = Units::MetersLength(6378137.0);
const double EllipsoidalEarthModel::m_wgs84_eccentricity_squared = 0.0066943799901413169961;
//-----------------------------------------------------------------------------------------

EllipsoidalEarthModel::EllipsoidalEarthModel()
      :
   m_semi_major_radius(m_wgs84_semimajor_radius),
   m_semi_major_radius_squared(m_wgs84_semimajor_radius * m_wgs84_semimajor_radius),
   m_eccentricity_squared(m_wgs84_eccentricity_squared),
   m_eccentricity_4(m_wgs84_eccentricity_squared * m_wgs84_eccentricity_squared) {

}

EllipsoidalEarthModel::~EllipsoidalEarthModel() {
}

void EllipsoidalEarthModel::ConvertGeodeticToAbsolute(
      const EarthModel::GeodeticPosition &geo,
      EarthModel::AbsolutePositionEcef &ecef) const {

   // FIXME incoming altitude is ignored
   const double sinLat = sin(geo.latitude);
   const double cosLat = cos(geo.latitude);
   const Units::Length N = this->m_semi_major_radius /
                           sqrt(1.0 - this->m_eccentricity_squared * sinLat * sinLat);
   ecef.x = N * cosLat * cos(geo.longitude);
   ecef.y = N * cosLat * sin(geo.longitude);
   ecef.z = N * (1.0 - this->m_eccentricity_squared) * sinLat;
   LOG4CPLUS_TRACE(m_logger, "(" << std::setprecision(15) <<
                                 Units::DegreesAngle(geo.latitude) << "," << Units::DegreesAngle(geo.longitude)
                                 << ") --> (" <<
                                 Units::MetersLength(ecef.x) << "," << Units::MetersLength(ecef.y) << ","
                                 << Units::MetersLength(ecef.z)
                                 << ")");
}

void EllipsoidalEarthModel::ConvertAbsoluteToGeodetic(
      const EarthModel::AbsolutePositionEcef &ecef,
      EarthModel::GeodeticPosition &geo) const {

   // Convert ECEF to Geodetic
   Units::Length z = ecef.z;

   // Ferrari's Solution (Wikipedia)
   double zeta = (1 - m_eccentricity_squared) * z * z / m_semi_major_radius_squared;
   Units::Length p = sqrt(ecef.x * ecef.x + ecef.y * ecef.y);
   double s = (m_eccentricity_4 * zeta * p * p) / (m_semi_major_radius_squared * 4);
   double rho = (p * p / m_semi_major_radius_squared + zeta - m_eccentricity_4) / 6;
   double rhocubed = rho * rho * rho;
   double t = pow(rhocubed + s + sqrt(s * (s + 2 * rhocubed)), 0.333333333333);
   double u = rho + t + (rho * rho) / t;
   double v = sqrt(u * u + m_eccentricity_4 * zeta);
   double w = m_wgs84_eccentricity_squared * (u + v - zeta) / (2 * v);
   double kappa = 1 + (m_wgs84_eccentricity_squared * (sqrt(u + v + w * w) + w)) / (u + v);

   // Now solve for lat & lon
   double latRadians = atan(kappa * z / p);
   double lonRadians = atan2(Units::MetersLength(ecef.y).value(),
                             Units::MetersLength(ecef.x).value());
   geo.latitude = Units::SignedRadiansAngle(latRadians);
   geo.longitude = Units::SignedRadiansAngle(lonRadians);
   geo.altitude = Units::MetersLength(0);   // FIXME

   LOG4CPLUS_TRACE(m_logger, "(" << std::setprecision(15) <<
                                 Units::MetersLength(ecef.x) << "," << Units::MetersLength(ecef.y) << ","
                                 << Units::MetersLength(ecef.z)
                                 << ") --> (" <<
                                 Units::DegreesAngle(geo.latitude) << "," << Units::DegreesAngle(geo.longitude)
                                 << ")");
}

std::shared_ptr<LocalTangentPlane> EllipsoidalEarthModel::MakeEnuConverter(
      const GeodeticPosition &pointOfTangencyGeo,
      const LocalPositionEnu &pointOfTangencyEnu) const {

   LOG4CPLUS_TRACE(m_logger, "Making converter tangent at " << pointOfTangencyGeo << " and " << pointOfTangencyEnu);

   EarthModel::AbsolutePositionEcef ecef;
   ConvertGeodeticToAbsolute(pointOfTangencyGeo, ecef);

   std::shared_ptr<LocalTangentPlane> converter =
         std::shared_ptr<LocalTangentPlane>(new LocalTangentPlane(this, ecef, pointOfTangencyEnu));
   converter->initializeRotationForGeodeticOrigin();
   // rotate around the -y axis to the specified latitude
   converter->rotateEnuFrame(0, -1, 0, pointOfTangencyGeo.latitude);
   // rotate around the z axis to the specified longitude
   converter->rotateEnuFrame(0, 0, 1, pointOfTangencyGeo.longitude);
   return converter;
}
