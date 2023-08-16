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

#include <scalar/UnsignedAngle.h>
#include "public/LatitudeLongitudePoint.h"
#include "public/GeolibUtils.h"

using namespace aaesim;

log4cplus::Logger LatitudeLongitudePoint::m_logger =
      log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("LatitudeLongitudePoint"));

LatitudeLongitudePoint::LatitudeLongitudePoint(const Units::SignedAngle &wgs84_latitude,
                                               const Units::SignedAngle &wgs84_longitude) {
   m_llpoint.latitude = Units::SignedRadiansAngle(wgs84_latitude).value();
   m_llpoint.longitude = Units::SignedRadiansAngle(wgs84_longitude).value();
}

Units::SignedAngle LatitudeLongitudePoint::GetLatitude() const { return Units::SignedRadiansAngle(m_llpoint.latitude); }

Units::SignedAngle LatitudeLongitudePoint::GetLongitude() const {
   return Units::SignedRadiansAngle(m_llpoint.longitude);
}

const geolib_idealab::LLPoint &LatitudeLongitudePoint::GetGeolibPrimitiveLLPoint() const { return m_llpoint; }

LatitudeLongitudePoint LatitudeLongitudePoint::CreateFromGeolibPrimitive(geolib_idealab::LLPoint ll_point) {
   return LatitudeLongitudePoint(Units::SignedRadiansAngle(ll_point.latitude),
                                 Units::SignedRadiansAngle(ll_point.longitude));
}

LatitudeLongitudePoint LatitudeLongitudePoint::ProjectDistanceAlongCourse(Units::Length projection_distance,
                                                                          Units::SignedAngle course_enu) const {
   return GeolibUtils::CalculateNewPoint(*this, projection_distance, course_enu);
}

LatitudeLongitudePoint LatitudeLongitudePoint::CreateFromWaypoint(Waypoint wgs84_waypoint) {
   return LatitudeLongitudePoint(wgs84_waypoint.GetLatitude(), wgs84_waypoint.GetLongitude());
}

LatitudeLongitudePoint LatitudeLongitudePoint::CreateFromGeodeticPosition(
      EllipsoidalEarthModel::GeodeticPosition geodetic_position) {
   return LatitudeLongitudePoint(geodetic_position.latitude, geodetic_position.longitude);
}

std::pair<Units::Length, Units::SignedAngle> LatitudeLongitudePoint::CalculateRelationshipBetweenPoints(
      const LatitudeLongitudePoint &other_point) const {
   return GeolibUtils::CalculateRelationshipBetweenPoints(*this, other_point);
}
bool LatitudeLongitudePoint::ArePointsEqual(const LatitudeLongitudePoint &test_point) const {
   return GeolibUtils::ArePointsMathematicallyEqual(*this, test_point);
}

bool LatitudeLongitudePoint::operator==(const LatitudeLongitudePoint &rhs) const {
   return GetLatitude() == rhs.GetLatitude() && GetLongitude() == rhs.GetLongitude();
}
bool LatitudeLongitudePoint::operator!=(const LatitudeLongitudePoint &rhs) const { return !(rhs == *this); }
