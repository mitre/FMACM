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

#include "scalar/Length.h"
#include "public/Waypoint.h"
#include "public/EllipsoidalEarthModel.h"
#include "geolib/Geolib.h"
#include "scalar/UnsignedAngle.h"
#include "scalar/SignedAngle.h"

/*
 *
 */
namespace aaesim {

class LatitudeLongitudePoint {

  public:
   LatitudeLongitudePoint() = default;

   LatitudeLongitudePoint(const Units::SignedAngle &wgs84_latitude, const Units::SignedAngle &wgs84_longitude);

   ~LatitudeLongitudePoint() = default;

   bool operator==(const LatitudeLongitudePoint &rhs) const;
   bool operator!=(const LatitudeLongitudePoint &rhs) const;

   Units::SignedAngle GetLatitude() const;

   Units::SignedAngle GetLongitude() const;

   const geolib_idealab::LLPoint &GetGeolibPrimitiveLLPoint() const;

   bool ArePointsEqual(const LatitudeLongitudePoint &test_point) const;

   static LatitudeLongitudePoint CreateFromGeolibPrimitive(geolib_idealab::LLPoint ll_point);

   static LatitudeLongitudePoint CreateFromWaypoint(const Waypoint &wgs84_waypoint);

   static LatitudeLongitudePoint CreateFromGeodeticPosition(
         const EllipsoidalEarthModel::GeodeticPosition &geodetic_position);

   /**
    *
    * @param projection_distance
    * @param course_enu this is the angle in the ENU convention used by aaesim
    * @see GeolibUtils::CalculateNewPoint
    * @return
    */
   LatitudeLongitudePoint ProjectDistanceAlongCourse(Units::Length projection_distance,
                                                     Units::SignedAngle course_enu) const;

   /**
    * Get the defined relationship (distance and course) between "this" and "other".
    *
    * @param other_point
    * @return defined relationship pair
    * @see GeolibUtils::CalculateRelationshipBetweenPoints
    */
   std::pair<Units::Length, Units::SignedAngle> CalculateRelationshipBetweenPoints(
         const LatitudeLongitudePoint &other_point) const;

  private:
   static log4cplus::Logger m_logger;
   geolib_idealab::LLPoint m_llpoint;
};
}  // namespace aaesim
