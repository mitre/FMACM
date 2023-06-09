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

#include "public/WaypointPassingMonitor.h"
#include "public/Wgs84PrecalcWaypoint.h"
#include "public/LatitudeLongitudePoint.h"

namespace aaesim {
namespace open_source {
class EuclideanWaypointMonitor : public WaypointPassingMonitor {
  public:
   ~EuclideanWaypointMonitor() = default;

   void Update(const aaesim::LatitudeLongitudePoint &position, const Units::SignedAngle &ground_course_enu) override;

   bool IsPassedWaypoint() const override { return m_is_passed_waypoint; }

   static std::shared_ptr<EuclideanWaypointMonitor> OfWgs84PrecalcWaypoint(
         const aaesim::Wgs84PrecalcWaypoint &waypoint);

   static std::shared_ptr<EuclideanWaypointMonitor> OfEllipsoidalPoint(
         const aaesim::LatitudeLongitudePoint &ellipsoidal_point);

   static std::shared_ptr<EuclideanWaypointMonitor> OfGeodeticPoint(const EarthModel::GeodeticPosition &geodetic_point);

  private:
   EuclideanWaypointMonitor(const aaesim::LatitudeLongitudePoint &lat_lon_point);

   void PerformFakeTranslationToEuclidean(const aaesim::LatitudeLongitudePoint &lat_lon_point, Units::Length &x,
                                          Units::Length &y);

   bool m_is_passed_waypoint;
   aaesim::LatitudeLongitudePoint m_point_to_monitor;
};
}  // namespace open_source
}  // namespace aaesim