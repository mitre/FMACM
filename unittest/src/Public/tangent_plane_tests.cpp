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

#include <gtest/gtest.h>

#include "public/SingleTangentPlaneSequence.h"
#include "public/TangentPlaneSequence.h"
#include "public/Waypoint.h"
#include "public/AircraftIntent.h"

namespace aaesim {
namespace open_source {
namespace test {
static double TIGHT_TOLERANCE_DEGREES{1e-12};
TEST(TangentPlaneSequence, StraightLineConsistency) {
   Waypoint start_waypoint{"start", Units::DegreesAngle(38.0), Units::DegreesAngle(-77.0)};
   Waypoint end_waypoint{"end", Units::DegreesAngle(40.0), Units::DegreesAngle(-70.0)};
   auto waypoints = std::list<Waypoint>{start_waypoint, end_waypoint};
   auto tangent_plane_sequence = std::make_unique<TangentPlaneSequence>(waypoints);

   auto comparator = [&tangent_plane_sequence](const Waypoint &waypoint) {
      EarthModel::LocalPositionEnu enu_position;
      tangent_plane_sequence->ConvertGeodeticToLocal(EarthModel::GeodeticPosition::CreateFromWaypoint(waypoint),
                                                     enu_position);
      EarthModel::GeodeticPosition computed_waypoint;
      tangent_plane_sequence->ConvertLocalToGeodetic(enu_position, computed_waypoint);
      EXPECT_NEAR(Units::DegreesAngle(computed_waypoint.latitude).value(),
                  Units::DegreesAngle(waypoint.GetLatitude()).value(), TIGHT_TOLERANCE_DEGREES);
      EXPECT_NEAR(Units::DegreesAngle(computed_waypoint.longitude).value(),
                  Units::DegreesAngle(waypoint.GetLongitude()).value(), TIGHT_TOLERANCE_DEGREES);
   };
   std::for_each(waypoints.begin(), waypoints.end(), comparator);
}

TEST(TangentPlaneSequence, LineSequenceConsistency) {
   Waypoint start_waypoint{"start", Units::DegreesAngle(35.0), Units::DegreesAngle(-77.0)};
   Waypoint wp1{"wp1", Units::DegreesAngle(37.5), Units::DegreesAngle(-76.0)};
   Waypoint wp2{"wp2", Units::DegreesAngle(37.6), Units::DegreesAngle(-71.0)};
   Waypoint end_waypoint{"end", Units::DegreesAngle(40.0), Units::DegreesAngle(-70.0)};
   auto waypoints = std::list<Waypoint>{start_waypoint, wp1, wp2, end_waypoint};
   auto tangent_plane_sequence = std::make_unique<TangentPlaneSequence>(waypoints);

   auto comparator = [&tangent_plane_sequence](const Waypoint &waypoint) {
      EarthModel::LocalPositionEnu enu_position;
      tangent_plane_sequence->ConvertGeodeticToLocal(EarthModel::GeodeticPosition::CreateFromWaypoint(waypoint),
                                                     enu_position);
      EarthModel::GeodeticPosition computed_waypoint;
      tangent_plane_sequence->ConvertLocalToGeodetic(enu_position, computed_waypoint);
      EXPECT_NEAR(Units::DegreesAngle(computed_waypoint.latitude).value(),
                  Units::DegreesAngle(waypoint.GetLatitude()).value(), TIGHT_TOLERANCE_DEGREES);
      EXPECT_NEAR(Units::DegreesAngle(computed_waypoint.longitude).value(),
                  Units::DegreesAngle(waypoint.GetLongitude()).value(), TIGHT_TOLERANCE_DEGREES);
   };
   std::for_each(waypoints.begin(), waypoints.end(), comparator);
}

TEST(TangentPlaneSequence, LineSequenceConsistency2) {
   SingleTangentPlaneSequence::ClearStaticMembers();
   Waypoint start_waypoint{"start", Units::DegreesAngle(35.0), Units::DegreesAngle(-77.0)};
   Waypoint wp1{"wp1", Units::DegreesAngle(37.5), Units::DegreesAngle(-76.0)};
   Waypoint wp2{"wp2", Units::DegreesAngle(37.6), Units::DegreesAngle(-71.0)};
   Waypoint end_waypoint{"end", Units::DegreesAngle(40.0), Units::DegreesAngle(-70.0)};
   auto waypoints = std::list<Waypoint>{start_waypoint, wp1, wp2, end_waypoint};
   AircraftIntent aircraft_intent;
   aircraft_intent.LoadWaypointsFromList(waypoints, std::list<Waypoint>(), std::list<Waypoint>());
   auto wplist = aircraft_intent.GetWaypointList();
   auto tangent_plane_sequence = std::make_shared<TangentPlaneSequence>(wplist);
   auto route_data = aircraft_intent.GetRouteData();

   struct ZippedData {
      Units::MetersLength x;
      Units::MetersLength y;
      Units::Angle lat;
      Units::Angle lon;
      static ZippedData Of(Units::MetersLength x, Units::MetersLength y, Units::Angle lat, Units::Angle lon) {
         ZippedData zd;
         zd.x = x;
         zd.y = y;
         zd.lat = lat;
         zd.lon = lon;
         return zd;
      };
   };
   std::vector<ZippedData> zipped_route;
   for (auto idx = 0; idx < route_data.m_high_altitude_constraint.size(); ++idx) {
      zipped_route.push_back(ZippedData::Of(route_data.m_x[idx], route_data.m_y[idx], route_data.m_latitude[idx],
                                            route_data.m_longitude[idx]));
   }

   auto enu_comparator_high_tolerance = [&tangent_plane_sequence](const ZippedData &zd) {
      EarthModel::LocalPositionEnu enu_position_off_path;
      enu_position_off_path.x = zd.x + Units::MetersLength(100.0);
      enu_position_off_path.y = zd.y - Units::MetersLength(100.0);
      EarthModel::GeodeticPosition computed_lat_lon;
      tangent_plane_sequence->ConvertLocalToGeodetic(enu_position_off_path, computed_lat_lon);

      EarthModel::LocalPositionEnu recomputed_enu_position;
      tangent_plane_sequence->ConvertGeodeticToLocal(computed_lat_lon, recomputed_enu_position);

      static double TOLERANCE_METERS{50};
      EXPECT_NEAR(Units::MetersLength(enu_position_off_path.x).value(),
                  Units::MetersLength(recomputed_enu_position.x).value(), TOLERANCE_METERS);
      EXPECT_NEAR(Units::MetersLength(enu_position_off_path.y).value(),
                  Units::MetersLength(recomputed_enu_position.y).value(), TOLERANCE_METERS);
   };
   std::for_each(zipped_route.begin(), zipped_route.end(), enu_comparator_high_tolerance);
}

}  // namespace test
}  // namespace open_source
}  // namespace aaesim
