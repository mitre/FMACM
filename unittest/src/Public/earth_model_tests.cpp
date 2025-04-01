#include <gtest/gtest.h>

#include "public/EarthModel.h"
#include "public/LocalTangentPlane.h"
#include "public/TangentPlaneSequence.h"
#include "public/EllipsoidalEarthModel.h"

const auto ENU_POSITION_TEST_TOLERANCE = Units::KilometersLength{1e-8};
const auto ECEF_POSITION_TEST_TOLERANCE = Units::KilometersLength{1e-8};
const auto DEGREE_POSITION_TEST_TOLERANCE = Units::DegreesAngle{1e-8};

const auto lat_reference = Units::DegreesAngle(46.017);
const auto lon_reference = Units::DegreesAngle(7.750);
const auto tangency_position_as_waypoint = Waypoint("test", lat_reference, lon_reference);
const auto lat_test_point = Units::DegreesAngle(45.976);
const auto lon_test_point = Units::DegreesAngle(7.658);
const auto test_point = EarthModel::GeodeticPosition::Of(lat_test_point, lon_test_point);

// -----------------------------------------------------------------------------------------
// These constants below are from running open source code GeographicLib
// repo: https://geographiclib.sourceforge.io/C++/doc/index.html
// test code: https://mustache.mitre.org/users/sbowman/repos/geographiclib-testing/browse
const auto ecef_x_from_geographiclib = Units::MetersLength{4400636.82668814};
const auto ecef_y_from_geographiclib = Units::MetersLength{591704.962660163};
const auto ecef_z_from_geographiclib = Units::MetersLength{4563394.05024529};
const auto enu_x_from_geographiclib = Units::MetersLength{-7129.70106116624};
const auto enu_y_from_geographiclib = Units::MetersLength{-4553.08211938867};
const auto enu_z_from_geographiclib = Units::MetersLength{-5.60559700015367};
// -----------------------------------------------------------------------------------------

TEST(LocalTangentPlane, lla2ecef_forward_reverse) {
   const auto earth_model = std::make_unique<EllipsoidalEarthModel>();
   EarthModel::AbsolutePositionEcef resolved_ecef;
   earth_model->ConvertGeodeticToAbsolute(test_point, resolved_ecef);

   ASSERT_NEAR(Units::MetersLength(resolved_ecef.x).value(), ecef_x_from_geographiclib.value(),
               Units::MetersLength(ECEF_POSITION_TEST_TOLERANCE).value());
   ASSERT_NEAR(Units::MetersLength(resolved_ecef.y).value(), ecef_y_from_geographiclib.value(),
               Units::MetersLength(ECEF_POSITION_TEST_TOLERANCE).value());
   ASSERT_NEAR(Units::MetersLength(resolved_ecef.z).value(), ecef_z_from_geographiclib.value(),
               Units::MetersLength(ECEF_POSITION_TEST_TOLERANCE).value());

   EarthModel::AbsolutePositionEcef ecef_test_point;
   ecef_test_point.x = ecef_x_from_geographiclib;
   ecef_test_point.y = ecef_y_from_geographiclib;
   ecef_test_point.z = ecef_z_from_geographiclib;
   EarthModel::GeodeticPosition resolved_position;
   earth_model->ConvertAbsoluteToGeodetic(ecef_test_point, resolved_position);
   ASSERT_NEAR(Units::DegreesAngle(resolved_position.latitude).value(), lat_test_point.value(),
               DEGREE_POSITION_TEST_TOLERANCE.value());
   ASSERT_NEAR(Units::DegreesAngle(resolved_position.longitude).value(), lon_test_point.value(),
               DEGREE_POSITION_TEST_TOLERANCE.value());
   ASSERT_DOUBLE_EQ(Units::MetersLength(resolved_position.altitude).value(), 0);
}

TEST(LocalTangentPlane, ecef2enu_forward_backward) {
   EarthModel::AbsolutePositionEcef ecef_position, updated_ecef_position;
   ecef_position.x = ecef_x_from_geographiclib;
   ecef_position.y = ecef_y_from_geographiclib;
   ecef_position.z = ecef_z_from_geographiclib;
   auto waypoint_list = std::list{tangency_position_as_waypoint};
   const auto earth_model = std::make_unique<EllipsoidalEarthModel>();
   const auto converter = std::make_unique<TangentPlaneSequence>(waypoint_list);
   EarthModel::LocalPositionEnu enu_position;

   converter->GetTangentPlanesFromInitialization().front()->ConvertAbsoluteToLocal(ecef_position, enu_position);
   ASSERT_NEAR(Units::MetersLength(enu_position.x).value(), enu_x_from_geographiclib.value(),
               ENU_POSITION_TEST_TOLERANCE.value());
   ASSERT_NEAR(Units::MetersLength(enu_position.y).value(), enu_y_from_geographiclib.value(),
               ENU_POSITION_TEST_TOLERANCE.value());
   ASSERT_NEAR(Units::MetersLength(enu_position.z).value(), enu_z_from_geographiclib.value(),
               ENU_POSITION_TEST_TOLERANCE.value());

   converter->GetTangentPlanesFromInitialization().front()->ConvertLocalToAbsolute(enu_position, updated_ecef_position);
   ASSERT_NEAR(Units::MetersLength(updated_ecef_position.x).value(), ecef_x_from_geographiclib.value(),
               ECEF_POSITION_TEST_TOLERANCE.value());
   ASSERT_NEAR(Units::MetersLength(updated_ecef_position.y).value(), ecef_y_from_geographiclib.value(),
               ECEF_POSITION_TEST_TOLERANCE.value());
   ASSERT_NEAR(Units::MetersLength(updated_ecef_position.z).value(), ecef_z_from_geographiclib.value(),
               ECEF_POSITION_TEST_TOLERANCE.value());
}

TEST(LocalTangentPlane, lla2enu_forward_reverse) {
   auto waypoint_list = std::list{tangency_position_as_waypoint};
   const auto earth_model = std::make_unique<EllipsoidalEarthModel>();
   const auto converter = std::make_unique<TangentPlaneSequence>(waypoint_list);
   EarthModel::LocalPositionEnu resolved_enu;

   converter->GetTangentPlanesFromInitialization().front()->ConvertGeodeticToLocal(test_point, resolved_enu);
   ASSERT_NEAR(Units::MetersLength(resolved_enu.x).value(), enu_x_from_geographiclib.value(),
               Units::MetersLength(ENU_POSITION_TEST_TOLERANCE).value());
   ASSERT_NEAR(Units::MetersLength(resolved_enu.y).value(), enu_y_from_geographiclib.value(),
               Units::MetersLength(ENU_POSITION_TEST_TOLERANCE).value());
   ASSERT_NEAR(Units::MetersLength(resolved_enu.z).value(), enu_z_from_geographiclib.value(),
               Units::MetersLength(ENU_POSITION_TEST_TOLERANCE).value());

   EarthModel::GeodeticPosition resolved_geodetic_pos;
   converter->GetTangentPlanesFromInitialization().front()->ConvertLocalToGeodetic(
         EarthModel::LocalPositionEnu::Of(enu_x_from_geographiclib, enu_y_from_geographiclib, enu_z_from_geographiclib),
         resolved_geodetic_pos);
   ASSERT_NEAR(Units::DegreesAngle(resolved_geodetic_pos.latitude).value(), lat_test_point.value(),
               DEGREE_POSITION_TEST_TOLERANCE.value());
   ASSERT_NEAR(Units::DegreesAngle(resolved_geodetic_pos.longitude).value(), lon_test_point.value(),
               DEGREE_POSITION_TEST_TOLERANCE.value());
   ASSERT_DOUBLE_EQ(Units::MetersLength(resolved_geodetic_pos.altitude).value(), 0);
}