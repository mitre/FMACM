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
#include "public/AircraftIntent.h"
#include "public/CoreUtils.h"
#include "public/SingleTangentPlaneSequence.h"
#include "public/TangentPlaneSequence.h"
#include "utils/public/PublicUtils.h"

using namespace aaesim::test::utils;

namespace aaesim {
namespace test {
namespace open_source {

class AircraftIntentTester : public AircraftIntent {
  public:
   AircraftIntentTester(const AircraftIntent aircraft_intent) {}
   std::list<Waypoint> Wrapper_AddConnectingTfLeg(const std::list<Waypoint> &first_waypoint_vector,
                                                  const std::list<Waypoint> &second_waypoint_vector) {
      return AddConnectingLeg(first_waypoint_vector, second_waypoint_vector);
   }
   static AircraftIntentTester MakeEmptyAircraftIntent() {
      const AircraftIntent aircraft_intent;
      return AircraftIntentTester(aircraft_intent);
   }
};

TEST(AircraftIntent, Copy) {
   const AircraftIntent expected_intent = PublicUtils::PrepareAircraftIntent("./resources/enroute_waypoints_test1.txt");

   // This is the tested method
   AircraftIntent copy_of_intent;
   copy_of_intent.Copy(expected_intent);
   // -------------------------

   ASSERT_TRUE(copy_of_intent == expected_intent);
}

TEST(AircraftIntent, CopyConstructor) {
   const AircraftIntent expected_intent = PublicUtils::PrepareAircraftIntent("./resources/enroute_waypoints_test1.txt");

   // This is the tested method
   const AircraftIntent copy_of_intent(expected_intent);
   // -------------------------

   ASSERT_TRUE(copy_of_intent == expected_intent);
}

TEST(AircraftIntent, AddConnectingTfLeg_test_0) {
   std::list<Waypoint> first_vector_set, second_vector_set;
   second_vector_set.push_back(Waypoint("1", Units::ONE_RADIAN_ANGLE, Units::ONE_RADIAN_ANGLE));
   AircraftIntentTester aircraft_intent_tester = AircraftIntentTester::MakeEmptyAircraftIntent();
   const std::list<Waypoint> modified_waypoints_vector =
         aircraft_intent_tester.Wrapper_AddConnectingTfLeg(first_vector_set, second_vector_set);

   EXPECT_EQ(modified_waypoints_vector.size(), 1);
   EXPECT_EQ(second_vector_set.back().GetName(), modified_waypoints_vector.front().GetName());
}

TEST(AircraftIntent, AddConnectingTfLeg_test_1) {
   std::list<Waypoint> first_vector_set, second_vector_set;
   first_vector_set.push_back(Waypoint("1", Units::ONE_RADIAN_ANGLE, Units::ONE_RADIAN_ANGLE));

   AircraftIntentTester aircraft_intent_tester = AircraftIntentTester::MakeEmptyAircraftIntent();
   const std::list<Waypoint> modified_waypoints_vector =
         aircraft_intent_tester.Wrapper_AddConnectingTfLeg(first_vector_set, second_vector_set);

   EXPECT_EQ(modified_waypoints_vector.size(), 1);
   EXPECT_LT(first_vector_set.back().GetName().compare(modified_waypoints_vector.front().GetName()), 0);
}

TEST(AircraftIntent, AddConnectingTfLeg_test_2) {
   std::list<Waypoint> first_vector_set, second_vector_set;
   first_vector_set.push_back(Waypoint("1", Units::ONE_RADIAN_ANGLE, Units::ONE_RADIAN_ANGLE));
   second_vector_set.push_back(Waypoint("2", Units::ONE_RADIAN_ANGLE, Units::ONE_RADIAN_ANGLE));

   AircraftIntentTester aircraft_intent_tester = AircraftIntentTester::MakeEmptyAircraftIntent();
   const std::list<Waypoint> modified_waypoints_vector =
         aircraft_intent_tester.Wrapper_AddConnectingTfLeg(first_vector_set, second_vector_set);

   EXPECT_EQ(modified_waypoints_vector.size(), 2);
   EXPECT_LT(first_vector_set.back().GetName().compare(modified_waypoints_vector.front().GetName()), 0);
   EXPECT_EQ(second_vector_set.back().GetName().compare(modified_waypoints_vector.back().GetName()), 0);
}

TEST(AircraftIntent, LoadEndToEndWaypoints) {

   std::string test_file_to_load = "./resources/aircraftintent_full_route_kden_kphx.txt";
   FILE *fp;
   fp = fopen(test_file_to_load.c_str(), "r");
   if (fp == NULL) {
      std::cout << "Test resource file " << test_file_to_load << " not found." << std::endl;
      FAIL();
   }

   DecodedStream stream;
   bool r = stream.open_file(test_file_to_load);
   if (!r) {
      std::cout << "Test resource file " << test_file_to_load << " could not be loaded." << std::endl;
      FAIL();
   }
   stream.set_echo(false);

   SingleTangentPlaneSequence::ClearStaticMembers();
   CoreUtils::UpdateMaximumAllowableSingleLegLength(Units::infinity());

   // This is the tested method ---------------
   AircraftIntent aircraft_intent;
   aircraft_intent.load(&stream);
   // -----------------------------------------

   ASSERT_TRUE(aircraft_intent.GetNumberOfWaypoints() == 26);
   ASSERT_TRUE(aircraft_intent.GetWaypoint(0).GetName().compare("17R") == 0);
   ASSERT_TRUE(aircraft_intent.GetRouteData().m_waypoint_phase_of_flight.front() ==
               AircraftIntent::WaypointPhaseOfFlight::ASCENT);
   ASSERT_TRUE(aircraft_intent.GetWaypoint(aircraft_intent.GetNumberOfWaypoints() - 1).GetName().compare("UXCUN") == 0);
   ASSERT_TRUE(aircraft_intent.GetRouteData().m_waypoint_phase_of_flight.back() ==
               AircraftIntent::WaypointPhaseOfFlight::DESCENT);
   EXPECT_TRUE(aircraft_intent.ContainsWaypointName("SIGHT"));
   EXPECT_FALSE(aircraft_intent.ContainsWaypointName("NOTFOUND"));

   CoreUtils::ResetMaximumAllowableSingleLegLength();
}

TEST(AircraftIntent, LoadOldWaypointDefinitionCleanly) {

   std::string test_file_to_load = "./resources/aircraft_intent_tight_turn.txt";
   FILE *fp;
   fp = fopen(test_file_to_load.c_str(), "r");
   if (fp == NULL) {
      std::cout << "Test resource file " << test_file_to_load << " not found." << std::endl;
      FAIL();
   }

   DecodedStream stream;
   bool r = stream.open_file(test_file_to_load);
   if (!r) {
      std::cout << "Test resource file " << test_file_to_load << " could not be loaded." << std::endl;
      FAIL();
   }
   stream.set_echo(false);

   SingleTangentPlaneSequence::ClearStaticMembers();
   CoreUtils::UpdateMaximumAllowableSingleLegLength(Units::infinity());

   // This is the tested method ---------------
   AircraftIntent aircraft_intent;
   aircraft_intent.load(&stream);
   // -----------------------------------------

   ASSERT_TRUE(aircraft_intent.GetNumberOfWaypoints() == 15);
   ASSERT_TRUE(aircraft_intent.GetWaypoint(0).GetName().compare("TACUS") == 0);
   ASSERT_TRUE(aircraft_intent.GetRouteData().m_waypoint_phase_of_flight.front() ==
               AircraftIntent::WaypointPhaseOfFlight::DESCENT);
   ASSERT_TRUE(aircraft_intent.GetWaypoint(aircraft_intent.GetNumberOfWaypoints() - 1).GetName().compare("RELIN") == 0);
   ASSERT_TRUE(aircraft_intent.GetRouteData().m_waypoint_phase_of_flight.back() ==
               AircraftIntent::WaypointPhaseOfFlight::DESCENT);

   CoreUtils::ResetMaximumAllowableSingleLegLength();
}

TEST(AircraftIntent, wgs84_to_xy) {
   // open the test data file and get a stream
   std::string testData = "./resources/EAGUL5_GALLUP.txt";
   FILE *fp;
   fp = fopen(testData.c_str(), "r");
   if (fp == NULL) {
      printf("Test resource file %s not found.", testData.c_str());
   }

   DecodedStream stream;
   bool r = stream.open_file(testData);
   if (!r) {
      stream.report_error("cant open file\n");
      exit(-60);
   }
   stream.set_echo(false);

   SingleTangentPlaneSequence::ClearStaticMembers();
   CoreUtils::UpdateMaximumAllowableSingleLegLength(Units::infinity());
   AircraftIntent aiTest;
   aiTest.load(&stream);  // read the test data
   aiTest.UpdateXYZFromLatLonWgs84();

   /*
    * Note: the hardcoded expect data below comes from a MATLAB implementation.
    */

   // Asserts on xWp
   const double xWpExpected[13] = {277562.820999161,
                                   201232.863838201,
                                   186285.934613104,
                                   134053.369540901,
                                   100297.996878049,
                                   73830.6751753431,
                                   44152.7658582631,
                                   39745.2432526958,
                                   25354.9291255275,
                                   18785.3834675821,
                                   11622.3437133676,
                                   6287.15962065872,
                                   0};
   for (int var = 0; var < aiTest.GetNumberOfWaypoints(); ++var) {
      EXPECT_NEAR(xWpExpected[var], aiTest.GetRouteData().m_x[var].value(), TOLERANCE_METERS_TIGHT);
   }

   // Asserts on yWp
   const double yWpExpected[13] = {233339.721977913,
                                   155112.436782425,
                                   140854.236841671,
                                   117022.543148477,
                                   101469.15976212,
                                   76726.3797563801,
                                   48809.8190509554,
                                   42659.8427600031,
                                   22574.5169130158,
                                   13184.5343448488,
                                   3044.06282124402,
                                   -10.9324304166729,
                                   0};
   for (int var = 0; var < aiTest.GetNumberOfWaypoints(); ++var) {
      EXPECT_NEAR(yWpExpected[var], aiTest.GetRouteData().m_y[var].value(), TOLERANCE_METERS_TIGHT);
   }

   CoreUtils::ResetMaximumAllowableSingleLegLength();
}

TEST(AircraftIntent, xyz_to_wgs84) {
   std::string testData = "./resources/EAGUL5_GALLUP.txt";
   FILE *fp;
   fp = fopen(testData.c_str(), "r");
   if (fp == NULL) {
      printf("Test resource file %s not found.", testData.c_str());
      FAIL();
   }

   DecodedStream stream;
   bool r = stream.open_file(testData);
   if (!r) {
      stream.report_error("cant open file\n");
      FAIL();
   }
   stream.set_echo(false);

   SingleTangentPlaneSequence::ClearStaticMembers();
   AircraftIntent aiTest;
   aiTest.load(&stream);
   aiTest.UpdateXYZFromLatLonWgs84();

   // Convert back to lat/lon
   Units::Angle latRad[128], lonRad[128];
   for (int i = 0; i < aiTest.GetNumberOfWaypoints(); i++) {
      aiTest.GetLatLonFromXYZ(Units::MetersLength(aiTest.GetRouteData().m_x[i]),
                              Units::MetersLength(aiTest.GetRouteData().m_y[i]),
                              Units::MetersLength(aiTest.GetRouteData().m_z[i]), latRad[i], lonRad[i]);
   }

   auto wp_list = aiTest.GetWaypointList();
   const auto tangent_plane_sequence = std::make_shared<TangentPlaneSequence>(wp_list);
   const auto waypoints = tangent_plane_sequence->GetWaypointsFromInitialization();

   for (int var = 0; var < aiTest.GetNumberOfWaypoints(); ++var) {
      EXPECT_NEAR(Units::RadiansAngle(waypoints[var].GetLatitude()).value(), Units::RadiansAngle(latRad[var]).value(),
                  TOLERANCE_RADIANS);
   }

   for (int var = 0; var < aiTest.GetNumberOfWaypoints(); ++var) {
      EXPECT_NEAR(Units::RadiansAngle(waypoints[var].GetLongitude()).value(), Units::RadiansAngle(lonRad[var]).value(),
                  TOLERANCE_RADIANS);
   }

   // go back the other way
   for (int i = 0; i < aiTest.GetNumberOfWaypoints(); ++i) {
      EarthModel::GeodeticPosition geo;
      geo.latitude = Units::RadiansAngle(latRad[i]);
      geo.longitude = Units::RadiansAngle(lonRad[i]);
      geo.altitude = Units::MetersLength(0);

      EarthModel::LocalPositionEnu enu;
      tangent_plane_sequence->ConvertGeodeticToLocal(geo, enu);

      EXPECT_NEAR(aiTest.GetRouteData().m_x[i].value(), Units::MetersLength(enu.x).value(), TOLERANCE_METERS);
      EXPECT_NEAR(aiTest.GetRouteData().m_y[i].value(), Units::MetersLength(enu.y).value(), TOLERANCE_METERS);
   }
}

TEST(AircraftIntent, load_waypoints_variations) {
   std::vector<std::pair<std::string, const unsigned int>> test_files = {
         std::make_pair("./resources/aircraft_intent_load_test1.txt", 27),
         std::make_pair("./resources/aircraft_intent_load_test2.txt", 12),
         std::make_pair("./resources/aircraft_intent_load_test3.txt", 2),
         std::make_pair("./resources/aircraft_intent_load_test4.txt", 13),
         std::make_pair("./resources/aircraft_intent_load_test5.txt", 16),
         std::make_pair("./resources/aircraft_intent_load_test6.txt", 15),
         std::make_pair("./resources/aircraft_intent_load_test7.txt", 26),
   };

   for (auto test_details : test_files) {
      auto test_file = test_details.first;
      auto expected_waypoint_count = test_details.second;
      FILE *fp;
      fp = fopen(test_file.c_str(), "r");
      if (fp == NULL) {
         std::cout << "Test resource file " << test_file << " not found." << std::endl;
         FAIL();
      }

      DecodedStream stream;
      bool r = stream.open_file(test_file);
      if (!r) {
         std::cout << "Test resource file " << test_file << " could not be loaded." << std::endl;
         FAIL();
      }
      stream.set_echo(false);

      SingleTangentPlaneSequence::ClearStaticMembers();
      CoreUtils::UpdateMaximumAllowableSingleLegLength(Units::infinity());

      // This is the tested method ---------------
      AircraftIntent aircraft_intent;
      aircraft_intent.load(&stream);
      EXPECT_EQ(expected_waypoint_count, aircraft_intent.GetNumberOfWaypoints());
   }
}

TEST(AircraftIntent, load_no_waypoints_throws) {
   std::string test_file = "./resources/aircraft_intent_load_test8.txt";
   FILE *fp;
   fp = fopen(test_file.c_str(), "r");
   if (fp == NULL) {
      std::cout << "Test resource file " << test_file << " not found." << std::endl;
      FAIL();
   }

   DecodedStream stream;
   bool r = stream.open_file(test_file);
   if (!r) {
      std::cout << "Test resource file " << test_file << " could not be loaded." << std::endl;
      FAIL();
   }
   stream.set_echo(false);

   SingleTangentPlaneSequence::ClearStaticMembers();
   CoreUtils::UpdateMaximumAllowableSingleLegLength(Units::infinity());

   AircraftIntent aircraft_intent;
   EXPECT_ANY_THROW(aircraft_intent.load(&stream));
}

TEST(AircraftIntent, test_consistency_long_route) {
   SingleTangentPlaneSequence::ClearStaticMembers();
   CoreUtils::ResetMaximumAllowableSingleLegLength();

   std::string test_file = "./resources/long_distance_route.txt";
   FILE *fp;
   fp = fopen(test_file.c_str(), "r");
   if (fp == NULL) {
      std::cout << "Test resource file " << test_file << " not found." << std::endl;
      FAIL();
   }

   DecodedStream stream;
   bool r = stream.open_file(test_file);
   if (!r) {
      std::cout << "Test resource file " << test_file << " could not be loaded." << std::endl;
      FAIL();
   }
   stream.set_echo(false);

   AircraftIntent aircraft_intent;
   aircraft_intent.load(&stream);
   auto wplist = aircraft_intent.GetWaypointList();
   auto position_converter = std::make_shared<TangentPlaneSequence>(wplist);
   for (Waypoint wp : aircraft_intent.GetWaypointList()) {
      EarthModel::LocalPositionEnu enu_position;
      EarthModel::GeodeticPosition geodetic_position;
      position_converter->ConvertGeodeticToLocal(EarthModel::GeodeticPosition::CreateFromWaypoint(wp), enu_position);
      position_converter->ConvertLocalToGeodetic(enu_position, geodetic_position);
      EXPECT_NEAR(Units::RadiansAngle(geodetic_position.latitude).value(),
                  Units::RadiansAngle(wp.GetLatitude()).value(), 1e-5);
      EXPECT_NEAR(Units::RadiansAngle(geodetic_position.longitude).value(),
                  Units::RadiansAngle(wp.GetLongitude()).value(), 1e-5);
   }
}

TEST(AircraftIntent, CopyAndTrimAfterNamedWaypoint1) {
   std::string test_file_to_load = "./resources/aircraftintent_full_route_kden_kphx.txt";
   const std::string waypoint_name_to_trim("VNNOM");
   AircraftIntent test_aircraft_intent = aaesim::test::utils::PublicUtils::LoadAircraftIntent(test_file_to_load);
   const uint before_trim_count = test_aircraft_intent.GetNumberOfWaypoints();
   const std::string final_waypoint_before_trim(test_aircraft_intent.GetWaypointList().back().GetName());
   AircraftIntent trimmed_result =
         AircraftIntent::CopyAndTrimAfterNamedWaypoint(test_aircraft_intent, waypoint_name_to_trim);
   EXPECT_GT(before_trim_count, trimmed_result.GetNumberOfWaypoints());
   EXPECT_TRUE(trimmed_result.ContainsWaypointName(waypoint_name_to_trim));
   EXPECT_FALSE(trimmed_result.ContainsWaypointName(final_waypoint_before_trim));
   EXPECT_TRUE(trimmed_result.GetRouteData().m_name.back().compare(waypoint_name_to_trim) == 0);
   EXPECT_EQ(test_aircraft_intent.GetPlannedCruiseAltitude(), trimmed_result.GetPlannedCruiseAltitude());
   EXPECT_EQ(test_aircraft_intent.GetPlannedCruiseMach(), trimmed_result.GetPlannedCruiseMach());
}

TEST(AircraftIntent, CopyAndTrimAfterNamedWaypoint2) {
   std::string test_file_to_load = "./resources/aircraftintent_full_route_kden_kphx.txt";
   const std::string waypoint_name_to_trim("SOLAR");
   AircraftIntent test_aircraft_intent = aaesim::test::utils::PublicUtils::LoadAircraftIntent(test_file_to_load);
   const uint before_trim_count = test_aircraft_intent.GetNumberOfWaypoints();
   const std::string final_waypoint_before_trim(test_aircraft_intent.GetWaypointList().back().GetName());
   AircraftIntent trimmed_result =
         AircraftIntent::CopyAndTrimAfterNamedWaypoint(test_aircraft_intent, waypoint_name_to_trim);
   EXPECT_GT(before_trim_count, trimmed_result.GetNumberOfWaypoints());
   EXPECT_TRUE(trimmed_result.ContainsWaypointName(waypoint_name_to_trim));
   EXPECT_FALSE(trimmed_result.ContainsWaypointName(final_waypoint_before_trim));
   EXPECT_TRUE(trimmed_result.GetRouteData().m_name.back().compare(waypoint_name_to_trim) == 0);
   EXPECT_EQ(test_aircraft_intent.GetPlannedCruiseAltitude(), trimmed_result.GetPlannedCruiseAltitude());
   EXPECT_EQ(test_aircraft_intent.GetPlannedCruiseMach(), trimmed_result.GetPlannedCruiseMach());
}

TEST(AircraftIntent, CopyAndTrimNoChange) {
   std::string test_file_to_load = "./resources/aircraftintent_full_route_kden_kphx.txt";
   const std::string waypoint_name_to_trim("UXCUN");
   AircraftIntent test_aircraft_intent = aaesim::test::utils::PublicUtils::LoadAircraftIntent(test_file_to_load);
   AircraftIntent trimmed_result =
         AircraftIntent::CopyAndTrimAfterNamedWaypoint(test_aircraft_intent, waypoint_name_to_trim);
   EXPECT_TRUE(test_aircraft_intent == trimmed_result);
}

TEST(AircraftIntent, findCommonWaypoint_NoCommonRoute) {
   const AircraftIntent ai1 = PublicUtils::LoadAircraftIntent("./resources/EAGUL5_GALLUP.txt");
   const AircraftIntent ai2 = PublicUtils::LoadAircraftIntent("./resources/TACUStoRELINIntent.txt");
   const std::pair<int, int> expected(-1, -1);
   const std::pair<int, int> actual = ai1.FindCommonWaypoint(ai2);
   EXPECT_EQ(expected, actual);
}

TEST(AircraftIntent, findCommonWaypoint_CommonRoute) {
   const AircraftIntent ai1 = PublicUtils::LoadAircraftIntent("./resources/EAGUL5_GALLUP.txt");
   const std::pair<int, int> expected(0, 0);
   const std::pair<int, int> actual = ai1.FindCommonWaypoint(ai1);  // check against self -- all waypoints in common
   EXPECT_EQ(expected, actual);
}

TEST(AircraftIntent, findCommonWaypoint_PartiallyCommonRoute) {
   CoreUtils::UpdateMaximumAllowableSingleLegLength(Units::infinity());
   const AircraftIntent ai1 = PublicUtils::LoadAircraftIntent("./resources/EAGUL5_GALLUP.txt");
   const AircraftIntent ai2 = PublicUtils::LoadAircraftIntent("./resources/AchievePointCalcs_Intent.txt");
   const std::pair<int, int> expected(2, 0);
   const std::pair<int, int> actual = ai1.FindCommonWaypoint(ai2);
   EXPECT_EQ(expected, actual);
   CoreUtils::ResetMaximumAllowableSingleLegLength();
}
TEST(AircraftIntent, insertNewPointAtBeginning) {
   /*
    * Load an aircraft intent definition from resources.
    * Insert a new location and test to make sure the new location
    * is at the correct index.
    */
   // open the test data file and get a stream
   std::string testData = "./resources/EAGUL5_GALLUP.txt";
   AircraftIntent aircraftIntent = PublicUtils::PrepareAircraftIntent(testData);

   const auto idx = 0;  // insert as first point
   const std::string testName("testpt");
   const Units::MetersLength x(10.0), y(10000.0);
   aircraftIntent.InsertPairAtIndex(testName, x, y, idx);

   // Assert
   const double tol = Units::MetersLength(1e-1).value();
   EXPECT_EQ(testName, aircraftIntent.GetWaypointName(idx));
   EXPECT_NEAR(x.value(), aircraftIntent.GetWaypointX(idx).value(), tol);
   EXPECT_NEAR(y.value(), aircraftIntent.GetWaypointY(idx).value(), tol);
}

TEST(AircraftIntent, findWaypointIx) {

   CoreUtils::UpdateMaximumAllowableSingleLegLength(Units::infinity());

   AircraftIntent intent = PublicUtils::PrepareAircraftIntent("./resources/findIndexIntent.txt");

   if (!intent.IsLoaded()) {
      FAIL();
   }

   EXPECT_EQ(9, intent.GetWaypointIndexByName("JAGAL"));    // waypoint in list
   EXPECT_EQ(-1, intent.GetWaypointIndexByName("SATURN"));  // waypoint not in list
   EXPECT_EQ(-1, intent.GetWaypointIndexByName(""));        // empty waypoint

   CoreUtils::ResetMaximumAllowableSingleLegLength();
}

TEST(AircraftIntent, insertNewPointAtEnd) {
   /*
    * Load an aircraft intent definition from resources.
    * Insert a new location and test to make sure the new location
    * is at the correct index.
    */
   // open the test data file and get a stream
   std::string testData = "./resources/EAGUL5_GALLUP.txt";
   AircraftIntent aircraftIntent = PublicUtils::PrepareAircraftIntent(testData);

   const auto idx =
         aircraftIntent.GetNumberOfWaypoints();  // insert as last point, so one index beyond end (zero-based)
   const std::string testName("testpt");
   const Units::MetersLength x(10.0), y(10000.0);
   aircraftIntent.InsertPairAtIndex(testName, x, y, idx);

   // Assert
   const auto testidx = aircraftIntent.GetNumberOfWaypoints() - 1;
   const double tol = Units::MetersLength(1e-1).value();
   EXPECT_EQ(testName, aircraftIntent.GetWaypointName(testidx));
   EXPECT_NEAR(x.value(), aircraftIntent.GetWaypointX(testidx).value(), tol);
   EXPECT_NEAR(y.value(), aircraftIntent.GetWaypointY(testidx).value(), tol);
}

}  // namespace open_source
}  // namespace test
}  // namespace aaesim
