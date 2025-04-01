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
#include "public/ArcOnEllipsoid.h"
#include "public/EarthModel.h"
#include "public/LatitudeLongitudePoint.h"
#include "geolib/Geolib.h"
#include "public/LineOnEllipsoid.h"
#include "public/GeolibUtils.h"
#include "MiniCSV/minicsv.h"

using namespace geolib_idealab;
using namespace aaesim;

namespace aaesim {
namespace test {
namespace open_source {

TEST(GeolibLibrary, basic_internal_consistency_test) {
   /*
    * The geolib library has its own extensive testing system (thousands of tests). This test does not
    * attempt to recreate that infrastructure. The point here is to ensure a basic operation works. Anything
    * more extensive should be pushed back onto the geolib testing infrastructure, not implemented here.
    */
   LLPoint origin;
   origin.latitude = 0.0;
   origin.longitude = 0.0;

   LLPoint destination;
   destination.latitude = 1.0 * M_PI / 180.0;
   destination.longitude = 1.0 * M_PI / 180.0;

   // Call inverse operation
   double epsilon = DEFAULT_EPS;
   double crs_radians_calculated = DBL_MIN;
   double reverse_course_radians_calculated = DBL_MIN;
   double distance_nm_calculated = DBL_MIN;
   ErrorSet error_set_inverse = inverse(origin, destination, &crs_radians_calculated,
                                        &reverse_course_radians_calculated, &distance_nm_calculated, epsilon);
   if (error_set_inverse != SUCCESS) {
      std::cout << "BLAH. Something went wrong with the inverse operation! " << formatErrorMessage(error_set_inverse)
                << std::endl;
      FAIL();
   }

   // Now call direct()
   LLPoint destination_calculated;
   ErrorSet error_set_direct =
         direct(origin, crs_radians_calculated, distance_nm_calculated, &destination_calculated, epsilon);
   if (error_set_direct != SUCCESS) {
      std::cout << "BLAH. Something went wrong with the direct operation! " << formatErrorMessage(error_set_direct)
                << std::endl;
      FAIL();
   }

   // Assert
   EXPECT_NEAR(destination.latitude, destination_calculated.latitude, 1e-15);
   EXPECT_NEAR(destination.longitude, destination_calculated.longitude, 1e-15);
}

TEST(LatitudeLongitudePoint, createObject) {

   LLPoint test_point;
   test_point.latitude = 10 * M_PI / 180;
   test_point.longitude = 5 * M_PI / 180;
   LatitudeLongitudePoint new_point(Units::SignedRadiansAngle(test_point.latitude),
                                    Units::SignedRadiansAngle(test_point.longitude));

   EXPECT_DOUBLE_EQ(test_point.latitude, Units::SignedRadiansAngle(new_point.GetLatitude()).value());
   EXPECT_DOUBLE_EQ(test_point.longitude, Units::SignedRadiansAngle(new_point.GetLongitude()).value());
}

TEST(LineOnEllipsoid, calculate_point_on_shape) {
   LatitudeLongitudePoint start_point(Units::SignedDegreesAngle(4), Units::SignedRadiansAngle(-10));
   LatitudeLongitudePoint end_point(Units::SignedDegreesAngle(3), Units::SignedRadiansAngle(-103));

   const LineOnEllipsoid geodesic = LineOnEllipsoid::CreateFromPoints(start_point, end_point);

   // This is the tested method ----
   const LatitudeLongitudePoint calculated_point =
         geodesic.CalculatePointAtDistanceFromStartPoint(geodesic.GetShapeLength());
   EXPECT_NEAR(Units::RadiansAngle(end_point.GetLatitude()).value(),
               Units::RadiansAngle(calculated_point.GetLatitude()).value(), 1e-8);
   EXPECT_NEAR(Units::RadiansAngle(end_point.GetLongitude()).value(),
               Units::RadiansAngle(calculated_point.GetLongitude()).value(), 1e-8);
   // ------------------------------
}

TEST(LineOnEllipsoid, calculate_course_at_point_on_shape) {
   LatitudeLongitudePoint start_point(Units::SignedDegreesAngle(4), Units::SignedRadiansAngle(-10));
   LatitudeLongitudePoint end_point(Units::SignedDegreesAngle(3), Units::SignedRadiansAngle(-103));

   const LineOnEllipsoid geodesic = LineOnEllipsoid::CreateFromPoints(start_point, end_point);

   // This is the tested method ----
   const std::pair<Units::SignedAngle, LatitudeLongitudePoint> calculated_course_and_point =
         geodesic.CalculateCourseAtDistanceFromStartPoint(geodesic.GetShapeLength());
   const Units::SignedRadiansAngle actual_course = calculated_course_and_point.first;
   const LatitudeLongitudePoint actual_point = calculated_course_and_point.second;
   EXPECT_NEAR(Units::SignedRadiansAngle(geodesic.GetForwardCourseEnuAtEndPoint()).value(), actual_course.value(),
               1e-8);
   EXPECT_NEAR(Units::RadiansAngle(end_point.GetLatitude()).value(),
               Units::RadiansAngle(actual_point.GetLatitude()).value(), 1e-8);
   EXPECT_NEAR(Units::RadiansAngle(end_point.GetLongitude()).value(),
               Units::RadiansAngle(actual_point.GetLongitude()).value(), 1e-8);
   // ------------------------------
}

TEST(LineOnEllipsoid, createLineFromTwoPoints) {

   LatitudeLongitudePoint start_point(Units::SignedDegreesAngle(4), Units::SignedRadiansAngle(-10));
   LatitudeLongitudePoint end_point(Units::SignedDegreesAngle(3), Units::SignedRadiansAngle(-103));

   // This is the tested method ----
   LineOnEllipsoid geodesic = LineOnEllipsoid::CreateFromPoints(start_point, end_point);
   // ------------------------------

   // Call directly into the geolib library
   const LLPoint origin = start_point.GetGeolibPrimitiveLLPoint();
   const LLPoint destination = end_point.GetGeolibPrimitiveLLPoint();
   double course_ned_radians_at_origin_calculated = DBL_MIN;
   double reverse_course_radians_at_dest_calculated = DBL_MIN;
   double distance_nm_calculated = DBL_MIN;
   ErrorSet error_set_inverse =
         inverse(origin, destination, &course_ned_radians_at_origin_calculated,
                 &reverse_course_radians_at_dest_calculated, &distance_nm_calculated, DEFAULT_EPS);
   if (error_set_inverse != ErrorCodes::SUCCESS) {
      FAIL();
   }

   EXPECT_NEAR(distance_nm_calculated, Units::NauticalMilesLength(geodesic.GetShapeLength()).value(), 1e-12);
   EXPECT_NEAR(
         course_ned_radians_at_origin_calculated,
         Units::UnsignedRadiansAngle(GeolibUtils::ConvertCourseFromEnuToNed(geodesic.GetForwardCourseEnuAtStartPoint()))
               .value(),
         1e-12);
   const double expected_forward_course_ned =
         modpos(reverse_course_radians_at_dest_calculated + M_PI,
                M_2PI);  // geolib inverse gives the reverse the course, so we need to do adjust to forward here
   EXPECT_NEAR(
         expected_forward_course_ned,
         Units::UnsignedRadiansAngle(GeolibUtils::ConvertCourseFromEnuToNed(geodesic.GetForwardCourseEnuAtEndPoint()))
               .value(),
         1e-12);
}

TEST(GeolibUtils, createArcFromThreeKnownPoints) {

   // Calculate an arc
   const Units::NauticalMilesLength expected_radius(2.5);
   const LatitudeLongitudePoint center_point(Units::SignedDegreesAngle(33.3862), Units::SignedRadiansAngle(-111.887));
   LLPoint start_point_calculated;
   ErrorSet error_set_1 = direct(center_point.GetGeolibPrimitiveLLPoint(), 0, expected_radius.value(),
                                 &start_point_calculated, DEFAULT_EPS);
   if (error_set_1 != ErrorCodes::SUCCESS) {
      FAIL();
   }
   LatitudeLongitudePoint start_point = LatitudeLongitudePoint::CreateFromGeolibPrimitive(start_point_calculated);

   LLPoint end_point_calculated;
   ErrorSet error_set_2 = direct(center_point.GetGeolibPrimitiveLLPoint(), M_PI / 2, expected_radius.value(),
                                 &end_point_calculated, DEFAULT_EPS);
   if (error_set_2 != ErrorCodes::SUCCESS) {
      FAIL();
   }
   LatitudeLongitudePoint end_point = LatitudeLongitudePoint::CreateFromGeolibPrimitive(end_point_calculated);

   // This is the tested method ----
   ArcOnEllipsoid arc_on_ellipsoid =
         GeolibUtils::CreateArcOnEllipsoid(start_point, end_point, center_point, ArcDirection::CLOCKWISE);
   // ------------------------------

   EXPECT_NEAR(M_PI / 2, Units::SignedRadiansAngle(arc_on_ellipsoid.GetArcAngularExtent()).value(), 1e-5);
   EXPECT_NEAR(expected_radius.value(), Units::NauticalMilesLength(arc_on_ellipsoid.GetRadius()).value(), 1e-5);
}

TEST(LatitudeLongitudePoint, calculate_new_point_1nm) {

   const LatitudeLongitudePoint start_point(Units::DegreesAngle(0.0), Units::DegreesAngle(0));
   const Units::NauticalMilesLength distance(1.0);
   const Units::SignedDegreesAngle go_north_enu(0);

   // This is the tested method ----
   LatitudeLongitudePoint calculated_point = start_point.ProjectDistanceAlongCourse(distance, go_north_enu);
   // ------------------------------

   // Now get comparison data for "truth" measurement by calling the inverse operation
   double crs_radians_ned_truth = DBL_MIN;
   double tmp_course_radians_calculated = DBL_MIN;
   double distance_nm_truth = DBL_MIN;
   ErrorSet error_set_inverse =
         inverse(start_point.GetGeolibPrimitiveLLPoint(), calculated_point.GetGeolibPrimitiveLLPoint(),
                 &crs_radians_ned_truth, &tmp_course_radians_calculated, &distance_nm_truth, GEOLIB_EPSILON);
   if (error_set_inverse != SUCCESS) {
      std::cout << "BLAH. Something went wrong with the inverse operation! " << formatErrorMessage(error_set_inverse)
                << std::endl;
      FAIL();
   }

   EXPECT_NEAR(distance_nm_truth, distance.value(), 1e-12);
   EXPECT_NEAR(crs_radians_ned_truth,
               Units::UnsignedRadiansAngle(GeolibUtils::ConvertCourseFromEnuToNed(go_north_enu)).value(), 1e-12);
}

TEST(LatitudeLongitudePoint, calculate_new_point_1meter) {

   const LatitudeLongitudePoint start_point(Units::DegreesAngle(0.0), Units::DegreesAngle(0));
   const Units::MetersLength distance(1.0);
   const Units::SignedDegreesAngle go_east_enu(90);

   // This is the tested method ----
   LatitudeLongitudePoint calculated_point = start_point.ProjectDistanceAlongCourse(distance, go_east_enu);
   // ------------------------------

   // Now get comparison data for "truth" measurement by calling the inverse operation
   double epsilon = DEFAULT_EPS;
   double crs_radians_ned_truth = DBL_MIN;
   double tmp_course_radians_calculated = DBL_MIN;
   double distance_nm_truth = DBL_MIN;
   ErrorSet error_set_inverse =
         inverse(start_point.GetGeolibPrimitiveLLPoint(), calculated_point.GetGeolibPrimitiveLLPoint(),
                 &crs_radians_ned_truth, &tmp_course_radians_calculated, &distance_nm_truth, epsilon);
   if (error_set_inverse != SUCCESS) {
      std::cout << "BLAH. Something went wrong with the inverse operation! " << formatErrorMessage(error_set_inverse)
                << std::endl;
      FAIL();
   }

   EXPECT_NEAR(distance_nm_truth, Units::NauticalMilesLength(distance).value(), 1e-12);
   EXPECT_NEAR(crs_radians_ned_truth,
               Units::UnsignedRadiansAngle(GeolibUtils::ConvertCourseFromEnuToNed(go_east_enu)).value(), 1e-12);
}

TEST(GeolibUtils, test_CalculateNewPoint) {
   const LatitudeLongitudePoint start_point(Units::DegreesAngle(0.0), Units::DegreesAngle(0));
   const Units::MetersLength distance(1.0);
   const Units::SignedDegreesAngle go_east_enu(0);

   // This is the tested method ----
   LatitudeLongitudePoint calculated_point = GeolibUtils::CalculateNewPoint(start_point, distance, go_east_enu);
   // ------------------------------

   // Now get comparison data for "truth" measurement by calling the inverse operation
   double epsilon = DEFAULT_EPS;
   double crs_radians_truth_ned_unsigned = DBL_MIN;
   double tmp_course_radians_calculated = DBL_MIN;
   double distance_nm_truth = DBL_MIN;
   ErrorSet error_set_inverse =
         inverse(start_point.GetGeolibPrimitiveLLPoint(), calculated_point.GetGeolibPrimitiveLLPoint(),
                 &crs_radians_truth_ned_unsigned, &tmp_course_radians_calculated, &distance_nm_truth, epsilon);
   if (error_set_inverse != SUCCESS) {
      std::cout << "BLAH. Something went wrong with the inverse operation! " << formatErrorMessage(error_set_inverse)
                << std::endl;
      FAIL();
   }

   EXPECT_NEAR(distance_nm_truth, Units::NauticalMilesLength(distance).value(), 1e-12);
   EXPECT_NEAR(crs_radians_truth_ned_unsigned,
               Units::SignedRadiansAngle(GeolibUtils::ConvertCourseFromEnuToNed(go_east_enu)).value(), 1e-12);
}

TEST(LatitudeLongitudePoint, test_RelationshipBewteenPoints_1meter) {
   const LatitudeLongitudePoint start_point(Units::DegreesAngle(0.0), Units::DegreesAngle(0));
   const Units::MetersLength distance(1.0);
   const Units::SignedDegreesAngle go_east_enu(0);

   // Now call directly into geolib for "truth" data point
   LLPoint end_point_from_geolib;
   ErrorSet error_set_direct =
         direct(start_point.GetGeolibPrimitiveLLPoint(),
                Units::UnsignedRadiansAngle(GeolibUtils::ConvertCourseFromEnuToNed(go_east_enu)).value(),
                Units::NauticalMilesLength(distance).value(), &end_point_from_geolib, GEOLIB_EPSILON);
   if (error_set_direct != SUCCESS) {
      std::cout << "BLAH. Something went wrong with the direct operation! " << formatErrorMessage(error_set_direct)
                << std::endl;
      FAIL();
   }
   const LatitudeLongitudePoint end_point = LatitudeLongitudePoint::CreateFromGeolibPrimitive(end_point_from_geolib);

   // This is the tested method ----
   std::pair<Units::MetersLength, Units::SignedRadiansAngle> point_relationship =
         start_point.CalculateRelationshipBetweenPoints(end_point);
   // ------------------------------

   EXPECT_NEAR(distance.value(), point_relationship.first.value(), 1e-10);
   EXPECT_NEAR(go_east_enu.value(), point_relationship.second.value(), 1e-12);
}

TEST(GeolibUtils, test_RelationshipBewteenPoints_1meter) {
   const LatitudeLongitudePoint start_point(Units::DegreesAngle(0.0), Units::DegreesAngle(0));
   const Units::MetersLength distance(1.0);
   const Units::SignedDegreesAngle go_east_enu(0);

   // Now call directly into geolib for "truth" data point
   LLPoint end_point_from_geolib;
   ErrorSet error_set_direct =
         direct(start_point.GetGeolibPrimitiveLLPoint(),
                Units::UnsignedRadiansAngle(GeolibUtils::ConvertCourseFromEnuToNed(go_east_enu)).value(),
                Units::NauticalMilesLength(distance).value(), &end_point_from_geolib, GEOLIB_EPSILON);
   if (error_set_direct != SUCCESS) {
      std::cout << "BLAH. Something went wrong with the direct operation! " << formatErrorMessage(error_set_direct)
                << std::endl;
      FAIL();
   }
   const LatitudeLongitudePoint end_point = LatitudeLongitudePoint::CreateFromGeolibPrimitive(end_point_from_geolib);

   // This is the tested method ----
   std::pair<Units::MetersLength, Units::SignedRadiansAngle> point_relationship =
         GeolibUtils::CalculateRelationshipBetweenPoints(start_point, end_point);
   // ------------------------------

   EXPECT_NEAR(distance.value(), point_relationship.first.value(), 1e-10);
   EXPECT_NEAR(go_east_enu.value(), point_relationship.second.value(), 1e-12);
}

TEST(GeolibUtils, test_RelationshipBewteenPoints_1nm) {
   const LatitudeLongitudePoint start_point(Units::DegreesAngle(0.0), Units::DegreesAngle(0));
   const Units::NauticalMilesLength distance(1.0);
   const Units::SignedDegreesAngle go_east_enu(0);

   // Now call directly into geolib for "truth" data point
   LLPoint end_point_from_geolib;
   ErrorSet error_set_direct =
         direct(start_point.GetGeolibPrimitiveLLPoint(),
                Units::UnsignedRadiansAngle(GeolibUtils::ConvertCourseFromEnuToNed(go_east_enu)).value(),
                Units::NauticalMilesLength(distance).value(), &end_point_from_geolib, GEOLIB_EPSILON);
   if (error_set_direct != SUCCESS) {
      std::cout << "BLAH. Something went wrong with the direct operation! " << formatErrorMessage(error_set_direct)
                << std::endl;
      FAIL();
   }
   const LatitudeLongitudePoint end_point = LatitudeLongitudePoint::CreateFromGeolibPrimitive(end_point_from_geolib);

   // This is the tested method ----
   std::pair<Units::NauticalMilesLength, Units::SignedRadiansAngle> point_relationship =
         GeolibUtils::CalculateRelationshipBetweenPoints(start_point, end_point);
   // ------------------------------

   EXPECT_NEAR(distance.value(), point_relationship.first.value(), 1e-10);
   EXPECT_NEAR(go_east_enu.value(), point_relationship.second.value(), 1e-12);
}

TEST(GeolibUtils, test_ConvertCourseFromEnuToNed) {
   // Create a mapping of input courses (in ENU coords) to expected ouput (in NED coords)
   std::vector<std::pair<Units::SignedDegreesAngle, Units::UnsignedDegreesAngle>> zipped_angles_input_then_expected;
   zipped_angles_input_then_expected.push_back(std::pair<Units::SignedDegreesAngle, Units::UnsignedDegreesAngle>(
         Units::SignedDegreesAngle(0), Units::UnsignedDegreesAngle(90)));
   zipped_angles_input_then_expected.push_back(std::pair<Units::SignedDegreesAngle, Units::UnsignedDegreesAngle>(
         Units::SignedDegreesAngle(45), Units::UnsignedDegreesAngle(45)));
   zipped_angles_input_then_expected.push_back(std::pair<Units::SignedDegreesAngle, Units::UnsignedDegreesAngle>(
         Units::SignedDegreesAngle(90), Units::UnsignedDegreesAngle(0)));
   zipped_angles_input_then_expected.push_back(std::pair<Units::SignedDegreesAngle, Units::UnsignedDegreesAngle>(
         Units::SignedDegreesAngle(135), Units::UnsignedDegreesAngle(315)));
   zipped_angles_input_then_expected.push_back(std::pair<Units::SignedDegreesAngle, Units::UnsignedDegreesAngle>(
         Units::SignedDegreesAngle(-45), Units::UnsignedDegreesAngle(135)));
   zipped_angles_input_then_expected.push_back(std::pair<Units::SignedDegreesAngle, Units::UnsignedDegreesAngle>(
         Units::SignedDegreesAngle(-90), Units::UnsignedDegreesAngle(180)));
   zipped_angles_input_then_expected.push_back(std::pair<Units::SignedDegreesAngle, Units::UnsignedDegreesAngle>(
         Units::SignedDegreesAngle(-95), Units::UnsignedDegreesAngle(185)));
   zipped_angles_input_then_expected.push_back(std::pair<Units::SignedDegreesAngle, Units::UnsignedDegreesAngle>(
         Units::SignedDegreesAngle(180), Units::UnsignedDegreesAngle(270)));

   // Test the conversion method
   for (auto test_pair : zipped_angles_input_then_expected) {
      const Units::SignedDegreesAngle enu_crs_to_test = test_pair.first;
      const Units::UnsignedDegreesAngle expected_result_after_conversion = test_pair.second;
      const Units::UnsignedDegreesAngle result_crs_ned = GeolibUtils::ConvertCourseFromEnuToNed(enu_crs_to_test);
      EXPECT_NEAR(expected_result_after_conversion.value(), result_crs_ned.value(), 1e-10);
   }
}

TEST(GeolibUtils, test_ConvertCourseFromNedToEnu) {
   // Create a mapping of input courses (in NED coords) to expected ouput (in ENU coords)
   std::vector<std::pair<Units::UnsignedDegreesAngle, Units::SignedDegreesAngle>> zipped_angles_input_then_expected;
   zipped_angles_input_then_expected.push_back(std::pair<Units::UnsignedDegreesAngle, Units::SignedDegreesAngle>(
         Units::UnsignedDegreesAngle(90), Units::SignedDegreesAngle(0)));
   zipped_angles_input_then_expected.push_back(std::pair<Units::UnsignedDegreesAngle, Units::SignedDegreesAngle>(
         Units::UnsignedDegreesAngle(45), Units::SignedDegreesAngle(45)));
   zipped_angles_input_then_expected.push_back(std::pair<Units::UnsignedDegreesAngle, Units::SignedDegreesAngle>(
         Units::UnsignedDegreesAngle(0), Units::SignedDegreesAngle(90)));
   zipped_angles_input_then_expected.push_back(std::pair<Units::UnsignedDegreesAngle, Units::SignedDegreesAngle>(
         Units::UnsignedDegreesAngle(315), Units::SignedDegreesAngle(135)));
   zipped_angles_input_then_expected.push_back(std::pair<Units::UnsignedDegreesAngle, Units::SignedDegreesAngle>(
         Units::UnsignedDegreesAngle(135), Units::SignedDegreesAngle(-45)));
   zipped_angles_input_then_expected.push_back(std::pair<Units::UnsignedDegreesAngle, Units::SignedDegreesAngle>(
         Units::UnsignedDegreesAngle(180), Units::SignedDegreesAngle(-90)));
   zipped_angles_input_then_expected.push_back(std::pair<Units::UnsignedDegreesAngle, Units::SignedDegreesAngle>(
         Units::UnsignedDegreesAngle(185), Units::SignedDegreesAngle(-95)));
   zipped_angles_input_then_expected.push_back(std::pair<Units::UnsignedDegreesAngle, Units::SignedDegreesAngle>(
         Units::UnsignedDegreesAngle(270), Units::SignedDegreesAngle(180)));

   // Test the conversion method
   for (auto test_pair : zipped_angles_input_then_expected) {
      const Units::SignedDegreesAngle ned_crs_to_test = test_pair.first;
      const Units::UnsignedDegreesAngle expected_result_after_conversion = test_pair.second;
      const Units::UnsignedDegreesAngle result_crs_enu = GeolibUtils::ConvertCourseFromNedToEnu(ned_crs_to_test);
      EXPECT_NEAR(expected_result_after_conversion.value(), result_crs_enu.value(), 1e-10);
   }
}

TEST(GeolibUtils, test_CalculateLineLineIntersectionPoint) {
   // Define an intersection point that is the truth point
   const LatitudeLongitudePoint expected_point(Units::DegreesAngle(38.0), Units::DegreesAngle(-77.3));

   // Define two lines that are guaranteed to pass through the intersection point
   const Units::NauticalMilesLength dist1 = Units::NauticalMilesLength(.1);
   const LatitudeLongitudePoint end_point1 =
         GeolibUtils::CalculateNewPoint(expected_point, dist1, Units::DegreesAngle(0));
   const LineOnEllipsoid tmp_line = LineOnEllipsoid::CreateFromPoints(expected_point, end_point1);
   const LatitudeLongitudePoint new_start_point1 = GeolibUtils::CalculateNewPoint(
         expected_point, dist1, tmp_line.GetForwardCourseEnuAtStartPoint() + Units::RadiansAngle(M_PI));
   const LineOnEllipsoid line1 = LineOnEllipsoid::CreateFromPoints(new_start_point1, end_point1);

   const Units::NauticalMilesLength dist2 = Units::NauticalMilesLength(.5);
   const LatitudeLongitudePoint end_point2 =
         GeolibUtils::CalculateNewPoint(expected_point, dist2, Units::DegreesAngle(10));
   LineOnEllipsoid line2 = LineOnEllipsoid::CreateFromPoints(expected_point, end_point2);

   // This is the tested method ----
   auto intersection_values = GeolibUtils::CalculateLineLineIntersectionPoint(line1, line2);
   const bool intersection_is_valid = std::get<0>(intersection_values);
   const LatitudeLongitudePoint calculated_intersection_point = std::get<1>(intersection_values);
   const std::vector<Units::Length> distances_to_intersection_point = std::get<2>(intersection_values);
   // ------------------------------

   EXPECT_NEAR(Units::SignedRadiansAngle(expected_point.GetLatitude()).value(),
               Units::SignedRadiansAngle(calculated_intersection_point.GetLatitude()).value(), 1e-10);
   EXPECT_NEAR(Units::SignedRadiansAngle(expected_point.GetLongitude()).value(),
               Units::SignedRadiansAngle(calculated_intersection_point.GetLongitude()).value(), 1e-10);
   EXPECT_NEAR(dist1.value(), Units::NauticalMilesLength(distances_to_intersection_point.front()).value(), 1e-10);
   EXPECT_NEAR(0.0, Units::NauticalMilesLength(distances_to_intersection_point.back()).value(), 1e-10);
   EXPECT_TRUE(intersection_is_valid);
}

TEST(GeolibUtils, test_NoPossibleLineLineIntersectionPoint) {

   // Define two lines that are guaranteed not to intersect
   const LatitudeLongitudePoint start_point1(Units::DegreesAngle(38.0), Units::DegreesAngle(-77.3));
   const LatitudeLongitudePoint end_point1 =
         GeolibUtils::CalculateNewPoint(start_point1, Units::NauticalMilesLength(.1), Units::DegreesAngle(0));
   const LineOnEllipsoid line1 = LineOnEllipsoid::CreateFromPoints(start_point1, end_point1);
   const LatitudeLongitudePoint start_point2(Units::DegreesAngle(40.1), Units::DegreesAngle(-112.8));
   const LatitudeLongitudePoint end_point2 =
         GeolibUtils::CalculateNewPoint(start_point2, Units::NauticalMilesLength(.5), Units::DegreesAngle(10));
   const LineOnEllipsoid line2 = LineOnEllipsoid::CreateFromPoints(start_point2, end_point2);

   // This is the tested method ----
   auto intersection_values = GeolibUtils::CalculateLineLineIntersectionPoint(line1, line2);
   bool intersection_is_valid = std::get<0>(intersection_values);
   // ------------------------------

   // We expect this to fail -- no intersection found
   EXPECT_FALSE(intersection_is_valid);
}

TEST(GeolibUtils, test_IsPointOnLine) {
   // Define a point that will be tested
   const LatitudeLongitudePoint test_point(Units::DegreesAngle(38.0), Units::DegreesAngle(-77.3));

   // Define a line that is guaranteed to pass through the test point
   const Units::NauticalMilesLength distance = Units::NauticalMilesLength(1);
   const LatitudeLongitudePoint point_to_east =
         GeolibUtils::CalculateNewPoint(test_point, distance, Units::SignedDegreesAngle(10));
   const LineOnEllipsoid tmp_line = LineOnEllipsoid::CreateFromPoints(test_point, point_to_east);
   const LatitudeLongitudePoint start_point = GeolibUtils::CalculateNewPoint(
         test_point, distance, tmp_line.GetForwardCourseEnuAtStartPoint() + Units::SignedRadiansAngle(M_PI));
   const LineOnEllipsoid line_with_test_point_on_it = LineOnEllipsoid::CreateFromPoints(start_point, point_to_east);

   // This is the tested method ----
   bool is_on_line = GeolibUtils::IsPointOnLine(line_with_test_point_on_it, test_point);
   // ------------------------------

   // We expect this to be true
   EXPECT_TRUE(is_on_line);
}

TEST(GeolibUtils, test_PoinIsNotOnLine) {
   // Define a point that will be used to build a line
   const LatitudeLongitudePoint line_mid_point(Units::DegreesAngle(38.0), Units::DegreesAngle(-77.3));
   const Units::NauticalMilesLength distance = Units::NauticalMilesLength(1);
   const LatitudeLongitudePoint point_to_east =
         GeolibUtils::CalculateNewPoint(line_mid_point, distance, Units::SignedDegreesAngle(10));
   const LineOnEllipsoid tmp_line = LineOnEllipsoid::CreateFromPoints(line_mid_point, point_to_east);
   const LatitudeLongitudePoint start_point = GeolibUtils::CalculateNewPoint(
         line_mid_point, distance, tmp_line.GetForwardCourseEnuAtStartPoint() + Units::SignedRadiansAngle(M_PI));
   const LineOnEllipsoid line_with_mid_point_on_it = LineOnEllipsoid::CreateFromPoints(start_point, point_to_east);

   // Define a point that is near the mid-point, but not actually the midpoint
   const LatitudeLongitudePoint test_point =
         GeolibUtils::CalculateNewPoint(line_mid_point, Units::MetersLength(.001), Units::SignedDegreesAngle(0));

   // This is the tested method ----
   bool is_on_line = GeolibUtils::IsPointOnLine(line_with_mid_point_on_it, test_point);
   // ------------------------------

   // We expect this to be false
   EXPECT_FALSE(is_on_line);
}

TEST(GeolibUtils, test_IsPointOnArc) {
   // Calculate an arc
   const Units::NauticalMilesLength defined_radius(2.5);
   const LatitudeLongitudePoint center_point(Units::SignedDegreesAngle(33.3862), Units::SignedRadiansAngle(-111.887));
   const LatitudeLongitudePoint start_point =
         GeolibUtils::CalculateNewPoint(center_point, defined_radius, Units::SignedDegreesAngle(90));
   const LatitudeLongitudePoint end_point =
         GeolibUtils::CalculateNewPoint(center_point, defined_radius, Units::SignedDegreesAngle(0));
   const ArcOnEllipsoid arc_on_ellipsoid =
         GeolibUtils::CreateArcOnEllipsoid(start_point, end_point, center_point, ArcDirection::CLOCKWISE);
   const LatitudeLongitudePoint point_on_arc =
         GeolibUtils::CalculateNewPoint(center_point, defined_radius, Units::SignedDegreesAngle(45));

   // This is the tested method ----
   bool is_on_arc = GeolibUtils::IsPointOnArc(arc_on_ellipsoid, point_on_arc);
   // ------------------------------

   // We expect this to be true
   EXPECT_TRUE(is_on_arc);
}

TEST(GeolibUtils, test_PoinIsNotOnArc) {
   // Calculate an arc
   const Units::NauticalMilesLength defined_radius(2.5);
   const LatitudeLongitudePoint center_point(Units::SignedDegreesAngle(33.3862), Units::SignedRadiansAngle(-111.887));
   const LatitudeLongitudePoint start_point =
         GeolibUtils::CalculateNewPoint(center_point, defined_radius, Units::SignedDegreesAngle(90));
   const LatitudeLongitudePoint end_point =
         GeolibUtils::CalculateNewPoint(center_point, defined_radius, Units::SignedDegreesAngle(0));
   const ArcOnEllipsoid arc_on_ellipsoid =
         GeolibUtils::CreateArcOnEllipsoid(start_point, end_point, center_point, ArcDirection::CLOCKWISE);

   // Define a point that is near the arc, but not actually on it
   const LatitudeLongitudePoint point_not_on_arc = GeolibUtils::CalculateNewPoint(
         center_point, defined_radius - Units::MetersLength(0.001), Units::SignedDegreesAngle(45));

   // This is the tested method ----
   bool is_on_arc = GeolibUtils::IsPointOnArc(arc_on_ellipsoid, point_not_on_arc);
   // ------------------------------

   // We expect this to be false
   EXPECT_FALSE(is_on_arc);
}

TEST(ArcOnEllipsoid, IsPointOnArc) {
   // Calculate an arc
   const Units::NauticalMilesLength defined_radius(2.5);
   const LatitudeLongitudePoint center_point(Units::SignedDegreesAngle(33.3862), Units::SignedRadiansAngle(-111.887));
   const LatitudeLongitudePoint start_point =
         GeolibUtils::CalculateNewPoint(center_point, defined_radius, Units::SignedDegreesAngle(90));
   const LatitudeLongitudePoint end_point =
         GeolibUtils::CalculateNewPoint(center_point, defined_radius, Units::SignedDegreesAngle(0));
   const ArcOnEllipsoid arc_on_ellipsoid =
         GeolibUtils::CreateArcOnEllipsoid(start_point, end_point, center_point, ArcDirection::CLOCKWISE);
   const LatitudeLongitudePoint point_on_arc =
         GeolibUtils::CalculateNewPoint(center_point, defined_radius, Units::SignedDegreesAngle(45));

   // This is the tested method ----
   bool is_on_arc = arc_on_ellipsoid.IsPointOnShape(point_on_arc);
   // ------------------------------

   // We expect this to be true
   EXPECT_TRUE(is_on_arc);
}

TEST(ArcOnEllipsoid, calculate_point_on_shape) {
   const Units::NauticalMilesLength defined_radius(2.5);
   const LatitudeLongitudePoint center_point(Units::SignedDegreesAngle(33.3862), Units::SignedRadiansAngle(-111.887));
   const LatitudeLongitudePoint start_point =
         GeolibUtils::CalculateNewPoint(center_point, defined_radius, Units::SignedDegreesAngle(90));
   const LatitudeLongitudePoint end_point =
         GeolibUtils::CalculateNewPoint(center_point, defined_radius, Units::SignedDegreesAngle(0));
   const ArcOnEllipsoid arc_on_ellipsoid =
         GeolibUtils::CreateArcOnEllipsoid(start_point, end_point, center_point, ArcDirection::CLOCKWISE);

   // This is the tested method ----
   const LatitudeLongitudePoint calculated_point =
         arc_on_ellipsoid.CalculatePointAtDistanceFromStartPoint(arc_on_ellipsoid.GetShapeLength());
   EXPECT_NEAR(Units::RadiansAngle(end_point.GetLatitude()).value(),
               Units::RadiansAngle(calculated_point.GetLatitude()).value(), 1e-8);
   EXPECT_NEAR(Units::RadiansAngle(end_point.GetLongitude()).value(),
               Units::RadiansAngle(calculated_point.GetLongitude()).value(), 1e-8);
   // ------------------------------
}

TEST(ArcOnEllipsoid, calculate_course_at_point_on_shape) {
   const Units::NauticalMilesLength defined_radius(2.5);
   const LatitudeLongitudePoint center_point(Units::SignedDegreesAngle(33.3862), Units::SignedRadiansAngle(-111.887));
   const LatitudeLongitudePoint start_point =
         GeolibUtils::CalculateNewPoint(center_point, defined_radius, Units::SignedDegreesAngle(90));
   const LatitudeLongitudePoint end_point =
         GeolibUtils::CalculateNewPoint(center_point, defined_radius, Units::SignedDegreesAngle(0));
   const ArcOnEllipsoid arc_on_ellipsoid =
         GeolibUtils::CreateArcOnEllipsoid(start_point, end_point, center_point, ArcDirection::CLOCKWISE);

   // This is the tested method ----
   const std::pair<Units::SignedRadiansAngle, LatitudeLongitudePoint> calculated_course_and_point_at_distance =
         arc_on_ellipsoid.CalculateCourseAtDistanceFromStartPoint(arc_on_ellipsoid.GetShapeLength());
   EXPECT_NEAR(Units::SignedRadiansAngle(arc_on_ellipsoid.GetCourseEnuTangentToEndPoint()).value(),
               calculated_course_and_point_at_distance.first.value(), 1e-8);
   EXPECT_NEAR(Units::RadiansAngle(end_point.GetLatitude()).value(),
               Units::RadiansAngle(calculated_course_and_point_at_distance.second.GetLatitude()).value(), 1e-8);
   EXPECT_NEAR(Units::RadiansAngle(end_point.GetLongitude()).value(),
               Units::RadiansAngle(calculated_course_and_point_at_distance.second.GetLongitude()).value(), 1e-8);
   // ------------------------------
}

TEST(LineOnEllipsoid, IsPointOnLine) {
   // Define a point that will be tested
   const LatitudeLongitudePoint test_point(Units::DegreesAngle(38.0), Units::DegreesAngle(-77.3));

   // Define a line that is guaranteed to pass through the test point
   const Units::NauticalMilesLength distance = Units::NauticalMilesLength(1);
   const LatitudeLongitudePoint point_to_east =
         GeolibUtils::CalculateNewPoint(test_point, distance, Units::SignedDegreesAngle(10));
   const LineOnEllipsoid tmp_line = LineOnEllipsoid::CreateFromPoints(test_point, point_to_east);
   const LatitudeLongitudePoint start_point = GeolibUtils::CalculateNewPoint(
         test_point, distance, tmp_line.GetForwardCourseEnuAtStartPoint() + Units::SignedRadiansAngle(M_PI));
   const LineOnEllipsoid line_with_test_point_on_it = LineOnEllipsoid::CreateFromPoints(start_point, point_to_east);

   // This is the tested method ----
   const bool is_on_line = line_with_test_point_on_it.IsPointOnShape(test_point);
   // ------------------------------

   // We expect this to be true
   EXPECT_TRUE(is_on_line);
}

TEST(GeolibUtils, FindPointOnLineUsingPerpendiculorProjection_valid) {
   // Define a point that will be used to build a line
   const LatitudeLongitudePoint line_mid_point(Units::DegreesAngle(38.0), Units::DegreesAngle(-77.3));
   const Units::NauticalMilesLength distance = Units::NauticalMilesLength(1);
   const LatitudeLongitudePoint point_to_east =
         GeolibUtils::CalculateNewPoint(line_mid_point, distance, Units::SignedDegreesAngle(10));
   const LineOnEllipsoid tmp_line = LineOnEllipsoid::CreateFromPoints(line_mid_point, point_to_east);
   const LatitudeLongitudePoint start_point = GeolibUtils::CalculateNewPoint(
         line_mid_point, distance, tmp_line.GetForwardCourseEnuAtStartPoint() + Units::SignedRadiansAngle(M_PI));
   const LineOnEllipsoid line_with_mid_point_on_it = LineOnEllipsoid::CreateFromPoints(start_point, point_to_east);

   // Define a point that is perpendicular to the mid-point
   const LatitudeLongitudePoint test_point =
         GeolibUtils::CalculateNewPoint(line_mid_point, Units::MetersLength(.001),
                                        tmp_line.GetForwardCourseEnuAtStartPoint() + Units::SignedDegreesAngle(90));

   // This is the tested method ----
   std::tuple<LatitudeLongitudePoint, Units::SignedAngle, Units::Length> perp_info =
         GeolibUtils::FindNearestPointOnLineUsingPerpendicularProjection(line_with_mid_point_on_it, test_point);
   const LatitudeLongitudePoint point_on_line = std::get<0>(perp_info);

   // ------------------------------

   // We expect these to match
   EXPECT_NEAR(Units::RadiansAngle(line_mid_point.GetLatitude()).value(),
               Units::RadiansAngle(point_on_line.GetLatitude()).value(), 1e-10);
   EXPECT_NEAR(Units::RadiansAngle(line_mid_point.GetLongitude()).value(),
               Units::RadiansAngle(point_on_line.GetLongitude()).value(), 1e-10);
}

TEST(GeolibUtils, IsPointInsideArc) {
   // Calculate an arc
   const Units::NauticalMilesLength defined_radius(2.5);
   const LatitudeLongitudePoint center_point(Units::SignedDegreesAngle(33.3862), Units::SignedRadiansAngle(-111.887));
   const LatitudeLongitudePoint start_point =
         GeolibUtils::CalculateNewPoint(center_point, defined_radius, Units::SignedDegreesAngle(90));
   const LatitudeLongitudePoint end_point =
         GeolibUtils::CalculateNewPoint(center_point, defined_radius, Units::SignedDegreesAngle(0));
   const ArcOnEllipsoid arc_on_ellipsoid =
         GeolibUtils::CreateArcOnEllipsoid(start_point, end_point, center_point, ArcDirection::CLOCKWISE);
   const LatitudeLongitudePoint point_inside_arc = GeolibUtils::CalculateNewPoint(
         center_point, defined_radius - Units::MetersLength(1e-5), Units::SignedDegreesAngle(45));

   // This is the tested method ----
   bool is_inside_arc = GeolibUtils::IsPointInsideArcSegment(arc_on_ellipsoid, point_inside_arc);
   // ------------------------------

   // We expect this to be true
   EXPECT_TRUE(is_inside_arc);
}

TEST(GeolibUtils, IsPointInsideArc_false) {
   // Calculate an arc
   const Units::NauticalMilesLength defined_radius(2.5);
   const LatitudeLongitudePoint center_point(Units::SignedDegreesAngle(33.3862), Units::SignedRadiansAngle(-111.887));
   const LatitudeLongitudePoint start_point =
         GeolibUtils::CalculateNewPoint(center_point, defined_radius, Units::SignedDegreesAngle(90));
   const LatitudeLongitudePoint end_point =
         GeolibUtils::CalculateNewPoint(center_point, defined_radius, Units::SignedDegreesAngle(0));
   const ArcOnEllipsoid arc_on_ellipsoid =
         GeolibUtils::CreateArcOnEllipsoid(start_point, end_point, center_point, ArcDirection::CLOCKWISE);
   const LatitudeLongitudePoint point_not_inside_arc = GeolibUtils::CalculateNewPoint(
         center_point, defined_radius + Units::MetersLength(1e-5), Units::SignedDegreesAngle(45));

   // This is the tested method ----
   bool is_inside_arc = GeolibUtils::IsPointInsideArcSegment(arc_on_ellipsoid, point_not_inside_arc);
   // ------------------------------

   // We expect this to be false
   EXPECT_FALSE(is_inside_arc);
}

TEST(ArcOnEllipsoid, IsPointInsideArc) {
   // Calculate an arc
   const Units::NauticalMilesLength defined_radius(2.5);
   const LatitudeLongitudePoint center_point(Units::SignedDegreesAngle(33.3862), Units::SignedRadiansAngle(-111.887));
   const LatitudeLongitudePoint start_point =
         GeolibUtils::CalculateNewPoint(center_point, defined_radius, Units::SignedDegreesAngle(90));
   const LatitudeLongitudePoint end_point =
         GeolibUtils::CalculateNewPoint(center_point, defined_radius, Units::SignedDegreesAngle(0));
   const ArcOnEllipsoid arc_on_ellipsoid =
         GeolibUtils::CreateArcOnEllipsoid(start_point, end_point, center_point, ArcDirection::CLOCKWISE);
   const LatitudeLongitudePoint point_inside_arc = GeolibUtils::CalculateNewPoint(
         center_point, defined_radius - Units::MetersLength(1e-5), Units::SignedDegreesAngle(45));

   // This is the tested method ----
   bool is_inside_arc = arc_on_ellipsoid.IsPointInsideArc(point_inside_arc);
   // ------------------------------

   // We expect this to be true
   EXPECT_TRUE(is_inside_arc);
}

TEST(GeolibUtils, FindNearestPointOnArc) {
   // Calculate an arc
   const Units::NauticalMilesLength defined_radius(2.5);
   const LatitudeLongitudePoint center_point(Units::SignedDegreesAngle(33.3862), Units::SignedRadiansAngle(-111.887));
   const LatitudeLongitudePoint start_point =
         GeolibUtils::CalculateNewPoint(center_point, defined_radius, Units::SignedDegreesAngle(90));
   const LatitudeLongitudePoint end_point =
         GeolibUtils::CalculateNewPoint(center_point, defined_radius, Units::SignedDegreesAngle(0));
   const ArcOnEllipsoid arc_on_ellipsoid =
         GeolibUtils::CreateArcOnEllipsoid(start_point, end_point, center_point, ArcDirection::CLOCKWISE);
   const LatitudeLongitudePoint point_on_arc_to_find =
         GeolibUtils::CalculateNewPoint(center_point, defined_radius, Units::SignedDegreesAngle(45));
   const LatitudeLongitudePoint point_outside_arc = GeolibUtils::CalculateNewPoint(
         center_point, defined_radius + Units::NauticalMilesLength(1), Units::SignedDegreesAngle(45));

   // This is the tested method ----
   std::pair<bool, const LatitudeLongitudePoint> ret =
         GeolibUtils::FindNearestPointOnArcUsingPerpendiculorProjection(arc_on_ellipsoid, point_outside_arc);
   // ------------------------------

   // Expect this to be true
   EXPECT_TRUE(ret.first);

   // Expect these to match
   EXPECT_NEAR(Units::RadiansAngle(point_on_arc_to_find.GetLatitude()).value(),
               Units::RadiansAngle(ret.second.GetLatitude()).value(), 1e-10);
   EXPECT_NEAR(Units::RadiansAngle(point_on_arc_to_find.GetLongitude()).value(),
               Units::RadiansAngle(ret.second.GetLongitude()).value(), 1e-10);
}

TEST(GeolibUtils, PointsAreMathematicallyEqual) {

   // Random location with lots of precision
   const LatitudeLongitudePoint point1(Units::SignedDegreesAngle(34.23423432341236),
                                       Units::SignedDegreesAngle(-112.1234123569987));
   const LatitudeLongitudePoint point2(point1);

   // This is the tested method -----------
   const bool is_equal = GeolibUtils::ArePointsMathematicallyEqual(point1, point2);
   // -------------------------------------

   EXPECT_TRUE(is_equal);
}

TEST(GeolibUtils, PointsAreNotMathematicallyEqual) {

   // Random location with lots of precision
   const LatitudeLongitudePoint point1(Units::SignedDegreesAngle(34.23423432341236),
                                       Units::SignedDegreesAngle(-112.1234123569987));
   const LatitudeLongitudePoint point_very_close_to_point1 =
         point1.ProjectDistanceAlongCourse(Units::MetersLength(1e-5), Units::DegreesAngle(2));

   // This is the tested method -----------
   const bool is_equal = GeolibUtils::ArePointsMathematicallyEqual(point1, point_very_close_to_point1);
   // -------------------------------------

   // They should not be equal
   EXPECT_FALSE(is_equal);
}

TEST(LatitudeLongitudePoint, PointsAreMathematicallyEqual) {

   // Random location with lots of precision
   const LatitudeLongitudePoint point1(Units::SignedDegreesAngle(34.23423432341236),
                                       Units::SignedDegreesAngle(-112.1234123569987));
   const LatitudeLongitudePoint point2(point1);

   // This is the tested method -----------
   const bool is_equal = point1.ArePointsEqual(point2);
   // -------------------------------------

   EXPECT_TRUE(is_equal);
}

TEST(LatitudeLongitudePoint, PointsAreNotMathematicallyEqual) {

   // Random location with lots of precision
   const LatitudeLongitudePoint point1(Units::SignedDegreesAngle(34.23423432341236),
                                       Units::SignedDegreesAngle(-112.1234123569987));
   const LatitudeLongitudePoint point_very_close_to_point1 =
         point1.ProjectDistanceAlongCourse(Units::MetersLength(1e-5), Units::DegreesAngle(2));

   // This is the tested method -----------
   const bool is_equal = point1.ArePointsEqual(point_very_close_to_point1);
   // -------------------------------------

   // They should not be equal
   EXPECT_FALSE(is_equal);
}

TEST(GeolibUtils, CreateArcTangentToTwoLines_valid_case) {
   /*
    * Use "real" data from in an operational AAESim scenario. This came from the
    * IM Test Vector work that Stuart did in 2020. These waypoints below are publicly published values
    * from a KDEN arrival.
    * HIMOM 39.788108 -104.808950 11000 0 210 0.78 0 11000 11000 200 200 0 0 0
      MCMUL 39.738073 -104.810275 10000 0 210 0.78 0 10000 10000 190 190 0 0 0
      KUGLN 39.691536 -104.752289 9000 0 170 0.78 0 9000 9000 190 190 2.8 39.737141 -104.751188
      BSAYN 39.739938 -104.688628 8000 0 170 0 0 9000 8000 170 170 2.9 39.74037993 -104.7518702
      CORDE 39.780965 -104.688147 7000 0 170 0 0 50000 7000 170 170 0 0 0
    */
   std::vector<std::tuple<bool, Waypoint, Units::Length>> waypoints_to_connect = {
         // tuple holds: bool for is_line, waypoint, rf_radius
         std::make_tuple(
               true,
               Waypoint("HIMOM", Units::DegreesAngle(39.788108), Units::DegreesAngle(-104.808950), Units::ZERO_LENGTH,
                        Units::ZERO_LENGTH, Units::ZERO_SPEED, Units::ZERO_LENGTH, Units::ZERO_SPEED),
               Units::infinity()),
         std::make_tuple(
               true,
               Waypoint("MCMUL", Units::DegreesAngle(39.738073), Units::DegreesAngle(-104.810275), Units::ZERO_LENGTH,
                        Units::ZERO_LENGTH, Units::ZERO_SPEED, Units::ZERO_LENGTH, Units::ZERO_SPEED),
               Units::infinity()),
         std::make_tuple(
               false,
               Waypoint("KUGLN", Units::DegreesAngle(39.691536), Units::DegreesAngle(-104.752289), Units::ZERO_LENGTH,
                        Units::ZERO_LENGTH, Units::ZERO_SPEED, Units::ZERO_LENGTH, Units::ZERO_SPEED),
               Units::NauticalMilesLength(2.8)),
         std::make_tuple(
               false,
               Waypoint("BSAYN", Units::DegreesAngle(39.739938), Units::DegreesAngle(-104.688628), Units::ZERO_LENGTH,
                        Units::ZERO_LENGTH, Units::ZERO_SPEED, Units::ZERO_LENGTH, Units::ZERO_SPEED),
               Units::NauticalMilesLength(2.9)),
         std::make_tuple(
               true,
               Waypoint("CORDE", Units::DegreesAngle(39.780965), Units::DegreesAngle(-104.688147), Units::ZERO_LENGTH,
                        Units::ZERO_LENGTH, Units::ZERO_SPEED, Units::ZERO_LENGTH, Units::ZERO_SPEED),
               Units::infinity()),
   };
   Waypoint prev_waypoint;
   Units::SignedAngle course_enu_tangency_end_of_prev_shape;
   std::vector<const ShapeOnEllipsoid *> simple_path_along_ground;
   for (auto item : waypoints_to_connect) {
      const bool is_line = std::get<0>(item);
      const Waypoint wpt = std::get<1>(item);
      const Units::Length radius = std::get<2>(item);

      if (wpt.GetName() != "HIMOM") {
         ShapeOnEllipsoid *shape;
         if (is_line) {
            LineOnEllipsoid tf_line =
                  LineOnEllipsoid::CreateFromPoints(LatitudeLongitudePoint::CreateFromWaypoint(prev_waypoint),
                                                    LatitudeLongitudePoint::CreateFromWaypoint(wpt));
            course_enu_tangency_end_of_prev_shape = tf_line.GetForwardCourseEnuAtEndPoint();
            shape = &tf_line;
         } else {
            const LatitudeLongitudePoint where_rf_began = LatitudeLongitudePoint::CreateFromWaypoint(prev_waypoint);
            const LatitudeLongitudePoint projected_pt_after_rf_began = where_rf_began.ProjectDistanceAlongCourse(
                  Units::NauticalMilesLength(100), course_enu_tangency_end_of_prev_shape);
            const LineOnEllipsoid fake_line_behind_rf_leg =
                  LineOnEllipsoid::CreateFromPoints(where_rf_began, projected_pt_after_rf_began);
            const LatitudeLongitudePoint where_rf_ends = LatitudeLongitudePoint::CreateFromWaypoint(wpt);
            std::tuple<LatitudeLongitudePoint, Units::SignedAngle, Units::Length> perp_info =
                  GeolibUtils::FindNearestPointOnLineUsingPerpendicularProjection(fake_line_behind_rf_leg,
                                                                                  where_rf_ends);
            const LatitudeLongitudePoint intersection_point_for_rf_outbound_leg = std::get<0>(perp_info);
            const LineOnEllipsoid fake_line_outbound_from_rf_leg =
                  LineOnEllipsoid::CreateFromPoints(intersection_point_for_rf_outbound_leg, where_rf_ends);
            const LineOnEllipsoid fake_line_outbound_from_rf_leg_extended =
                  fake_line_outbound_from_rf_leg.CreateExtendedLine(Units::NauticalMilesLength(100));

            // This is the tested method -----------
            std::pair<bool, ArcOnEllipsoid> data = GeolibUtils::CreateArcTangentToTwoLines(
                  fake_line_behind_rf_leg, fake_line_outbound_from_rf_leg_extended, radius);
            bool rf_leg_found = data.first;
            ArcOnEllipsoid rf_leg = data.second;
            // -------------------------------------

            // Test assertions ---------------------
            const double latitude_tolerance_degrees = 1e-2;
            const double longitude_tolerance_degrees = 1e-2;
            const double distance_tolerance_meters = 200;

            // Expect the arc to be found
            EXPECT_TRUE(rf_leg_found);

            // Expect that the arc's start point will be close to the previous waypoint
            EXPECT_NEAR(Units::DegreesAngle(where_rf_began.GetLatitude()).value(),
                        Units::DegreesAngle(rf_leg.GetStartPoint().GetLatitude()).value(), latitude_tolerance_degrees);
            EXPECT_NEAR(Units::DegreesAngle(where_rf_began.GetLongitude()).value(),
                        Units::DegreesAngle(rf_leg.GetStartPoint().GetLongitude()).value(),
                        longitude_tolerance_degrees);
            std::pair<Units::Length, Units::SignedAngle> return_info =
                  where_rf_began.CalculateRelationshipBetweenPoints(rf_leg.GetStartPoint());
            Units::MetersLength actual_distance = std::get<0>(return_info);
            EXPECT_LE(actual_distance.value(), distance_tolerance_meters);

            // Expect that the arc's end point will be close to the current waypoint
            EXPECT_NEAR(Units::DegreesAngle(where_rf_ends.GetLatitude()).value(),
                        Units::DegreesAngle(rf_leg.GetEndPoint().GetLatitude()).value(), latitude_tolerance_degrees);
            EXPECT_NEAR(Units::DegreesAngle(where_rf_ends.GetLongitude()).value(),
                        Units::DegreesAngle(rf_leg.GetEndPoint().GetLongitude()).value(), longitude_tolerance_degrees);
            return_info = where_rf_ends.CalculateRelationshipBetweenPoints(rf_leg.GetEndPoint());
            actual_distance = std::get<0>(return_info);
            EXPECT_LE(actual_distance.value(), distance_tolerance_meters);

            // Expect that the arc's radius will be mathematically close to the input radius
            EXPECT_NEAR(Units::NauticalMilesLength(radius).value(),
                        Units::NauticalMilesLength(rf_leg.GetRadius()).value(), 1e-10);
            // -------------------------------------

            // Setup for next iteration of for loop
            shape = &rf_leg;
            course_enu_tangency_end_of_prev_shape = rf_leg.GetCourseEnuTangentToEndPoint();
         }
         simple_path_along_ground.push_back(shape);
      }
      prev_waypoint = wpt;
   }

   // Clean up
   simple_path_along_ground.clear();
}

TEST(GeolibUtils, CreateArcInboundLineAndEndPoint_valid_case) {
   /*
    * Use "real" data from in an operational AAESim scenario. This came from the
    * IM Test Vector work that Stuart did in 2020. These waypoints below are publicly published values
    * from a KDEN arrival.
    * HIMOM 39.788108 -104.808950 11000 0 210 0.78 0 11000 11000 200 200 0 0 0
      MCMUL 39.738073 -104.810275 10000 0 210 0.78 0 10000 10000 190 190 0 0 0
      KUGLN 39.691536 -104.752289 9000 0 170 0.78 0 9000 9000 190 190 2.8 39.737141 -104.751188
      BSAYN 39.739938 -104.688628 8000 0 170 0 0 9000 8000 170 170 2.9 39.74037993 -104.7518702
      CORDE 39.780965 -104.688147 7000 0 170 0 0 50000 7000 170 170 0 0 0
    */
   std::vector<std::tuple<bool, Waypoint, Units::Length>> waypoints_to_connect = {
         // tuple holds: bool for is_line, waypoint, rf_radius
         std::make_tuple(
               true,
               Waypoint("HIMOM", Units::DegreesAngle(39.788108), Units::DegreesAngle(-104.808950), Units::ZERO_LENGTH,
                        Units::ZERO_LENGTH, Units::ZERO_SPEED, Units::ZERO_LENGTH, Units::ZERO_SPEED),
               Units::infinity()),
         std::make_tuple(
               true,
               Waypoint("MCMUL", Units::DegreesAngle(39.738073), Units::DegreesAngle(-104.810275), Units::ZERO_LENGTH,
                        Units::ZERO_LENGTH, Units::ZERO_SPEED, Units::ZERO_LENGTH, Units::ZERO_SPEED),
               Units::infinity()),
         std::make_tuple(
               false,
               Waypoint("KUGLN", Units::DegreesAngle(39.691536), Units::DegreesAngle(-104.752289), Units::ZERO_LENGTH,
                        Units::ZERO_LENGTH, Units::ZERO_SPEED, Units::ZERO_LENGTH, Units::ZERO_SPEED),
               Units::NauticalMilesLength(2.8)),
         std::make_tuple(
               false,
               Waypoint("BSAYN", Units::DegreesAngle(39.739938), Units::DegreesAngle(-104.688628), Units::ZERO_LENGTH,
                        Units::ZERO_LENGTH, Units::ZERO_SPEED, Units::ZERO_LENGTH, Units::ZERO_SPEED),
               Units::NauticalMilesLength(2.9)),
         std::make_tuple(
               true,
               Waypoint("CORDE", Units::DegreesAngle(39.780965), Units::DegreesAngle(-104.688147), Units::ZERO_LENGTH,
                        Units::ZERO_LENGTH, Units::ZERO_SPEED, Units::ZERO_LENGTH, Units::ZERO_SPEED),
               Units::infinity()),
   };
   Waypoint prev_waypoint;
   Units::SignedAngle course_enu_tangency_end_of_prev_shape;
   std::vector<const ShapeOnEllipsoid *> simple_path_along_ground;
   for (auto item : waypoints_to_connect) {
      const bool is_line = std::get<0>(item);
      const Waypoint wpt = std::get<1>(item);
      const Units::Length radius = std::get<2>(item);

      if (wpt.GetName() != "HIMOM") {
         ShapeOnEllipsoid *shape;
         if (is_line) {
            LineOnEllipsoid tf_line =
                  LineOnEllipsoid::CreateFromPoints(LatitudeLongitudePoint::CreateFromWaypoint(prev_waypoint),
                                                    LatitudeLongitudePoint::CreateFromWaypoint(wpt));
            course_enu_tangency_end_of_prev_shape = tf_line.GetForwardCourseEnuAtEndPoint();
            shape = &tf_line;
         } else {
            const LatitudeLongitudePoint where_rf_began = LatitudeLongitudePoint::CreateFromWaypoint(prev_waypoint);
            const LatitudeLongitudePoint where_rf_ends = LatitudeLongitudePoint::CreateFromWaypoint(wpt);
            const LatitudeLongitudePoint projected_start_point = where_rf_began.ProjectDistanceAlongCourse(
                  Units::NauticalMilesLength(100),
                  course_enu_tangency_end_of_prev_shape + Units::SignedRadiansAngle(180));
            const LineOnEllipsoid fake_inbound_line_to_rf_leg =
                  LineOnEllipsoid::CreateFromPoints(projected_start_point, where_rf_began);

            // This is the tested method -----------
            ArcOnEllipsoid rf_leg =
                  GeolibUtils::CreateArcFromInboundShapeAndEndPoint(&fake_inbound_line_to_rf_leg, where_rf_ends);
            // -------------------------------------

            // Test assertions ---------------------
            const double latitude_tolerance_degrees = 1e-8;
            const double longitude_tolerance_degrees = 1e-8;
            const double distance_tolerance_meters = 20;

            // Expect that the arc's start point will be close to the previous waypoint
            EXPECT_NEAR(Units::DegreesAngle(where_rf_began.GetLatitude()).value(),
                        Units::DegreesAngle(rf_leg.GetStartPoint().GetLatitude()).value(), latitude_tolerance_degrees);
            EXPECT_NEAR(Units::DegreesAngle(where_rf_began.GetLongitude()).value(),
                        Units::DegreesAngle(rf_leg.GetStartPoint().GetLongitude()).value(),
                        longitude_tolerance_degrees);
            std::pair<Units::Length, Units::SignedAngle> return_info =
                  where_rf_began.CalculateRelationshipBetweenPoints(rf_leg.GetStartPoint());
            Units::MetersLength actual_distance = std::get<0>(return_info);
            EXPECT_LE(actual_distance.value(), distance_tolerance_meters);

            // Expect that the arc's end point will be close to the current waypoint
            EXPECT_NEAR(Units::DegreesAngle(where_rf_ends.GetLatitude()).value(),
                        Units::DegreesAngle(rf_leg.GetEndPoint().GetLatitude()).value(), latitude_tolerance_degrees);
            EXPECT_NEAR(Units::DegreesAngle(where_rf_ends.GetLongitude()).value(),
                        Units::DegreesAngle(rf_leg.GetEndPoint().GetLongitude()).value(), longitude_tolerance_degrees);
            return_info = where_rf_ends.CalculateRelationshipBetweenPoints(rf_leg.GetEndPoint());
            actual_distance = std::get<0>(return_info);
            EXPECT_LE(actual_distance.value(), distance_tolerance_meters);

            // Expect that the arc's radius will be mathematically close to the input radius
            EXPECT_NEAR(Units::NauticalMilesLength(radius).value(),
                        Units::NauticalMilesLength(rf_leg.GetRadius()).value(), distance_tolerance_meters);
            // -------------------------------------

            // Setup for next iteration of for loop
            shape = &rf_leg;
            course_enu_tangency_end_of_prev_shape = rf_leg.GetCourseEnuTangentToEndPoint();
         }
         simple_path_along_ground.push_back(shape);
      }
      prev_waypoint = wpt;
   }

   // Clean up
   simple_path_along_ground.clear();
}

TEST(GeolibUtils, CreateArcInboundLineAndEndPoint_valid_case2) {
   /*
    * Use published data from Jeppesen 2012. This is an arrival into KPSP.
    * FERNN,33.97281666666667,-116.54705277777778,IF
      JEVOK,33.88010833333333,-116.46062500000001,TF
      CUXIT,33.95426944444445,-116.29662777777777,RF,4.72
      HOPLI,34.04785277777778,-116.33561388888889,TF
      YOCUL,34.111425000000004,-116.40140833333334,RF,6.83
      WASAK,34.09063888888889,-116.52534166666666,RF,5.00
    */
   std::vector<std::tuple<bool, Waypoint, Units::Length>> waypoints_to_connect = {
         // tuple holds: bool for is_line, waypoint, rf_radius
         std::make_tuple(true,
                         Waypoint("FERNN", Units::DegreesAngle(33.97281666666667),
                                  Units::DegreesAngle(-116.54705277777778), Units::ZERO_LENGTH, Units::ZERO_LENGTH,
                                  Units::ZERO_SPEED, Units::ZERO_LENGTH, Units::ZERO_SPEED),
                         Units::infinity()),
         std::make_tuple(true,
                         Waypoint("JEVOK", Units::DegreesAngle(33.88010833333333),
                                  Units::DegreesAngle(-116.46062500000001), Units::ZERO_LENGTH, Units::ZERO_LENGTH,
                                  Units::ZERO_SPEED, Units::ZERO_LENGTH, Units::ZERO_SPEED),
                         Units::infinity()),
         std::make_tuple(false,
                         Waypoint("CUSIT", Units::DegreesAngle(33.95426944444445),
                                  Units::DegreesAngle(-116.29662777777777), Units::ZERO_LENGTH, Units::ZERO_LENGTH,
                                  Units::ZERO_SPEED, Units::ZERO_LENGTH, Units::ZERO_SPEED),
                         Units::NauticalMilesLength(4.72)),
         std::make_tuple(true,
                         Waypoint("HOPLI", Units::DegreesAngle(34.04785277777778),
                                  Units::DegreesAngle(-116.33561388888889), Units::ZERO_LENGTH, Units::ZERO_LENGTH,
                                  Units::ZERO_SPEED, Units::ZERO_LENGTH, Units::ZERO_SPEED),
                         Units::infinity()),
         std::make_tuple(false,
                         Waypoint("YOCUL", Units::DegreesAngle(34.111425000000004),
                                  Units::DegreesAngle(-116.40140833333334), Units::ZERO_LENGTH, Units::ZERO_LENGTH,
                                  Units::ZERO_SPEED, Units::ZERO_LENGTH, Units::ZERO_SPEED),
                         Units::NauticalMilesLength(6.83)),
         std::make_tuple(false,
                         Waypoint("WASAK", Units::DegreesAngle(34.09063888888889),
                                  Units::DegreesAngle(-116.52534166666666), Units::ZERO_LENGTH, Units::ZERO_LENGTH,
                                  Units::ZERO_SPEED, Units::ZERO_LENGTH, Units::ZERO_SPEED),
                         Units::NauticalMilesLength(5.00)),
   };
   Waypoint prev_waypoint;
   Units::SignedAngle course_enu_tangency_end_of_prev_shape;
   std::vector<const ShapeOnEllipsoid *> simple_path_along_ground;
   for (auto item : waypoints_to_connect) {
      const bool is_line = std::get<0>(item);
      const Waypoint wpt = std::get<1>(item);
      const Units::Length radius = std::get<2>(item);

      if (wpt.GetName() != "FERNN") {
         ShapeOnEllipsoid *shape;
         if (is_line) {
            LineOnEllipsoid tf_line =
                  LineOnEllipsoid::CreateFromPoints(LatitudeLongitudePoint::CreateFromWaypoint(prev_waypoint),
                                                    LatitudeLongitudePoint::CreateFromWaypoint(wpt));
            course_enu_tangency_end_of_prev_shape = tf_line.GetForwardCourseEnuAtEndPoint();
            shape = &tf_line;
         } else {
            const LatitudeLongitudePoint where_rf_began = LatitudeLongitudePoint::CreateFromWaypoint(prev_waypoint);
            const LatitudeLongitudePoint where_rf_ends = LatitudeLongitudePoint::CreateFromWaypoint(wpt);
            const LatitudeLongitudePoint projected_start_point = where_rf_began.ProjectDistanceAlongCourse(
                  Units::NauticalMilesLength(100),
                  course_enu_tangency_end_of_prev_shape + Units::SignedRadiansAngle(180));
            const LineOnEllipsoid fake_inbound_line_to_rf_leg =
                  LineOnEllipsoid::CreateFromPoints(projected_start_point, where_rf_began);

            // This is the tested method -----------
            ArcOnEllipsoid rf_leg =
                  GeolibUtils::CreateArcFromInboundShapeAndEndPoint(&fake_inbound_line_to_rf_leg, where_rf_ends);
            // -------------------------------------

            // Test assertions ---------------------
            const double latitude_tolerance_degrees = 1e-8;
            const double longitude_tolerance_degrees = 1e-8;
            const double distance_tolerance_meters = 20;

            // Expect that the arc's start point will be close to the previous waypoint
            EXPECT_NEAR(Units::DegreesAngle(where_rf_began.GetLatitude()).value(),
                        Units::DegreesAngle(rf_leg.GetStartPoint().GetLatitude()).value(), latitude_tolerance_degrees);
            EXPECT_NEAR(Units::DegreesAngle(where_rf_began.GetLongitude()).value(),
                        Units::DegreesAngle(rf_leg.GetStartPoint().GetLongitude()).value(),
                        longitude_tolerance_degrees);
            std::pair<Units::Length, Units::SignedAngle> return_info =
                  where_rf_began.CalculateRelationshipBetweenPoints(rf_leg.GetStartPoint());
            Units::MetersLength actual_distance = std::get<0>(return_info);
            EXPECT_LE(actual_distance.value(), distance_tolerance_meters);

            // Expect that the arc's end point will be close to the current waypoint
            EXPECT_NEAR(Units::DegreesAngle(where_rf_ends.GetLatitude()).value(),
                        Units::DegreesAngle(rf_leg.GetEndPoint().GetLatitude()).value(), latitude_tolerance_degrees);
            EXPECT_NEAR(Units::DegreesAngle(where_rf_ends.GetLongitude()).value(),
                        Units::DegreesAngle(rf_leg.GetEndPoint().GetLongitude()).value(), longitude_tolerance_degrees);
            return_info = where_rf_ends.CalculateRelationshipBetweenPoints(rf_leg.GetEndPoint());
            actual_distance = std::get<0>(return_info);
            EXPECT_LE(actual_distance.value(), distance_tolerance_meters);

            // Expect that the arc's radius will be mathematically close to the input radius
            EXPECT_NEAR(Units::NauticalMilesLength(radius).value(),
                        Units::NauticalMilesLength(rf_leg.GetRadius()).value(), distance_tolerance_meters);
            // -------------------------------------

            // Setup for next iteration of for loop
            shape = &rf_leg;
            course_enu_tangency_end_of_prev_shape = rf_leg.GetCourseEnuTangentToEndPoint();
         }
         simple_path_along_ground.push_back(shape);
      }
      prev_waypoint = wpt;
   }

   // Clean up
   simple_path_along_ground.clear();
}

TEST(GeolibUtils, LineArcIntersection_valid_case) {
   /*
    * Use published data from Jeppesen 2012. This is an arrival into KPSP.
    * FERNN,33.97281666666667,-116.54705277777778,IF
      JEVOK,33.88010833333333,-116.46062500000001,TF
      CUXIT,33.95426944444445,-116.29662777777777,RF,4.72
      HOPLI,34.04785277777778,-116.33561388888889,TF
      YOCUL,34.111425000000004,-116.40140833333334,RF,6.83
      WASAK,34.09063888888889,-116.52534166666666,RF,5.00
    */
   std::vector<std::tuple<bool, Waypoint, Units::Length>> waypoints_to_connect = {
         // tuple holds: bool for is_line, waypoint, rf_radius
         std::make_tuple(true,
                         Waypoint("FERNN", Units::DegreesAngle(33.97281666666667),
                                  Units::DegreesAngle(-116.54705277777778), Units::ZERO_LENGTH, Units::ZERO_LENGTH,
                                  Units::ZERO_SPEED, Units::ZERO_LENGTH, Units::ZERO_SPEED),
                         Units::infinity()),
         std::make_tuple(true,
                         Waypoint("JEVOK", Units::DegreesAngle(33.88010833333333),
                                  Units::DegreesAngle(-116.46062500000001), Units::ZERO_LENGTH, Units::ZERO_LENGTH,
                                  Units::ZERO_SPEED, Units::ZERO_LENGTH, Units::ZERO_SPEED),
                         Units::infinity()),
         std::make_tuple(false,
                         Waypoint("CUSIT", Units::DegreesAngle(33.95426944444445),
                                  Units::DegreesAngle(-116.29662777777777), Units::ZERO_LENGTH, Units::ZERO_LENGTH,
                                  Units::ZERO_SPEED, Units::ZERO_LENGTH, Units::ZERO_SPEED),
                         Units::NauticalMilesLength(4.72)),
         std::make_tuple(true,
                         Waypoint("HOPLI", Units::DegreesAngle(34.04785277777778),
                                  Units::DegreesAngle(-116.33561388888889), Units::ZERO_LENGTH, Units::ZERO_LENGTH,
                                  Units::ZERO_SPEED, Units::ZERO_LENGTH, Units::ZERO_SPEED),
                         Units::infinity()),
         std::make_tuple(false,
                         Waypoint("YOCUL", Units::DegreesAngle(34.111425000000004),
                                  Units::DegreesAngle(-116.40140833333334), Units::ZERO_LENGTH, Units::ZERO_LENGTH,
                                  Units::ZERO_SPEED, Units::ZERO_LENGTH, Units::ZERO_SPEED),
                         Units::NauticalMilesLength(6.83)),
         std::make_tuple(false,
                         Waypoint("WASAK", Units::DegreesAngle(34.09063888888889),
                                  Units::DegreesAngle(-116.52534166666666), Units::ZERO_LENGTH, Units::ZERO_LENGTH,
                                  Units::ZERO_SPEED, Units::ZERO_LENGTH, Units::ZERO_SPEED),
                         Units::NauticalMilesLength(5.00)),
   };
   Waypoint prev_waypoint;
   Units::SignedAngle course_enu_tangency_end_of_prev_shape;
   std::vector<const ShapeOnEllipsoid *> simple_path_along_ground;
   for (auto item : waypoints_to_connect) {
      const bool is_line = std::get<0>(item);
      const Waypoint wpt = std::get<1>(item);

      if (wpt.GetName() != "FERNN") {
         ShapeOnEllipsoid *shape;
         if (is_line) {
            LineOnEllipsoid tf_line =
                  LineOnEllipsoid::CreateFromPoints(LatitudeLongitudePoint::CreateFromWaypoint(prev_waypoint),
                                                    LatitudeLongitudePoint::CreateFromWaypoint(wpt));
            course_enu_tangency_end_of_prev_shape = tf_line.GetForwardCourseEnuAtEndPoint();
            shape = &tf_line;
         } else {
            const LatitudeLongitudePoint where_rf_began = LatitudeLongitudePoint::CreateFromWaypoint(prev_waypoint);
            const LatitudeLongitudePoint where_rf_ends = LatitudeLongitudePoint::CreateFromWaypoint(wpt);
            const LatitudeLongitudePoint projected_start_point = where_rf_began.ProjectDistanceAlongCourse(
                  Units::NauticalMilesLength(100),
                  course_enu_tangency_end_of_prev_shape + Units::SignedRadiansAngle(180));
            const LineOnEllipsoid fake_inbound_line_to_rf_leg =
                  LineOnEllipsoid::CreateFromPoints(projected_start_point, where_rf_began);
            ArcOnEllipsoid rf_leg =
                  GeolibUtils::CreateArcFromInboundShapeAndEndPoint(&fake_inbound_line_to_rf_leg, where_rf_ends);

            // This is the tested method -----------
            std::vector<std::pair<bool, LatitudeLongitudePoint>> intersection_data =
                  GeolibUtils::CalculateLineArcIntersectionPoints(fake_inbound_line_to_rf_leg, rf_leg);
            // -------------------------------------

            // Test assertions ---------------------
            const double latitude_tolerance_degrees = 1e-8;
            const double longitude_tolerance_degrees = 1e-8;

            for (auto pr : intersection_data) {
               // Expect that an intersection was found and is on both shapes
               EXPECT_TRUE(pr.first);

               // Expect that the intersection point will be close to the arc's start point
               const LatitudeLongitudePoint intersection_point = pr.second;
               EXPECT_NEAR(Units::DegreesAngle(intersection_point.GetLatitude()).value(),
                           Units::DegreesAngle(rf_leg.GetStartPoint().GetLatitude()).value(),
                           latitude_tolerance_degrees);
               EXPECT_NEAR(Units::DegreesAngle(intersection_point.GetLongitude()).value(),
                           Units::DegreesAngle(rf_leg.GetStartPoint().GetLongitude()).value(),
                           longitude_tolerance_degrees);
            }
            // -------------------------------------

            // Setup for next iteration of for loop
            shape = &rf_leg;
            course_enu_tangency_end_of_prev_shape = rf_leg.GetCourseEnuTangentToEndPoint();
         }
         simple_path_along_ground.push_back(shape);
      }
      prev_waypoint = wpt;
   }

   // Clean up
   simple_path_along_ground.clear();
}

TEST(LineOnEllipsoid, GetRelativeDirection) {
   // Output stream that handles writing when object is destroyed
   const std::vector<Waypoint> start_points = {
         Waypoint("kphx_airport", Units::DegreesAngle(33.4342778), Units::DegreesAngle(-112.0115833),
                  Units::ZERO_LENGTH, Units::ZERO_LENGTH, Units::ZERO_SPEED, Units::ZERO_LENGTH, Units::ZERO_SPEED),
         Waypoint("ksfo_airport", Units::DegreesAngle(37.6188056), Units::DegreesAngle(-122.3754167),
                  Units::ZERO_LENGTH, Units::ZERO_LENGTH, Units::ZERO_SPEED, Units::ZERO_LENGTH, Units::ZERO_SPEED),
         Waypoint("kden_airport", Units::DegreesAngle(39.8616667), Units::DegreesAngle(-104.6731667),
                  Units::ZERO_LENGTH, Units::ZERO_LENGTH, Units::ZERO_SPEED, Units::ZERO_LENGTH, Units::ZERO_SPEED),
         Waypoint("ksfo_airport", Units::DegreesAngle(42.3629444), Units::DegreesAngle(-71.0063889), Units::ZERO_LENGTH,
                  Units::ZERO_LENGTH, Units::ZERO_SPEED, Units::ZERO_LENGTH, Units::ZERO_SPEED),
         Waypoint("katl_airport", Units::DegreesAngle(33.6366996), Units::DegreesAngle(-84.4278640), Units::ZERO_LENGTH,
                  Units::ZERO_LENGTH, Units::ZERO_SPEED, Units::ZERO_LENGTH, Units::ZERO_SPEED)};

   const Units::MetersLength distance_off_line(1e-5);  // this should be small to test navigation situations that are
                                                       // "very close" to the line being followed
   for (const Waypoint wpt : start_points) {
      const LatitudeLongitudePoint start_point = LatitudeLongitudePoint::CreateFromWaypoint(wpt);

      for (double crs_ned = 0; crs_ned <= 2 * M_PI; crs_ned += M_PI / 20) {
         const Units::SignedRadiansAngle course_enu =
               GeolibUtils::ConvertCourseFromNedToEnu(Units::UnsignedRadiansAngle(crs_ned));
         const LatitudeLongitudePoint end_point =
               start_point.ProjectDistanceAlongCourse(Units::NauticalMilesLength(100), course_enu);
         const LineOnEllipsoid line_on_ellipsoid = LineOnEllipsoid::CreateFromPoints(start_point, end_point);

         // Test: point on line result
         const LatitudeLongitudePoint test_point_actually_on_line =
               start_point.ProjectDistanceAlongCourse(Units::NauticalMilesLength(1), course_enu);
         // This is the tested method -----------
         ShapeOnEllipsoid::DIRECTION_RELATIVE_TO_SHAPE actual_dir1 =
               line_on_ellipsoid.GetRelativeDirection(test_point_actually_on_line);
         EXPECT_TRUE(actual_dir1 == ShapeOnEllipsoid::ON_SHAPE);
         // -------------------------------------

         // Test: point to left result
         const LatitudeLongitudePoint test_point_actually_left_of_line =
               test_point_actually_on_line.ProjectDistanceAlongCourse(
                     distance_off_line, GeolibUtils::ConvertCourseFromNedToEnu(Units::UnsignedRadiansAngle(crs_ned) -
                                                                               Units::RadiansAngle(M_PI_2)));
         // This is the tested method -----------
         ShapeOnEllipsoid::DIRECTION_RELATIVE_TO_SHAPE actual_dir2 =
               line_on_ellipsoid.GetRelativeDirection(test_point_actually_left_of_line);
         EXPECT_TRUE(actual_dir2 == ShapeOnEllipsoid::LEFT_OF_SHAPE);
         // -------------------------------------

         // Test: point to right result
         const LatitudeLongitudePoint test_point_actually_right_of_line =
               test_point_actually_on_line.ProjectDistanceAlongCourse(
                     distance_off_line, GeolibUtils::ConvertCourseFromNedToEnu(Units::UnsignedRadiansAngle(crs_ned) +
                                                                               Units::RadiansAngle(M_PI_2)));
         // This is the tested method -----------
         ShapeOnEllipsoid::DIRECTION_RELATIVE_TO_SHAPE actual_dir3 =
               line_on_ellipsoid.GetRelativeDirection(test_point_actually_right_of_line);
         EXPECT_TRUE(actual_dir3 == ShapeOnEllipsoid::RIGHT_OF_SHAPE);
         // -------------------------------------
      }
   }
}

TEST(EarthModel, ToUnitVector) {
   EarthModel::AbsolutePositionEcef test_vector, unit_vector;
   test_vector.x = Units::MetersLength(3);
   test_vector.y = Units::MetersLength(4);
   test_vector.z = Units::MetersLength(5);
   // Test method---------------------------
   unit_vector = test_vector.ToUnitVector();
   // --------------------------------------

   const Units::MetersLength unity(1);
   EXPECT_NEAR(unity.value(),
               Units::MetersLength(
                     Units::sqrt(Units::sqr(unit_vector.x) + Units::sqr(unit_vector.y) + Units::sqr(unit_vector.z)))
                     .value(),
               1e-10);
}

TEST(ArcOnEllipsoid, GetRelativeDirection) {
   const std::vector<Waypoint> center_points = {
         Waypoint("kphx_airport", Units::DegreesAngle(33.4342778), Units::DegreesAngle(-112.0115833),
                  Units::ZERO_LENGTH, Units::ZERO_LENGTH, Units::ZERO_SPEED, Units::ZERO_LENGTH, Units::ZERO_SPEED),
         Waypoint("ksfo_airport", Units::DegreesAngle(37.6188056), Units::DegreesAngle(-122.3754167),
                  Units::ZERO_LENGTH, Units::ZERO_LENGTH, Units::ZERO_SPEED, Units::ZERO_LENGTH, Units::ZERO_SPEED),
         Waypoint("kden_airport", Units::DegreesAngle(39.8616667), Units::DegreesAngle(-104.6731667),
                  Units::ZERO_LENGTH, Units::ZERO_LENGTH, Units::ZERO_SPEED, Units::ZERO_LENGTH, Units::ZERO_SPEED),
         Waypoint("ksfo_airport", Units::DegreesAngle(42.3629444), Units::DegreesAngle(-71.0063889), Units::ZERO_LENGTH,
                  Units::ZERO_LENGTH, Units::ZERO_SPEED, Units::ZERO_LENGTH, Units::ZERO_SPEED),
         Waypoint("katl_airport", Units::DegreesAngle(33.6366996), Units::DegreesAngle(-84.4278640), Units::ZERO_LENGTH,
                  Units::ZERO_LENGTH, Units::ZERO_SPEED, Units::ZERO_LENGTH, Units::ZERO_SPEED)};

   const Units::MetersLength distance_off_arc(1e-5);  // this should be small to test navigation situations that are
                                                      // "very close" to the shape being followed
   const Units::NauticalMilesLength arc_radius_approximate(5);
   const geolib_idealab::ArcDirection arc_directions[2] = {geolib_idealab::ArcDirection::CLOCKWISE,
                                                           geolib_idealab::ArcDirection::COUNTERCLOCKWISE};

   for (const Waypoint wpt : center_points) {
      const LatitudeLongitudePoint center_point_approximate = LatitudeLongitudePoint::CreateFromWaypoint(wpt);

      for (geolib_idealab::ArcDirection arc_direction : arc_directions) {

         for (double arc_start_course_ned = 3 * M_PI / 2; arc_start_course_ned <= 2 * M_PI;
              arc_start_course_ned += M_PI / 2) {
            const Units::SignedRadiansAngle arc_start_course_enu =
                  GeolibUtils::ConvertCourseFromNedToEnu(Units::UnsignedRadiansAngle(arc_start_course_ned));
            LatitudeLongitudePoint line_end_point =
                  center_point_approximate.ProjectDistanceAlongCourse(arc_radius_approximate, arc_start_course_enu);
            LatitudeLongitudePoint line_start_point, arc_end_point;
            if (arc_direction == geolib_idealab::ArcDirection::CLOCKWISE) {
               line_start_point = line_end_point.ProjectDistanceAlongCourse(
                     arc_radius_approximate, arc_start_course_enu + Units::PI_RADIANS_ANGLE / 2);
               arc_end_point = center_point_approximate.ProjectDistanceAlongCourse(
                     arc_radius_approximate, arc_start_course_enu - Units::PI_RADIANS_ANGLE / 2);
            } else {
               line_start_point = line_end_point.ProjectDistanceAlongCourse(
                     arc_radius_approximate, arc_start_course_enu - Units::PI_RADIANS_ANGLE / 2);
               arc_end_point = center_point_approximate.ProjectDistanceAlongCourse(
                     arc_radius_approximate, arc_start_course_enu + Units::PI_RADIANS_ANGLE / 2);
            }
            const LineOnEllipsoid inbound_line = LineOnEllipsoid::CreateFromPoints(line_start_point, line_end_point);
            const ArcOnEllipsoid arc_on_ellipsoid =
                  GeolibUtils::CreateArcFromInboundShapeAndEndPoint(&inbound_line, arc_end_point);
            EXPECT_TRUE(arc_on_ellipsoid.GetArcDirection() == arc_direction);

            // Test: point on arc result
            LatitudeLongitudePoint test_point_actually_on_arc;
            if (arc_direction == geolib_idealab::ArcDirection::CLOCKWISE) {
               test_point_actually_on_arc = arc_on_ellipsoid.GetCenterPoint().ProjectDistanceAlongCourse(
                     arc_on_ellipsoid.GetRadius(), arc_start_course_enu - Units::PI_RADIANS_ANGLE / 4);
            } else {
               test_point_actually_on_arc = arc_on_ellipsoid.GetCenterPoint().ProjectDistanceAlongCourse(
                     arc_on_ellipsoid.GetRadius(), arc_start_course_enu + Units::PI_RADIANS_ANGLE / 4);
            }
            // This is the tested method -----------
            ShapeOnEllipsoid::DIRECTION_RELATIVE_TO_SHAPE actual_dir1 =
                  arc_on_ellipsoid.GetRelativeDirection(test_point_actually_on_arc);
            EXPECT_TRUE(actual_dir1 == ShapeOnEllipsoid::ON_SHAPE);

            // Test: point to left result
            LatitudeLongitudePoint test_point_actually_left_of_line;
            ShapeOnEllipsoid::DIRECTION_RELATIVE_TO_SHAPE actual_dir2;
            if (arc_direction == geolib_idealab::ArcDirection::CLOCKWISE) {
               test_point_actually_left_of_line = arc_on_ellipsoid.GetCenterPoint().ProjectDistanceAlongCourse(
                     arc_on_ellipsoid.GetRadius() + distance_off_arc,
                     GeolibUtils::ConvertCourseFromNedToEnu(
                           Units::UnsignedRadiansAngle(arc_start_course_ned + M_PI_4)));
               // This is the tested method -----------
               actual_dir2 = arc_on_ellipsoid.GetRelativeDirection(test_point_actually_left_of_line);
               EXPECT_TRUE(actual_dir2 == ShapeOnEllipsoid::LEFT_OF_SHAPE);
               // -------------------------------------
            } else {
               test_point_actually_left_of_line = arc_on_ellipsoid.GetCenterPoint().ProjectDistanceAlongCourse(
                     arc_on_ellipsoid.GetRadius() - distance_off_arc,
                     GeolibUtils::ConvertCourseFromNedToEnu(
                           Units::UnsignedRadiansAngle(arc_start_course_ned - M_PI_4)));
               // This is the tested method -----------
               actual_dir2 = arc_on_ellipsoid.GetRelativeDirection(test_point_actually_left_of_line);
               EXPECT_TRUE(actual_dir2 == ShapeOnEllipsoid::LEFT_OF_SHAPE);
               // -------------------------------------
            }

            // Test: point to right result
            LatitudeLongitudePoint test_point_actually_right_of_line;
            ShapeOnEllipsoid::DIRECTION_RELATIVE_TO_SHAPE actual_dir3;
            if (arc_direction == geolib_idealab::ArcDirection::CLOCKWISE) {
               test_point_actually_right_of_line = arc_on_ellipsoid.GetCenterPoint().ProjectDistanceAlongCourse(
                     arc_on_ellipsoid.GetRadius() - distance_off_arc,
                     GeolibUtils::ConvertCourseFromNedToEnu(
                           Units::UnsignedRadiansAngle(arc_start_course_ned + M_PI_4)));
               // This is the tested method -----------
               actual_dir3 = arc_on_ellipsoid.GetRelativeDirection(test_point_actually_right_of_line);
               EXPECT_TRUE(actual_dir3 == ShapeOnEllipsoid::RIGHT_OF_SHAPE);
               // -------------------------------------
            } else {
               test_point_actually_right_of_line = arc_on_ellipsoid.GetCenterPoint().ProjectDistanceAlongCourse(
                     arc_on_ellipsoid.GetRadius() + distance_off_arc,
                     GeolibUtils::ConvertCourseFromNedToEnu(
                           Units::UnsignedRadiansAngle(arc_start_course_ned - M_PI_4)));
               // This is the tested method -----------
               actual_dir3 = arc_on_ellipsoid.GetRelativeDirection(test_point_actually_right_of_line);
               EXPECT_TRUE(actual_dir3 == ShapeOnEllipsoid::RIGHT_OF_SHAPE);
               // -------------------------------------
            }
         }
      }
   }
}

TEST(LineOnEllipsoid, GetDistanceToEndPoint) {
   // Test 1: point that is on line already--------
   const LatitudeLongitudePoint zero_zero = LatitudeLongitudePoint(Units::ZERO_ANGLE, Units::ZERO_ANGLE);
   const LatitudeLongitudePoint one_zero = LatitudeLongitudePoint(Units::SignedDegreesAngle(1), Units::ZERO_ANGLE);
   const LineOnEllipsoid test_line_north = LineOnEllipsoid::CreateFromPoints(zero_zero, one_zero);
   const LatitudeLongitudePoint test_point_on_line = zero_zero.ProjectDistanceAlongCourse(
         test_line_north.GetShapeLength() / 2, test_line_north.GetForwardCourseEnuAtStartPoint());
   const Units::MetersLength expected_distance_to_end_point = test_line_north.GetShapeLength() / 2;

   // This is the test method ------------------
   const Units::MetersLength actual_distance_to_end_point_1 = test_line_north.GetDistanceToEndPoint(test_point_on_line);
   // ------------------------------------------
   EXPECT_NEAR(expected_distance_to_end_point.value(), actual_distance_to_end_point_1.value(), 1e-9);

   // Test 2: point not on line-----------------
   const LatitudeLongitudePoint test_point_not_on_line = test_point_on_line.ProjectDistanceAlongCourse(
         Units::NauticalMilesLength(1),
         test_line_north.GetForwardCourseEnuAtStartPoint() + Units::PI_RADIANS_ANGLE / 2);

   // This is the test method ------------------
   const Units::MetersLength actual_distance_to_end_point_2 =
         test_line_north.GetDistanceToEndPoint(test_point_not_on_line);
   // ------------------------------------------
   EXPECT_NEAR(expected_distance_to_end_point.value(), actual_distance_to_end_point_2.value(), 1e-9);
}

TEST(ArcOnEllipsoid, GetDistanceToEndPoint) {
   // Test 1: point that is on arc--------
   const LatitudeLongitudePoint one_zero = LatitudeLongitudePoint(Units::SignedDegreesAngle(1), Units::ZERO_ANGLE);
   const LatitudeLongitudePoint inbound_line_start_point =
         one_zero.ProjectDistanceAlongCourse(Units::NauticalMilesLength(1), Units::PI_RADIANS_ANGLE);
   const LineOnEllipsoid inbound_line_to_arc = LineOnEllipsoid::CreateFromPoints(inbound_line_start_point, one_zero);
   const LatitudeLongitudePoint arc_end_point = one_zero.ProjectDistanceAlongCourse(
         Units::NauticalMilesLength(5),
         inbound_line_to_arc.GetForwardCourseEnuAtEndPoint() - Units::PI_RADIANS_ANGLE / 2);
   const ArcOnEllipsoid arc_on_ellipsoid =
         GeolibUtils::CreateArcFromInboundShapeAndEndPoint(&inbound_line_to_arc, arc_end_point);
   LatitudeLongitudePoint test_point_on_arc;
   if (arc_on_ellipsoid.GetArcDirection() == geolib_idealab::ArcDirection::CLOCKWISE) {
      test_point_on_arc = arc_on_ellipsoid.GetCenterPoint().ProjectDistanceAlongCourse(
            arc_on_ellipsoid.GetRadius(),
            arc_on_ellipsoid.GetStartAzimuthEnu() + arc_on_ellipsoid.GetArcAngularExtent() / 2);
   } else {
      test_point_on_arc = arc_on_ellipsoid.GetCenterPoint().ProjectDistanceAlongCourse(
            arc_on_ellipsoid.GetRadius(),
            arc_on_ellipsoid.GetStartAzimuthEnu() - arc_on_ellipsoid.GetArcAngularExtent() / 2);
   }
   const Units::MetersLength expected_distance_to_end_point = arc_on_ellipsoid.GetShapeLength() / 2;

   // This is the test method ------------------
   const Units::MetersLength actual_distance_to_end_point_1 = arc_on_ellipsoid.GetDistanceToEndPoint(test_point_on_arc);
   // ------------------------------------------
   EXPECT_NEAR(expected_distance_to_end_point.value(), actual_distance_to_end_point_1.value(), 1e-8);

   // Test 2: point not on line-----------------
   LatitudeLongitudePoint test_point_not_on_line;
   if (arc_on_ellipsoid.GetArcDirection() == geolib_idealab::ArcDirection::CLOCKWISE) {
      test_point_not_on_line = arc_on_ellipsoid.GetCenterPoint().ProjectDistanceAlongCourse(
            arc_on_ellipsoid.GetRadius() + Units::NauticalMilesLength(1),
            arc_on_ellipsoid.GetStartAzimuthEnu() + arc_on_ellipsoid.GetArcAngularExtent() / 2);
   } else {
      test_point_not_on_line = arc_on_ellipsoid.GetCenterPoint().ProjectDistanceAlongCourse(
            arc_on_ellipsoid.GetRadius() - Units::NauticalMilesLength(1),
            arc_on_ellipsoid.GetStartAzimuthEnu() - arc_on_ellipsoid.GetArcAngularExtent() / 2);
   }

   // This is the test method ------------------
   const Units::MetersLength actual_distance_to_end_point_2 =
         arc_on_ellipsoid.GetDistanceToEndPoint(test_point_not_on_line);
   // ------------------------------------------
   EXPECT_NEAR(expected_distance_to_end_point.value(), actual_distance_to_end_point_2.value(), 1e-8);
}

TEST(EarthModel, VectorDotProduct) {
   EarthModel::AbsolutePositionEcef one_zero_zero, zero_one_zero;
   one_zero_zero.x = Units::MetersLength(1);
   one_zero_zero.y = Units::ZERO_LENGTH;
   one_zero_zero.z = Units::ZERO_LENGTH;
   zero_one_zero.x = Units::ZERO_LENGTH;
   zero_one_zero.y = one_zero_zero.x;
   zero_one_zero.z = Units::ZERO_LENGTH;
   EXPECT_NEAR(0, Units::MetersLength(VectorDotProduct(one_zero_zero, zero_one_zero)).value(), 1e-10);
}

TEST(EarthModel, VectorCrossProduct) {
   EarthModel::AbsolutePositionEcef one_zero_zero, zero_one_zero;
   one_zero_zero.x = Units::MetersLength(1);
   one_zero_zero.y = Units::ZERO_LENGTH;
   one_zero_zero.z = Units::ZERO_LENGTH;
   zero_one_zero.x = Units::ZERO_LENGTH;
   zero_one_zero.y = one_zero_zero.x;
   zero_one_zero.z = Units::ZERO_LENGTH;

   // Cross [1,0,0] and [0,1,0]
   EarthModel::AbsolutePositionEcef actual_cp_result_test1 = VectorCrossProduct(one_zero_zero, zero_one_zero);

   EarthModel::AbsolutePositionEcef expect_zero_zero_one;
   expect_zero_zero_one.x = Units::ZERO_LENGTH;
   expect_zero_zero_one.y = Units::ZERO_LENGTH;
   expect_zero_zero_one.z = Units::MetersLength(1);

   EXPECT_NEAR(expect_zero_zero_one.x.value(), Units::MetersLength(actual_cp_result_test1.x).value(), 1e-10);
   EXPECT_NEAR(expect_zero_zero_one.y.value(), Units::MetersLength(actual_cp_result_test1.y).value(), 1e-10);
   EXPECT_NEAR(expect_zero_zero_one.z.value(), Units::MetersLength(actual_cp_result_test1.z).value(), 1e-10);

   // Cross [0,1,0] and [1,0,0]
   EarthModel::AbsolutePositionEcef actual_cp_result_test2 = VectorCrossProduct(zero_one_zero, one_zero_zero);

   EarthModel::AbsolutePositionEcef expect_zero_zero_negative_one;
   expect_zero_zero_negative_one.x = Units::ZERO_LENGTH;
   expect_zero_zero_negative_one.y = Units::ZERO_LENGTH;
   expect_zero_zero_negative_one.z = Units::MetersLength(-1);

   EXPECT_NEAR(expect_zero_zero_negative_one.x.value(), Units::MetersLength(actual_cp_result_test2.x).value(), 1e-10);
   EXPECT_NEAR(expect_zero_zero_negative_one.y.value(), Units::MetersLength(actual_cp_result_test2.y).value(), 1e-10);
   EXPECT_NEAR(expect_zero_zero_negative_one.z.value(), Units::MetersLength(actual_cp_result_test2.z).value(), 1e-10);
}

TEST(GeolibUtils, IsSuccess_ReturnsFalse) {
   // Define two lines that are guaranteed -not- to intersect
   const LatitudeLongitudePoint start_point1(Units::DegreesAngle(38.0), Units::DegreesAngle(-77.3));
   const LatitudeLongitudePoint end_point1 =
         GeolibUtils::CalculateNewPoint(start_point1, Units::NauticalMilesLength(.1), Units::DegreesAngle(0));
   const LineOnEllipsoid line1 = LineOnEllipsoid::CreateFromPoints(start_point1, end_point1);
   const LatitudeLongitudePoint start_point2(Units::DegreesAngle(40.1), Units::DegreesAngle(-112.8));
   const LatitudeLongitudePoint end_point2 =
         GeolibUtils::CalculateNewPoint(start_point2, Units::NauticalMilesLength(.5), Units::DegreesAngle(10));
   const LineOnEllipsoid line2 = LineOnEllipsoid::CreateFromPoints(start_point2, end_point2);

   double crs31, distance_line1_start_to_intx_point, crs32, distance_line2_start_to_intx_point;
   geolib_idealab::LLPoint intersection_point;
   ErrorSet error_set =
         geoIntx(line1.GetStartPoint().GetGeolibPrimitiveLLPoint(), line1.GetEndPoint().GetGeolibPrimitiveLLPoint(),
                 line1.GetLineType(), &crs31, &distance_line1_start_to_intx_point,
                 line2.GetStartPoint().GetGeolibPrimitiveLLPoint(), line2.GetEndPoint().GetGeolibPrimitiveLLPoint(),
                 line2.GetLineType(), &crs32, &distance_line2_start_to_intx_point, &intersection_point,
                 GEOLIB_TOLERANCE, GEOLIB_EPSILON);

   EXPECT_FALSE(GeolibUtils::IsSuccess(error_set));
   EXPECT_TRUE(GeolibUtils::HasErrorBitSet(error_set, geolib_idealab::ErrorCodes::NO_INTERSECTION_ERR));
}

TEST(GeolibUtils, IsSuccess_ReturnsTrue) {
   // Define two lines that are guaranteed to intersect
   const LatitudeLongitudePoint start_point1(Units::DegreesAngle(38.0), Units::DegreesAngle(-77.3));
   const LatitudeLongitudePoint end_point1 =
         GeolibUtils::CalculateNewPoint(start_point1, Units::NauticalMilesLength(1), Units::DegreesAngle(0));
   const LineOnEllipsoid line1 = LineOnEllipsoid::CreateFromPoints(start_point1, end_point1);
   const LatitudeLongitudePoint start_point2 =
         line1.CalculatePointAtDistanceFromStartPoint(Units::NauticalMilesLength(.1));
   const LatitudeLongitudePoint end_point2 =
         GeolibUtils::CalculateNewPoint(start_point2, Units::NauticalMilesLength(.5), Units::DegreesAngle(10));
   const LineOnEllipsoid line2 = LineOnEllipsoid::CreateFromPoints(start_point2, end_point2);

   double crs31, distance_line1_start_to_intx_point, crs32, distance_line2_start_to_intx_point;
   geolib_idealab::LLPoint intersection_point;
   ErrorSet error_set =
         geoIntx(line1.GetStartPoint().GetGeolibPrimitiveLLPoint(), line1.GetEndPoint().GetGeolibPrimitiveLLPoint(),
                 line1.GetLineType(), &crs31, &distance_line1_start_to_intx_point,
                 line2.GetStartPoint().GetGeolibPrimitiveLLPoint(), line2.GetEndPoint().GetGeolibPrimitiveLLPoint(),
                 line2.GetLineType(), &crs32, &distance_line2_start_to_intx_point, &intersection_point,
                 GEOLIB_TOLERANCE, GEOLIB_EPSILON);

   EXPECT_TRUE(GeolibUtils::IsSuccess(error_set));
   EXPECT_FALSE(GeolibUtils::HasErrorBitSet(error_set, geolib_idealab::ErrorCodes::NO_INTERSECTION_ERR));
}

TEST(GeolibUtils, HasErrorBitSet_SUCCESS) {
   const ErrorSet error_set{geolib_idealab::ErrorCodes::SUCCESS};
   EXPECT_TRUE(GeolibUtils::HasErrorBitSet(error_set, geolib_idealab::ErrorCodes::SUCCESS));
}

TEST(GeolibUtils, HasErrorBitSet) {
   const ErrorSet error_set{geolib_idealab::ErrorCodes::NO_INTERSECTION_ERR};
   EXPECT_TRUE(GeolibUtils::HasErrorBitSet(error_set, geolib_idealab::ErrorCodes::NO_INTERSECTION_ERR));
}

TEST(GeolibUtils, HasErrorBitSet_Combo) {
   ErrorSet error_set{geolib_idealab::ErrorCodes::NO_INTERSECTION_ERR};
   error_set |= geolib_idealab::ErrorCodes::ERROR_MAX_REACHED_ERR;
   EXPECT_TRUE(GeolibUtils::HasErrorBitSet(error_set, geolib_idealab::ErrorCodes::NO_INTERSECTION_ERR));
   EXPECT_TRUE(GeolibUtils::HasErrorBitSet(error_set, geolib_idealab::ErrorCodes::ERROR_MAX_REACHED_ERR));
}

}  // namespace open_source
}  // namespace test
}  // namespace aaesim