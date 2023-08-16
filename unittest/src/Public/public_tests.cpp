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

#include "public/CustomMath.h"
#include "public/AircraftCalculations.h"
#include "public/AircraftIntent.h"
#include "public/AlongPathDistanceCalculator.h"
#include "public/CoreUtils.h"
#include "public/DirectionOfFlightCourseCalculator.h"
#include "public/FlightEnvelopeSpeedLimiter.h"
#include "public/Guidance.h"
#include "public/HorizontalPathTracker.h"
#include "public/VectorDifferenceWindEvaluator.h"
#include "public/PositionCalculator.h"
#include "public/ScenarioUtils.h"
#include "public/SimulationTime.h"
#include "public/SingleTangentPlaneSequence.h"
#include "public/StandardAtmosphere.h"
#include "public/WindZero.h"
#include "public/Wgs84PrecalcWaypoint.h"
#include "public/EuclideanWaypointMonitor.h"
#include "public/InvalidIndexException.h"
#include "utility/CustomUnits.h"
#include "utils/public/OldCustomMathUtils.h"
#include "utils/public/PublicUtils.h"
#include <gtest/gtest.h>

using namespace std;
using namespace aaesim::test::utils;
using namespace aaesim::open_source;

namespace aaesim {
namespace test {
namespace open_source {

class TestHorizontalPathTracker : public HorizontalPathTracker {
   // A mock implementation that allows us to get at protected methods
  public:
   TestHorizontalPathTracker(const std::vector<HorizontalPath> &horizontal_trajectory,
                             TrajectoryIndexProgressionDirection expected_index_progression)
      : HorizontalPathTracker(horizontal_trajectory, expected_index_progression) {}
   bool TestIsPositionOnNode(Units::Length position_x, Units::Length position_y) {
      std::vector<HorizontalPath>::size_type node_index;
      bool is_on_node = IsPositionOnNode(position_x, position_y, node_index);
      if (is_on_node) UpdateCurrentIndex(node_index);
      return is_on_node;
   }

   bool TestIsDistanceAlongPathOnNode(Units::Length distance_along_path) {
      std::vector<HorizontalPath>::size_type node_index;
      bool is_on_node = IsDistanceAlongPathOnNode(distance_along_path, node_index);
      if (is_on_node) UpdateCurrentIndex(node_index);
      return is_on_node;
   }
};

TEST(CoreUtils, interpolate_trivial) {
   const int upper_index = 1;
   const double value = 0.5;
   std::vector<double> x_vals{0.0, 1.0}, y_vals{0.0, 1.0};
   const double y_expected = 0.5;

   // Test
   double y_actual = CoreUtils::LinearlyInterpolate(upper_index, value, x_vals, y_vals);
   ASSERT_EQ(y_expected, y_actual);
}

TEST(CoreUtils, interpolate_domain_error) {
   const int upper_index = 1;
   const double value = -0.5;
   std::vector<double> x_vals{0.0, 1.0}, y_vals{0.0, 1.0};

   // Test
   try {
      // Expect throw
      CoreUtils::LinearlyInterpolate(upper_index, value, x_vals,
                                     y_vals);  // value is not on the x_vals vector. Should throw.
      FAIL();                                  // if here, then throw didn't occur. Fail test.
   } catch (std::domain_error &e) {
      // Test passes.
   }
}

TEST(CoreUtils, interpolate_outofrange_error) {
   const int upper_index = -1;
   const double value = 0.5;
   std::vector<double> x_vals{0.0, 1.0}, y_vals{0.0, 1.0};

   // Test
   try {
      // Expect throw
      CoreUtils::LinearlyInterpolate(upper_index, value, x_vals,
                                     y_vals);  // value is not on the x_vals vector. Should throw.
      FAIL();                                  // if here, then throw didn't occur. Fail test.
   } catch (std::out_of_range &e) {
      // Test passes.
   }
}

TEST(CoreUtils, SignOfValue) {
   const double value = 10.0;
   ASSERT_EQ(1.0, CoreUtils::SignOfValue(value));
   ASSERT_EQ(-1.0, CoreUtils::SignOfValue(value * -1.0));
   ASSERT_EQ(0, CoreUtils::SignOfValue(0.0));
}

TEST(CoreUtils, LimitOnInterval) {
   const double value = 10.0, upper_limit = 15, lower_limit = 5;
   ASSERT_EQ(value, CoreUtils::LimitOnInterval(value, lower_limit, upper_limit));
   ASSERT_EQ(upper_limit, CoreUtils::LimitOnInterval(value * 2, lower_limit, upper_limit));   // too big
   ASSERT_EQ(lower_limit, CoreUtils::LimitOnInterval(value * -1, lower_limit, upper_limit));  // too small
   ASSERT_EQ(upper_limit,
             CoreUtils::LimitOnInterval(upper_limit, lower_limit, upper_limit));  // on interval lower limit
   ASSERT_EQ(lower_limit,
             CoreUtils::LimitOnInterval(lower_limit, lower_limit, upper_limit));  // on interval upper limit
}

TEST(CoreUtils, IM_Index) {
   // Set up a simple test vector
   std::vector<double> input_vector = {0.0, 1.0, 2.0, 3.0, 4.0};

   // Value-to-find greater than max value in vector
   double test_value = 4.3;

   int returned_index = CoreUtils::FindNearestIndex(test_value, input_vector);

   EXPECT_DOUBLE_EQ(returned_index, 4);

   // Value-to-find less than min value in vector
   test_value = -0.5;

   returned_index = CoreUtils::FindNearestIndex(test_value, input_vector);

   EXPECT_DOUBLE_EQ(returned_index, 0);

   // Value-to-find less than value at given start index
   test_value = 1.5;

   returned_index = CoreUtils::FindNearestIndex(test_value, input_vector);

   EXPECT_DOUBLE_EQ(returned_index, 2);

   // Value-to-find greater than value at given start index
   test_value = 3.5;

   returned_index = CoreUtils::FindNearestIndex(test_value, input_vector);

   EXPECT_DOUBLE_EQ(returned_index, 4);
}

TEST(CoreUtils, calculateEuclideanDistance) {
   // Calculate from a 3-4-5 triangle
   const Units::FeetLength expected = Units::FeetLength(5.0);
   const Units::FeetLength l4 = Units::FeetLength(4.0);
   const Units::FeetLength l3 = Units::FeetLength(3.0);
   const std::pair<Units::Length, Units::Length> pair1(l3, Units::ZERO_LENGTH);
   const std::pair<Units::Length, Units::Length> pair2(Units::ZERO_LENGTH, l4);
   const Units::FeetLength actual = CoreUtils::CalculateEuclideanDistance(pair1, pair2);
   EXPECT_EQ(expected, actual);
}

TEST(Atmosphere, temperature) {
   const Units::Temperature offset_temperature = Units::CelsiusTemperature(5);
   Atmosphere *atm = new StandardAtmosphere(offset_temperature);
   // check for discontinuity at tropopause
   Units::MetersLength half_eps(1e-6);
   Units::KelvinTemperature delta(1e-3);
   Units::KelvinTemperature temp_pre_tropo = atm->GetTemperature(atm->GetTropopauseHeight() - half_eps);
   Units::KelvinTemperature temp_post_tropo = atm->GetTemperature(atm->GetTropopauseHeight() + half_eps);
   EXPECT_NEAR(temp_pre_tropo.value(), temp_post_tropo.value(), delta.value());

   // check for temperature below absolute zero
   Units::KelvinTemperature temp_very_high_alt = atm->GetTemperature(Units::MetersLength(1e8));
   EXPECT_GE(temp_very_high_alt.value(), 0);
   EXPECT_EQ(offset_temperature, static_cast<StandardAtmosphere *>(atm)->GetTemperatureOffset());
   delete atm;
}

TEST(Atmosphere, density_low_altitude) {
   Atmosphere *atm = new StandardAtmosphere(Units::CelsiusTemperature(0));

   // below H_TROP
   const Units::MetersLength alt(3000.0);
   const Units::KilogramsMeterDensity expectedRho(9.0926e-1);  // from tabular standard atmosphere
   const Units::PascalsPressure expectedPressure(7.0121e4);    // from tabular standard atmosphere
   Units::KilogramsMeterDensity rho;
   Units::PascalsPressure pressure;
   atm->AirDensity(alt, rho, pressure);
   EXPECT_NEAR(rho.value(), expectedRho.value(), 1e-3);
   EXPECT_NEAR(pressure.value(), expectedPressure.value(), 1e2);  // tolerance can be large; Pascals are big numbers!

   delete atm;
}

TEST(Atmosphere, density_above_htrop) {
   Atmosphere *atm = new StandardAtmosphere(Units::CelsiusTemperature(0));

   // above H_TROP
   const Units::MetersLength alt(3000.0);
   const Units::MetersLength higher_alt = alt + atm->GetTropopauseHeight();
   const Units::KilogramsMeterDensity expectedRho(2.268e-1);  // from tabular standard atmosphere
   const Units::PascalsPressure expectedPressure(1.4101e4);   // from tabular standard atmosphere
   Units::KilogramsMeterDensity rho;
   Units::PascalsPressure pressure;
   atm->AirDensity(higher_alt, rho, pressure);
   EXPECT_NEAR(rho.value(), expectedRho.value(), 1e-3);
   EXPECT_NEAR(pressure.value(), expectedPressure.value(), 1e2);  // tolerance can be large; Pascals are big numbers!

   delete atm;
}
TEST(Atmosphere, speed) {
   // warm day, 4.5 F (2.5 C) above nominal
   Atmosphere *atm = new StandardAtmosphere(Units::CelsiusTemperature(2.5));

   const Units::Length alt = Units::MetersLength(3000.0);
   Units::Speed cas = Units::MetersPerSecondSpeed(400.0);
   Units::Speed tas = atm->CAS2TAS(cas, alt);
   EXPECT_NEAR(Units::MetersPerSecondSpeed(tas).value(), 443.67, .02);
   Units::Speed cas2 = atm->TAS2CAS(tas, alt);
   EXPECT_DOUBLE_EQ(Units::MetersPerSecondSpeed(cas).value(), Units::MetersPerSecondSpeed(cas2).value());
   delete atm;
}

TEST(AircraftCalculations, anglebetweenvectors) {
   // zero angle
   const Units::SignedRadiansAngle expectedAngle0(0.0), tol(1e-5);
   Units::SignedRadiansAngle actual = AircraftCalculations::ComputeAngleBetweenVectors(
         Units::ZERO_LENGTH, Units::ZERO_LENGTH, Units::MetersLength(1.0), Units::MetersLength(1.0),
         Units::MetersLength(1.0), Units::MetersLength(1.0));
   EXPECT_NEAR(expectedAngle0.value(), actual.value(), tol.value());

   // positive 45
   const Units::SignedRadiansAngle expectedAngle1 = Units::SignedRadiansAngle(M_PI / 4);
   actual = AircraftCalculations::ComputeAngleBetweenVectors(
         Units::ZERO_LENGTH, Units::ZERO_LENGTH, Units::MetersLength(1), Units::MetersLength(0),
         Units::MetersLength(sqrt(2)), Units::MetersLength(sqrt(2)));
   EXPECT_NEAR(expectedAngle1.value(), actual.value(), tol.value());

   // negative 45
   const Units::SignedRadiansAngle expectedAngle2 = Units::SignedRadiansAngle(M_PI / 4);
   actual = AircraftCalculations::ComputeAngleBetweenVectors(
         Units::ZERO_LENGTH, Units::ZERO_LENGTH, Units::MetersLength(1), Units::MetersLength(0),
         Units::MetersLength(sqrt(2)), Units::MetersLength(-sqrt(2)));
   EXPECT_NEAR(expectedAngle2.value(), actual.value(), tol.value());
}

TEST(AircraftCalculations, ComputeCrossProduct_trivial) {
   const Units::MetersLength x_vertex(0.0), y_vertex(0.0);
   const Units::MetersLength vector1_x(1.0), vector1_y(0.0);
   const Units::MetersLength vector2_x(0.0), vector2_y(1.0);
   const Units::MetersArea expected(1.0);
   Units::Area actual =
         AircraftCalculations::ComputeCrossProduct(x_vertex, y_vertex, vector1_x, vector1_y, vector2_x, vector2_y);
   ASSERT_EQ(expected, actual);
}

TEST(SimulationTime, basicTests) {

   // Tests various SimulationTime functions.

   // NOTE:This not a complete test of SimulationTime.  These
   // tests setup basically to test the make function.

   SimulationTime simtime;

   // Basic test

   SimulationTime::SetSimulationTimeStep(Units::SecondsTime(1.0));

   simtime.SetCycle(4);

   EXPECT_DOUBLE_EQ(SimulationTime::GetSimulationTimeStep().value(), 1.0);
   EXPECT_EQ(simtime.GetCycle(), 4);
   EXPECT_DOUBLE_EQ(simtime.GetCurrentSimulationTime().value(), 4.0);

   // Increment test.

   simtime.Increment();

   EXPECT_DOUBLE_EQ(simtime.GetCurrentSimulationTime().value(), 5.0);

   // Half time test.

   SimulationTime::SetSimulationTimeStep(Units::SecondsTime(0.5));
   simtime.SetCycle(21);

   EXPECT_DOUBLE_EQ(SimulationTime::GetSimulationTimeStep().value(), 0.5);
   EXPECT_EQ(simtime.GetCycle(), 21);
   EXPECT_DOUBLE_EQ(simtime.GetCurrentSimulationTime().value(), 10.5);

   // Make test 1

   Units::SecondsTime maketime(43.5);

   SimulationTime simmake = SimulationTime::Of(maketime);

   EXPECT_DOUBLE_EQ(SimulationTime::GetSimulationTimeStep().value(), 0.5);
   EXPECT_EQ(simmake.GetCycle(), 87);
   EXPECT_DOUBLE_EQ(simmake.GetCurrentSimulationTime().value(), 43.5);

   // Make test 2-0.2 step.

   SimulationTime::SetSimulationTimeStep(Units::SecondsTime(0.2));

   maketime = Units::SecondsTime(24.6);

   simmake = SimulationTime::Of(maketime);

   EXPECT_DOUBLE_EQ(SimulationTime::GetSimulationTimeStep().value(), 0.2);
   EXPECT_EQ(simmake.GetCycle(), 123);
   EXPECT_DOUBLE_EQ(simmake.GetCurrentSimulationTime().value(), 24.6);
   SimulationTime::SetSimulationTimeStep(Units::SecondsTime(1.0));  // be kind & reset for future tests
}

TEST(HorizontalPathTracker, consistency_check_straight_line_reverse) {
   /*
    * Test the behavior of HorizontalPathTracker. The point
    * here is to make sure the internal behavior is self-consistent.
    */
   const Units::MetersLength tolXY(100.0), tol_distance(1e-9);
   const Units::DegreesAngle tolCrs(0.5);
   for (int quad = aaesim::test::utils::Quadrant::FIRST; quad <= aaesim::test::utils::Quadrant::FOURTH; ++quad) {

      const std::vector<HorizontalPath> horizontal_trajectory =
            aaesim::test::utils::PublicUtils::CreateStraightHorizontalPath(
                  static_cast<aaesim::test::utils::Quadrant>(quad));
      PositionCalculator position_calculator(horizontal_trajectory, TrajectoryIndexProgressionDirection::DECREMENTING);
      AlongPathDistanceCalculator distance_calculator(horizontal_trajectory,
                                                      TrajectoryIndexProgressionDirection::DECREMENTING);

      std::vector<HorizontalPath>::const_reverse_iterator reverse_iterator = horizontal_trajectory.rbegin();
      for (; reverse_iterator != horizontal_trajectory.rend(); ++reverse_iterator) {
         Units::MetersLength x1(reverse_iterator->GetXPositionMeters()), y1(reverse_iterator->GetYPositionMeters()),
               dist;
         Units::UnsignedAngle crs1;
         distance_calculator.CalculateAlongPathDistanceFromPosition(x1, y1, dist, crs1);

         // Expect the distance to match the same distance in the horizontal path
         // description
         EXPECT_NEAR(reverse_iterator->m_path_length_cumulative_meters, Units::MetersLength(dist).value(),
                     tol_distance.value());

         Units::MetersLength x2, y2;
         Units::UnsignedAngle crs2;
         position_calculator.CalculatePositionFromAlongPathDistance(dist, x2, y2, crs2);

         EXPECT_NEAR(x1.value(), x2.value(), tolXY.value());
         EXPECT_NEAR(y1.value(), y2.value(), tolXY.value());
         EXPECT_NEAR(Units::SignedDegreesAngle(normalize(crs1)).value(),
                     Units::SignedDegreesAngle(normalize(crs2)).value(), tolCrs.value());
         EXPECT_FALSE(position_calculator.IsPassedEndOfRoute());
         EXPECT_FALSE(distance_calculator.IsPassedEndOfRoute());
      }

      // test off end of route
      Units::MetersLength x1(0), y1(0);
      switch (quad) {
         case aaesim::test::utils::Quadrant::FIRST:
            x1 = Units::MetersLength(-10);
            y1 = Units::MetersLength(-10);
            break;
         case aaesim::test::utils::Quadrant::SECOND:
            x1 = Units::MetersLength(10);
            y1 = Units::MetersLength(-10);
            break;
         case aaesim::test::utils::Quadrant::THIRD:
            x1 = Units::MetersLength(10);
            y1 = Units::MetersLength(10);
            break;
         case aaesim::test::utils::Quadrant::FOURTH:
            x1 = Units::MetersLength(-10);
            y1 = Units::MetersLength(10);
            break;
         default:
            break;
      }
      Units::MetersLength dist;
      Units::UnsignedAngle crs1;
      distance_calculator.CalculateAlongPathDistanceFromPosition(x1, y1, dist, crs1);

      Units::MetersLength x2, y2;
      Units::UnsignedAngle crs2;
      position_calculator.CalculatePositionFromAlongPathDistance(dist, x2, y2, crs2);

      EXPECT_NEAR(x1.value(), x2.value(), tolXY.value());
      EXPECT_NEAR(y1.value(), y2.value(), tolXY.value());
      EXPECT_NEAR(Units::SignedDegreesAngle(normalize(crs1)).value(),
                  Units::SignedDegreesAngle(normalize(crs2)).value(), tolCrs.value());
      EXPECT_TRUE(position_calculator.IsPassedEndOfRoute());
      EXPECT_TRUE(distance_calculator.IsPassedEndOfRoute());
   }
}

TEST(HorizontalPathTracker, consistency_check_straight_line_forward) {
   /*
    * Test the behavior of HorizontalPathTracker. The point
    * here is to make sure the internal behavior is self-consistent.
    */

   const std::vector<HorizontalPath> horizontal_trajectory =
         aaesim::test::utils::PublicUtils::CreateStraightHorizontalPath(aaesim::test::utils::Quadrant::FIRST);
   PositionCalculator position_calculator(horizontal_trajectory, TrajectoryIndexProgressionDirection::INCREMENTING);
   AlongPathDistanceCalculator distance_calculator(horizontal_trajectory,
                                                   TrajectoryIndexProgressionDirection::INCREMENTING);

   const Units::MetersLength tolXY(100.0), tol_distance(1e-9);
   const Units::DegreesAngle tolCrs(0.5);

   // test off end of route
   Units::MetersLength x1(-10), y1(-10), dist;
   Units::UnsignedAngle crs1;
   distance_calculator.CalculateAlongPathDistanceFromPosition(x1, y1, dist, crs1);

   Units::MetersLength x2, y2;
   Units::UnsignedAngle crs2;
   position_calculator.CalculatePositionFromAlongPathDistance(dist, x2, y2, crs2);

   EXPECT_NEAR(x1.value(), x2.value(), tolXY.value());
   EXPECT_NEAR(y1.value(), y2.value(), tolXY.value());
   EXPECT_NEAR(Units::SignedDegreesAngle(normalize(crs1)).value(), Units::SignedDegreesAngle(normalize(crs2)).value(),
               tolCrs.value());
   EXPECT_TRUE(position_calculator.IsPassedEndOfRoute());
   EXPECT_TRUE(distance_calculator.IsPassedEndOfRoute());

   std::vector<HorizontalPath>::const_iterator iterator = horizontal_trajectory.begin();
   for (; iterator != horizontal_trajectory.end(); ++iterator) {
      Units::MetersLength x1(iterator->GetXPositionMeters()), y1(iterator->GetYPositionMeters()), dist;
      Units::UnsignedAngle crs1;
      distance_calculator.CalculateAlongPathDistanceFromPosition(x1, y1, dist, crs1);

      // Expect the distance to match the same distance in the horizontal path
      // description
      EXPECT_NEAR(iterator->m_path_length_cumulative_meters, Units::MetersLength(dist).value(), tol_distance.value());

      Units::MetersLength x2, y2;
      Units::UnsignedAngle crs2;
      if (dist.value() > iterator->m_path_length_cumulative_meters) {
         // The assert above has ensured general accuracy of the calculated
         // distance. However, the calculated distance can be slightly larger than
         // the cummulative distance. This is allowable in the typical operations
         // of these calls. However in the case of this test, we need to drop a
         // little accuracy to correctly call
         // CalculatePositionFromAlongPathDistance as the next test.
         dist = Units::MetersLength(static_cast<float>(Units::MetersLength(dist).value()));
      }
      position_calculator.CalculatePositionFromAlongPathDistance(dist, x2, y2, crs2);

      EXPECT_NEAR(x1.value(), x2.value(), tolXY.value());
      EXPECT_NEAR(y1.value(), y2.value(), tolXY.value());
      EXPECT_NEAR(Units::SignedDegreesAngle(normalize(crs1)).value(),
                  Units::SignedDegreesAngle(normalize(crs2)).value(), tolCrs.value());
      EXPECT_FALSE(position_calculator.IsPassedEndOfRoute());
      EXPECT_FALSE(distance_calculator.IsPassedEndOfRoute());
   }
}

TEST(HorizontalPathTracker, consistency_check_straight_line_nodirection) {
   /*
    * Test the behavior of HorizontalPathTracker. The point
    * here is to make sure the internal behavior is self-consistent.
    */

   const std::vector<HorizontalPath> horizontal_trajectory =
         aaesim::test::utils::PublicUtils::CreateStraightHorizontalPath(aaesim::test::utils::Quadrant::FIRST);
   PositionCalculator position_calculator(horizontal_trajectory, TrajectoryIndexProgressionDirection::UNDEFINED);
   AlongPathDistanceCalculator distance_calculator(horizontal_trajectory,
                                                   TrajectoryIndexProgressionDirection::UNDEFINED);

   const Units::MetersLength tolXY(100.0), tol_distance(1e-9);
   const Units::DegreesAngle tolCrs(0.5);

   // test off end of route
   Units::MetersLength x1(-10), y1(-10), dist;
   Units::UnsignedAngle crs1;
   distance_calculator.CalculateAlongPathDistanceFromPosition(x1, y1, dist, crs1);

   Units::MetersLength x2, y2;
   Units::UnsignedAngle crs2;
   position_calculator.CalculatePositionFromAlongPathDistance(dist, x2, y2, crs2);

   EXPECT_NEAR(x1.value(), x2.value(), tolXY.value());
   EXPECT_NEAR(y1.value(), y2.value(), tolXY.value());
   EXPECT_NEAR(Units::SignedDegreesAngle(normalize(crs1)).value(), Units::SignedDegreesAngle(normalize(crs2)).value(),
               tolCrs.value());
   EXPECT_TRUE(position_calculator.IsPassedEndOfRoute());
   EXPECT_TRUE(distance_calculator.IsPassedEndOfRoute());

   // Test at beginning of route
   const HorizontalPath front = horizontal_trajectory.front();
   {
      x1 = Units::MetersLength(front.GetXPositionMeters());
      y1 = Units::MetersLength(front.GetYPositionMeters());
      distance_calculator.CalculateAlongPathDistanceFromPosition(x1, y1, dist, crs1);

      // Expect the distance to match the same distance in the horizontal path
      // description
      EXPECT_NEAR(front.m_path_length_cumulative_meters, Units::MetersLength(dist).value(), tol_distance.value());

      position_calculator.CalculatePositionFromAlongPathDistance(dist, x2, y2, crs2);

      EXPECT_NEAR(x1.value(), x2.value(), tolXY.value());
      EXPECT_NEAR(y1.value(), y2.value(), tolXY.value());
      EXPECT_NEAR(Units::SignedDegreesAngle(normalize(crs1)).value(),
                  Units::SignedDegreesAngle(normalize(crs2)).value(), tolCrs.value());
      EXPECT_FALSE(position_calculator.IsPassedEndOfRoute());
      EXPECT_FALSE(distance_calculator.IsPassedEndOfRoute());
   }

   // Jump to the end of the route and test
   const HorizontalPath back = horizontal_trajectory.back();
   {
      x1 = Units::MetersLength(back.GetXPositionMeters());
      y1 = Units::MetersLength(back.GetYPositionMeters());
      distance_calculator.CalculateAlongPathDistanceFromPosition(x1, y1, dist, crs1);

      // Expect the distance to match the same distance in the horizontal path
      // description
      EXPECT_NEAR(back.m_path_length_cumulative_meters, Units::MetersLength(dist).value(), tol_distance.value());

      position_calculator.CalculatePositionFromAlongPathDistance(dist, x2, y2, crs2);

      EXPECT_NEAR(x1.value(), x2.value(), tolXY.value());
      EXPECT_NEAR(y1.value(), y2.value(), tolXY.value());
      EXPECT_NEAR(Units::SignedDegreesAngle(normalize(crs1)).value(),
                  Units::SignedDegreesAngle(normalize(crs2)).value(), tolCrs.value());
      EXPECT_FALSE(position_calculator.IsPassedEndOfRoute());
      EXPECT_FALSE(distance_calculator.IsPassedEndOfRoute());
   }
}

TEST(AlongPathDistanceCalculator, check_for_throw_when_invalid_call_made_incrementing) {
   const std::vector<HorizontalPath> horizontal_trajectory =
         aaesim::test::utils::PublicUtils::CreateStraightHorizontalPath(aaesim::test::utils::Quadrant::FIRST);
   AlongPathDistanceCalculator distance_calculator(horizontal_trajectory,
                                                   TrajectoryIndexProgressionDirection::INCREMENTING);

   // Jump to the end of the route and test. Expect a throw
   Units::Length dist;
   const HorizontalPath front_node = horizontal_trajectory.front();
   const HorizontalPath test_node_should_throw = horizontal_trajectory.back();
   try {
      distance_calculator.CalculateAlongPathDistanceFromPosition(Units::MetersLength(front_node.GetXPositionMeters()),
                                                                 Units::MetersLength(front_node.GetYPositionMeters()),
                                                                 dist);  // should pass
      distance_calculator.CalculateAlongPathDistanceFromPosition(
            Units::MetersLength(test_node_should_throw.GetXPositionMeters()),
            Units::MetersLength(test_node_should_throw.GetYPositionMeters()), dist);  // should throw
      FAIL();                                                                         // this should NOT be hit
   } catch (exception &e) {
      // if here, the test has passed
   }
}

TEST(CustomMath, atan3_values) {
   EXPECT_DOUBLE_EQ(atan3(5, 5), M_PI * .25);
   EXPECT_DOUBLE_EQ(atan3(5, -5), M_PI * .75);
   EXPECT_DOUBLE_EQ(atan3(-5, -5), M_PI * 1.25);
   EXPECT_DOUBLE_EQ(atan3(-5, 5), M_PI * 1.75);
}

TEST(CustomMath, quantize) {
   Units::Speed s1 = Units::FeetPerSecondSpeed(12);
   Units::Speed sq = Units::FeetPerSecondSpeed(5);
   Units::Speed s2 = quantize(s1, sq);
   double result = Units::FeetPerSecondSpeed(s2).value();
   EXPECT_DOUBLE_EQ(result, 10);

   s1 = Units::FeetPerSecondSpeed(-3);
   s2 = quantize(s1, sq);
   result = Units::FeetPerSecondSpeed(s2).value();
   EXPECT_DOUBLE_EQ(result, -5);

   Units::Length d1 = Units::NauticalMilesLength(19);
   Units::Length dq = Units::NauticalMilesLength(4);
   Units::Length d2 = quantize(d1, dq);
   result = Units::NauticalMilesLength(d2).value();
   EXPECT_DOUBLE_EQ(result, 20);
}

TEST(RandomGenerator, uniformSample) {

   double seed = 15;
   ScenarioUtils::RANDOM_NUMBER_GENERATOR.SetSeed(seed);

   double s1 = 0, s2 = 0, s3 = 0, s4 = 0;
   int n = 100000;
   for (int i = 0; i < n; i++) {
      double x = ScenarioUtils::RANDOM_NUMBER_GENERATOR.UniformSample();
      EXPECT_GE(x, 0);
      EXPECT_LE(x, 1);
      double x2 = x * x;
      s1 += x;
      s2 += x2;
      s3 += x * x2;
      s4 += x2 * x2;
   }
   double ee = sqrt(1 / (double)n);
   double m1 = s1 / n;
   EXPECT_NEAR(.5, m1, ee);
   double m2 = s2 / n;
   EXPECT_NEAR(1.0 / 3.0, m2, ee);
   double m3 = s3 / n;
   EXPECT_NEAR(0.25, m3, ee);
   double m4 = s4 / n;
   EXPECT_NEAR(0.2, m4, ee);
}

TEST(RandomGenerator, uniformConsistencyTest) {
   OldCustomMath cm;

   double seed = 54321.0;
   ScenarioUtils::RANDOM_NUMBER_GENERATOR.SetSeed(seed);

   bool same = true;

   for (int ix = 0; ix < 1000; ix++) {
      double c = cm.uniform(seed);
      double r = ScenarioUtils::RANDOM_NUMBER_GENERATOR.UniformSample();

      same = same && (c == r);
   }

   EXPECT_TRUE(same);
   EXPECT_DOUBLE_EQ(seed, ScenarioUtils::RANDOM_NUMBER_GENERATOR.GetSeed());
}

TEST(RandomGenerator, rayleighConsistencyTest) {
   OldCustomMath cm;

   double seed = 54321.0;
   ScenarioUtils::RANDOM_NUMBER_GENERATOR.SetSeed(seed);

   bool same = true;

   for (int ix = 0; ix < 1000; ix++) {
      double c = cm.Rayleigh(0.0, 12.0, seed);
      double r = ScenarioUtils::RANDOM_NUMBER_GENERATOR.RayleighSample(0.0, 12.0);

      same = same && (c == r);
   }

   EXPECT_TRUE(same);
   EXPECT_DOUBLE_EQ(seed, ScenarioUtils::RANDOM_NUMBER_GENERATOR.GetSeed());
}

TEST(RandomGenerator, laplaceConsistencyTest) {
   OldCustomMath cm;

   double seed = 54321.0;
   ScenarioUtils::RANDOM_NUMBER_GENERATOR.SetSeed(seed);

   bool same = true;

   for (int ix = 0; ix < 1000; ix++) {
      double c = cm.laplace(69.0, seed);
      double r = ScenarioUtils::RANDOM_NUMBER_GENERATOR.LaplaceSample(69.0);

      same = same && (c == r);
   }

   EXPECT_TRUE(same);
   EXPECT_DOUBLE_EQ(seed, ScenarioUtils::RANDOM_NUMBER_GENERATOR.GetSeed());
}

TEST(RandomGenerator, gaussConsistencyTest) {
   OldCustomMath cm;

   double seed = 54321.0;
   ScenarioUtils::RANDOM_NUMBER_GENERATOR.SetSeed(seed);

   bool same = true;

   for (int ix = 0; ix < 1000; ix++) {
      double c = cm.gauss(0.0, 1.0, seed);
      double r = ScenarioUtils::RANDOM_NUMBER_GENERATOR.GaussianSample(0.0, 1.0);

      same = same && (c == r);
   }

   EXPECT_TRUE(same);
   EXPECT_DOUBLE_EQ(seed, ScenarioUtils::RANDOM_NUMBER_GENERATOR.GetSeed());
}

TEST(RandomGenerator, truncateGaussConsistencyTest) {
   OldCustomMath cm;

   double seed = 54321.0;
   ScenarioUtils::RANDOM_NUMBER_GENERATOR.SetSeed(seed);

   bool same = true;

   for (int ix = 0; ix < 1000; ix++) {
      double c = cm.trunc_gauss(0.0, (15.0 / 1.96), 3.0, seed);
      double r = ScenarioUtils::RANDOM_NUMBER_GENERATOR.TruncatedGaussianSample(0.0, (15.0 / 1.96), 3.0);
      same = same && (c == r);
   }

   EXPECT_TRUE(same);
   EXPECT_DOUBLE_EQ(seed, ScenarioUtils::RANDOM_NUMBER_GENERATOR.GetSeed());
}

TEST(DVector, access) {
   DVector v(1000, 1003);
   for (int i = v.GetMin(); i <= v.GetMax(); i++) {
      v[i] = i;
   }
   EXPECT_DOUBLE_EQ(v[1002], 1002);
   EXPECT_THROW(v[1005], InvalidIndexException);
   EXPECT_DOUBLE_EQ(v[1003], 1003);
}

TEST(DMatrix, multiply) {
   double a1[2][3] = {{1, 2, 3}, {4, 5, 6}};
   DMatrix a((double **)&a1, 0, 1, 0, 2);
   double b1[2][2] = {{7, 8}, {9, 10}};
   DMatrix b((double **)&b1, 0, 1, 0, 1);
   EXPECT_THROW(a * b, DMatrix::IncompatibleDimensionsException);

   DMatrix &ba = b * a;
   EXPECT_EQ(ba[0][0], 39);
   EXPECT_EQ(ba.GetMaxRow(), 1);
   EXPECT_EQ(ba.GetMaxColumn(), 2);
}

TEST(AircraftState, extrapolate) {
   AircraftState state_in;
   state_in.m_time = 0;
   state_in.m_xd = 100.0;
   state_in.m_yd = -100.0;
   state_in.SetZd(100.0);

   AircraftState state_out;
   double extrapolate_time = 10.0;

   state_out.Extrapolate(state_in, extrapolate_time);

   if (state_out.m_time == -1) {
      printf("Nothing was extrapolated!");
      FAIL();
   }

   EXPECT_DOUBLE_EQ(state_out.m_time, extrapolate_time);
   EXPECT_DOUBLE_EQ(state_out.m_x, 1000.0);
   EXPECT_DOUBLE_EQ(state_out.m_y, -1000.0);
   EXPECT_DOUBLE_EQ(state_out.m_z, 1000.0);
}

TEST(Units, CustomUnits) {
   Units::InvertedLength pm1 = Units::PerMeterInvertedLength(.25);
   Units::Length pm2 = Units::MetersLength(1);
   double pm = pm1 * pm2;
   EXPECT_DOUBLE_EQ(pm, .25);

   Units::InvertedSpeed pk1 = Units::SecondsPerNauticalMileInvertedSpeed(3600);
   Units::Speed pk2 = Units::KnotsSpeed(1);
   double pk = pk1 * pk2;
   EXPECT_DOUBLE_EQ(pk, 1);

   Units::InvertedLengthGain lg1 = Units::SecondsSquaredPerMeterInvertedLengthGain(5.0);
   Units::LengthGain lg2 = Units::MetersPerSecondSquaredLengthGain(0.7);
   double lg = lg1 * lg2;
   EXPECT_DOUBLE_EQ(lg, 3.5);
}

TEST(AlongPathDistanceCalculator, check_for_throw_when_invalid_call_made_decrementing) {
   const std::vector<HorizontalPath> horizontal_trajectory =
         aaesim::test::utils::PublicUtils::CreateStraightHorizontalPath(aaesim::test::utils::Quadrant::FIRST);
   AlongPathDistanceCalculator distance_calculator(horizontal_trajectory,
                                                   TrajectoryIndexProgressionDirection::DECREMENTING);

   // Jump to the end of the route and test. Expect a throw
   Units::Length dist;
   const HorizontalPath front_node = horizontal_trajectory.back();
   const HorizontalPath test_node_should_throw = horizontal_trajectory.front();
   try {
      distance_calculator.CalculateAlongPathDistanceFromPosition(Units::MetersLength(front_node.GetXPositionMeters()),
                                                                 Units::MetersLength(front_node.GetYPositionMeters()),
                                                                 dist);  // should pass
      distance_calculator.CalculateAlongPathDistanceFromPosition(
            Units::MetersLength(test_node_should_throw.GetXPositionMeters()),
            Units::MetersLength(test_node_should_throw.GetYPositionMeters()), dist);  // should throw
      FAIL();                                                                         // this should NOT be hit
   } catch (exception &e) {
      // if here, the test has passed
   }
}
TEST(PositionCalculator, check_for_throw_when_invalid_call_made) {
   const std::vector<HorizontalPath> horizontal_trajectory =
         aaesim::test::utils::PublicUtils::CreateStraightHorizontalPath(aaesim::test::utils::Quadrant::FIRST);
   PositionCalculator position_calculator(horizontal_trajectory, TrajectoryIndexProgressionDirection::INCREMENTING);

   // Jump to the end of the route and test. Expect a throw
   Units::Length x, y;
   Units::UnsignedAngle crs;
   const HorizontalPath test_node_should_throw = horizontal_trajectory.back();
   try {
      position_calculator.CalculatePositionFromAlongPathDistance(
            Units::MetersLength(test_node_should_throw.m_path_length_cumulative_meters), x, y, crs);  // should throw
      FAIL();  // this should NOT be hit
   } catch (exception &e) {
      // if here, the test has passed
   }
}

TEST(HorizontalPathCourseCalculator, consistency_check_straight_line_nodirection) {
   const Units::SignedDegreesAngle known_course(45.0);
   const Units::SignedDegreesAngle expected_reciprocal_course(known_course + Units::SignedDegreesAngle(180.0));
   const std::vector<HorizontalPath> horizontal_trajectory =
         aaesim::test::utils::PublicUtils::CreateStraightHorizontalPath(aaesim::test::utils::Quadrant::FIRST);
   DirectionOfFlightCourseCalculator course_calculator(horizontal_trajectory,
                                                       TrajectoryIndexProgressionDirection::UNDEFINED);

   const Units::DegreesAngle tol_crs(0.5);

   // test off end of route
   Units::UnsignedAngle actual_crs;
   course_calculator.CalculateCourseAtAlongPathDistance(Units::MetersLength(-50.0), actual_crs);
   EXPECT_NEAR(expected_reciprocal_course.value(), Units::SignedDegreesAngle(normalize(actual_crs)).value(),
               tol_crs.value());
   EXPECT_TRUE(course_calculator.IsPassedEndOfRoute());

   // Test at beginning of route
   const HorizontalPath front = horizontal_trajectory.front();
   {
      course_calculator.CalculateCourseAtAlongPathDistance(Units::MetersLength(front.m_path_length_cumulative_meters),
                                                           actual_crs);
      EXPECT_NEAR(expected_reciprocal_course.value(), Units::SignedDegreesAngle(normalize(actual_crs)).value(),
                  tol_crs.value());
      EXPECT_FALSE(course_calculator.IsPassedEndOfRoute());
   }

   // Jump to the end of the route and test
   const HorizontalPath back = horizontal_trajectory.back();
   {
      course_calculator.CalculateCourseAtAlongPathDistance(Units::MetersLength(back.m_path_length_cumulative_meters),
                                                           actual_crs);
      EXPECT_NEAR(expected_reciprocal_course.value(), Units::SignedDegreesAngle(normalize(actual_crs)).value(),
                  tol_crs.value());
      EXPECT_FALSE(course_calculator.IsPassedEndOfRoute());
   }
}

TEST(BackwardCourseCalculator, consistency_check_start_end_course) {
   const Units::DegreesAngle tol_crs(0.005);
   const Units::SignedDegreesAngle known_course(45.0);
   const Units::SignedDegreesAngle expected_reciprocal_course(known_course + Units::SignedDegreesAngle(180.0));
   const std::vector<HorizontalPath> horizontal_trajectory =
         aaesim::test::utils::PublicUtils::CreateStraightHorizontalPath(aaesim::test::utils::Quadrant::FIRST);
   DirectionOfFlightCourseCalculator course_calculator(horizontal_trajectory,
                                                       TrajectoryIndexProgressionDirection::UNDEFINED);

   Units::UnsignedAngle actual_start_course = course_calculator.GetCourseAtPathEnd();
   EXPECT_NEAR(expected_reciprocal_course.value(), Units::SignedDegreesAngle(normalize(actual_start_course)).value(),
               tol_crs.value());

   Units::UnsignedAngle actual_end_course = course_calculator.GetCourseAtPathStart();
   EXPECT_NEAR(expected_reciprocal_course.value(), Units::SignedDegreesAngle(normalize(actual_end_course)).value(),
               tol_crs.value());
}

TEST(BackwardCourseCalculator, consistency_check_incrementing) {
   const Units::DegreesAngle tol_crs(0.005);
   const Units::SignedDegreesAngle known_course(45.0);
   const Units::SignedDegreesAngle expected_reciprocal_course(known_course + Units::SignedDegreesAngle(180.0));
   const std::vector<HorizontalPath> horizontal_trajectory =
         aaesim::test::utils::PublicUtils::CreateStraightHorizontalPath(aaesim::test::utils::Quadrant::FIRST);
   DirectionOfFlightCourseCalculator course_calculator(horizontal_trajectory,
                                                       TrajectoryIndexProgressionDirection::INCREMENTING);

   Units::UnsignedAngle actual_back_course;
   bool actual_return_bool =
         course_calculator.CalculateCourseAtAlongPathDistance(Units::MetersLength(-10.), actual_back_course);
   EXPECT_TRUE(actual_return_bool);
   EXPECT_TRUE(course_calculator.IsPassedEndOfRoute());
   EXPECT_NEAR(expected_reciprocal_course.value(), Units::SignedDegreesAngle(normalize(actual_back_course)).value(),
               tol_crs.value());

   std::vector<HorizontalPath>::const_iterator iterator = horizontal_trajectory.begin();
   for (; iterator != horizontal_trajectory.end(); ++iterator) {
      Units::UnsignedAngle actual_back_course;
      bool actual_return_bool = course_calculator.CalculateCourseAtAlongPathDistance(
            Units::MetersLength(iterator->m_path_length_cumulative_meters), actual_back_course);
      EXPECT_TRUE(actual_return_bool);
      EXPECT_FALSE(course_calculator.IsPassedEndOfRoute());
      EXPECT_NEAR(expected_reciprocal_course.value(), Units::SignedDegreesAngle(normalize(actual_back_course)).value(),
                  tol_crs.value());
   }
}

TEST(BackwardCourseCalculator, course_throws_for_wrong_progression) {
   const Units::DegreesAngle tol_crs(0.005);
   const Units::SignedDegreesAngle known_course(45.0);
   const Units::SignedDegreesAngle expected_reciprocal_course(known_course + Units::SignedDegreesAngle(180.0));
   const std::vector<HorizontalPath> horizontal_trajectory =
         aaesim::test::utils::PublicUtils::CreateStraightHorizontalPath(aaesim::test::utils::Quadrant::FIRST);
   DirectionOfFlightCourseCalculator course_calculator(horizontal_trajectory,
                                                       TrajectoryIndexProgressionDirection::INCREMENTING);
   HorizontalPath start_of_path = horizontal_trajectory.back();
   try {
      Units::UnsignedAngle actual_back_course;
      course_calculator.CalculateCourseAtAlongPathDistance(
            Units::MetersLength(start_of_path.m_path_length_cumulative_meters), actual_back_course);
      FAIL();
   } catch (logic_error &e) {
      // This is correct. Test passed.
   }

   course_calculator =
         DirectionOfFlightCourseCalculator(horizontal_trajectory, TrajectoryIndexProgressionDirection::DECREMENTING);
   try {
      Units::UnsignedAngle actual_back_course;
      course_calculator.CalculateCourseAtAlongPathDistance(Units::zero(), actual_back_course);
      FAIL();
   } catch (logic_error &e) {
      // This is correct. Test passed.
   }
}

TEST(BackwardCourseCalculator, consistency_check_decrementing) {
   const Units::DegreesAngle tol_crs(0.005);
   const Units::SignedDegreesAngle known_course(45.0);
   const Units::SignedDegreesAngle expected_reciprocal_course(known_course + Units::SignedDegreesAngle(180.0));
   const std::vector<HorizontalPath> horizontal_trajectory =
         aaesim::test::utils::PublicUtils::CreateStraightHorizontalPath(aaesim::test::utils::Quadrant::FIRST);
   DirectionOfFlightCourseCalculator course_calculator(horizontal_trajectory,
                                                       TrajectoryIndexProgressionDirection::DECREMENTING);

   std::vector<HorizontalPath>::const_reverse_iterator iterator = horizontal_trajectory.crbegin();
   for (; iterator != horizontal_trajectory.crend(); ++iterator) {
      Units::UnsignedAngle actual_back_course;
      bool actual_return_bool = course_calculator.CalculateCourseAtAlongPathDistance(
            Units::MetersLength(iterator->m_path_length_cumulative_meters), actual_back_course);

      EXPECT_TRUE(actual_return_bool);
      EXPECT_FALSE(course_calculator.IsPassedEndOfRoute());
      EXPECT_NEAR(expected_reciprocal_course.value(), Units::SignedDegreesAngle(normalize(actual_back_course)).value(),
                  tol_crs.value());
   }

   Units::UnsignedAngle actual_back_course;
   bool actual_return_bool =
         course_calculator.CalculateCourseAtAlongPathDistance(Units::MetersLength(-10.0), actual_back_course);
   EXPECT_TRUE(actual_return_bool);
   EXPECT_TRUE(course_calculator.IsPassedEndOfRoute());
   EXPECT_NEAR(expected_reciprocal_course.value(), Units::SignedDegreesAngle(normalize(actual_back_course)).value(),
               tol_crs.value());
}

TEST(AlongPathDistanceCalculator, consistency_check_two_public_methods) {
   const Units::DegreesAngle tol_crs(0.5);
   const Units::SignedDegreesAngle known_course(45.0);
   const Units::SignedDegreesAngle expected_reciprocal_course(known_course + Units::SignedDegreesAngle(180.0));
   const std::vector<HorizontalPath> horizontal_trajectory =
         aaesim::test::utils::PublicUtils::CreateStraightHorizontalPath(aaesim::test::utils::Quadrant::FIRST);
   AlongPathDistanceCalculator distance_calculator(horizontal_trajectory,
                                                   TrajectoryIndexProgressionDirection::UNDEFINED);

   // test at end of route
   const HorizontalPath horizontal_path_node = horizontal_trajectory.back();
   Units::UnsignedAngle actual_crs;
   Units::MetersLength actual_distance_to_go_method_1, actual_distance_to_go_method_2;
   distance_calculator.CalculateAlongPathDistanceFromPosition(
         Units::MetersLength(horizontal_path_node.GetXPositionMeters()),
         Units::MetersLength(horizontal_path_node.GetYPositionMeters()), actual_distance_to_go_method_1, actual_crs);
   distance_calculator.CalculateAlongPathDistanceFromPosition(
         Units::MetersLength(horizontal_path_node.GetXPositionMeters()),
         Units::MetersLength(horizontal_path_node.GetYPositionMeters()), actual_distance_to_go_method_2);
   EXPECT_NEAR(horizontal_path_node.m_path_length_cumulative_meters, actual_distance_to_go_method_1.value(), 1e-13);
   EXPECT_NEAR(horizontal_path_node.m_path_length_cumulative_meters, actual_distance_to_go_method_2.value(), 1e-13);
   EXPECT_NEAR(expected_reciprocal_course.value(), Units::SignedDegreesAngle(normalize(actual_crs)).value(),
               tol_crs.value());
}

TEST(WindZero, test_for_zero_behavior) {
   Units::MetersPerSecondSpeed u, v;
   WindZero zero_wind;
   zero_wind.InterpolateWind(Units::zero(), Units::zero(), Units::zero(), u, v);
   EXPECT_EQ(0., u.value());
   EXPECT_EQ(0., v.value());

   Units::CelsiusTemperature temperature =
         zero_wind.InterpolateTemperature(Units::zero(), Units::zero(), Units::zero());
   EXPECT_NEAR(288.15, temperature.value(), 1e-1);
}

TEST(TestHorizontalPathTracker, check_is_on_node_position) {
   const std::vector<HorizontalPath> horizontal_trajectory =
         aaesim::test::utils::PublicUtils::CreateStraightHorizontalPath(aaesim::test::utils::Quadrant::FIRST);

   // Decrementing
   {
      TestHorizontalPathTracker tracker(horizontal_trajectory, TrajectoryIndexProgressionDirection::DECREMENTING);
      for (std::vector<HorizontalPath>::size_type index = horizontal_trajectory.size() - 1; index > 0; --index) {
         const HorizontalPath hp = horizontal_trajectory[index];
         bool actual_return_bool = tracker.TestIsPositionOnNode(Units::MetersLength(hp.GetXPositionMeters()),
                                                                Units::MetersLength(hp.GetYPositionMeters()));
         std::vector<HorizontalPath>::size_type actual_node_index = tracker.GetCurrentTrajectoryIndex();

         EXPECT_TRUE(actual_return_bool);
         EXPECT_EQ(index, actual_node_index);
      }
   }

   // Incrementing
   {
      TestHorizontalPathTracker tracker(horizontal_trajectory, TrajectoryIndexProgressionDirection::INCREMENTING);
      for (std::vector<HorizontalPath>::size_type index = 0; index < horizontal_trajectory.size(); ++index) {
         const HorizontalPath hp = horizontal_trajectory[index];
         bool actual_return_bool = tracker.TestIsPositionOnNode(Units::MetersLength(hp.GetXPositionMeters()),
                                                                Units::MetersLength(hp.GetYPositionMeters()));
         std::vector<HorizontalPath>::size_type actual_node_index = tracker.GetCurrentTrajectoryIndex();
         EXPECT_TRUE(actual_return_bool);
         EXPECT_EQ(index, actual_node_index);
      }
   }

   // Undefined
   {
      TestHorizontalPathTracker tracker(horizontal_trajectory, TrajectoryIndexProgressionDirection::UNDEFINED);
      for (std::vector<HorizontalPath>::size_type index = 0; index < horizontal_trajectory.size(); ++index) {
         const HorizontalPath hp = horizontal_trajectory[index];
         bool actual_return_bool = tracker.TestIsPositionOnNode(Units::MetersLength(hp.GetXPositionMeters()),
                                                                Units::MetersLength(hp.GetYPositionMeters()));
         std::vector<HorizontalPath>::size_type actual_node_index = tracker.GetCurrentTrajectoryIndex();
         EXPECT_TRUE(actual_return_bool);
         EXPECT_EQ(index, actual_node_index);
      }
   }
}

TEST(TestHorizontalPathTracker, check_is_on_node_distance) {
   const std::vector<HorizontalPath> horizontal_trajectory =
         aaesim::test::utils::PublicUtils::CreateStraightHorizontalPath(aaesim::test::utils::Quadrant::FIRST);

   // Decrementing
   {
      TestHorizontalPathTracker tracker(horizontal_trajectory, TrajectoryIndexProgressionDirection::DECREMENTING);
      for (std::vector<HorizontalPath>::size_type index = horizontal_trajectory.size() - 1; index > 0; --index) {
         const HorizontalPath hp = horizontal_trajectory[index];
         bool actual_return_bool =
               tracker.TestIsDistanceAlongPathOnNode(Units::MetersLength(hp.m_path_length_cumulative_meters));
         std::vector<HorizontalPath>::size_type actual_node_index = tracker.GetCurrentTrajectoryIndex();

         EXPECT_TRUE(actual_return_bool);
         EXPECT_EQ(index, actual_node_index);
      }
   }

   // Incrementing
   {
      TestHorizontalPathTracker tracker(horizontal_trajectory, TrajectoryIndexProgressionDirection::INCREMENTING);
      for (std::vector<HorizontalPath>::size_type index = 0; index < horizontal_trajectory.size(); ++index) {
         const HorizontalPath hp = horizontal_trajectory[index];
         bool actual_return_bool =
               tracker.TestIsDistanceAlongPathOnNode(Units::MetersLength(hp.m_path_length_cumulative_meters));
         std::vector<HorizontalPath>::size_type actual_node_index = tracker.GetCurrentTrajectoryIndex();

         EXPECT_TRUE(actual_return_bool);
         EXPECT_EQ(index, actual_node_index);
      }
   }

   // Undefined
   {
      TestHorizontalPathTracker tracker(horizontal_trajectory, TrajectoryIndexProgressionDirection::UNDEFINED);
      for (std::vector<HorizontalPath>::size_type index = 0; index < horizontal_trajectory.size(); ++index) {
         const HorizontalPath hp = horizontal_trajectory[index];
         bool actual_return_bool =
               tracker.TestIsDistanceAlongPathOnNode(Units::MetersLength(hp.m_path_length_cumulative_meters));
         std::vector<HorizontalPath>::size_type actual_node_index = tracker.GetCurrentTrajectoryIndex();

         EXPECT_TRUE(actual_return_bool);
         EXPECT_EQ(index, actual_node_index);
      }
   }
}

TEST(StandardAtmosphere, mach_ias_transition_isa) {
   // Test mach/ias conversion at altitude using mach of unity.
   const std::vector<double> test_machs = {.6, .65, .713, .868};
   const std::vector<Units::Length> test_altitudes = {Units::FeetLength(27800.0), Units::FeetLength(30000.0),
                                                      Units::FeetLength(35000.0)};
   const Units::FeetLength tolerance(50.0);

   for (double test_mach : test_machs) {
      for (Units::FeetLength test_altitude : test_altitudes) {
         // zero temp offset
         const StandardAtmosphere atmosphere_0(Units::CelsiusTemperature(0.));
         Units::Speed ias_at_test_mach = atmosphere_0.MachToIAS(test_mach, test_altitude);
         Units::FeetLength actual_alt_trans_sea_level_old =
               atmosphere_0.GetMachIASTransition(ias_at_test_mach, test_mach);
         Units::FeetLength actual_alt_trans_sea_level = atmosphere_0.GetMachIASTransition(ias_at_test_mach, test_mach);
         EXPECT_NEAR(test_altitude.value(), actual_alt_trans_sea_level_old.value(), tolerance.value());
         EXPECT_NEAR(test_altitude.value(), actual_alt_trans_sea_level.value(), tolerance.value());
      }
   }
}

TEST(AircraftState, GetHeadingCcwFromEastRadians) {

   for (int q = Quadrant::FIRST; q <= Quadrant::FOURTH; ++q) {

      vector<HorizontalPath> known_course_path = PublicUtils::CreateStraightHorizontalPath(static_cast<Quadrant>(q));
      AircraftState state1, state2, test_state;
      state1.m_x = known_course_path[0].GetXPositionMeters();
      state1.m_y = known_course_path[0].GetYPositionMeters();
      state1.m_z = 0;
      state1.m_time = 0;
      state2.m_x = known_course_path[1].GetXPositionMeters();
      state2.m_y = known_course_path[1].GetYPositionMeters();
      state2.m_z = 0;
      state2.m_time = 1;

      state1.m_xd = state2.m_x / (state2.m_time - state1.m_time);
      state1.m_yd = state2.m_y / (state2.m_time - state1.m_time);
      state2.m_xd = state1.m_xd;
      state2.m_yd = state1.m_yd;

      test_state.Interpolate(state1, state2, 0.5);
      const Units::SignedRadiansAngle reported_heading = test_state.GetHeadingCcwFromEastRadians();
      EXPECT_NEAR(known_course_path[0].m_path_course, reported_heading.value(), 1e-3);
   }
}

TEST(Guidance, GetIasCommandIntegerKnots) {
   Guidance guidance;

   // test rounding up
   guidance.m_ias_command = Units::KnotsSpeed(209.9);
   EXPECT_EQ(210, guidance.GetIasCommandIntegerKnots());

   // test rounding down
   guidance.m_ias_command = Units::KnotsSpeed(209.1);
   EXPECT_EQ(209, guidance.GetIasCommandIntegerKnots());
}

TEST(StandardAtmosphere, MakeInstanceFromTemperatureOffset) {
   auto zero_offset_sa = StandardAtmosphere::MakeInstanceFromTemperatureOffset(Units::CelsiusTemperature(0));

   ASSERT_EQ(0, Units::CelsiusTemperature(zero_offset_sa->GetTemperatureOffset()).value());
}

class PredictedWindEvaluatorTest : public ::testing::Test {
  protected:
   PredictedWindEvaluatorTest()
      : aircraft_state(),
        weather_prediction(Wind::CreateZeroWindPrediction()),
        reference_cas(Units::ZERO_SPEED),
        reference_altitude(Units::ZERO_LENGTH),
        sensed_atmosphere(StandardAtmosphere::MakeInstanceFromTemperatureOffset(Units::CelsiusTemperature(0))) {}

   aaesim::open_source::AircraftState aircraft_state;
   WeatherPrediction weather_prediction;
   Units::Speed reference_cas;
   Units::Length reference_altitude;
   std::shared_ptr<Atmosphere> sensed_atmosphere;
};

TEST_F(PredictedWindEvaluatorTest, VectorDifferenceWindEvaluator) {
   const std::shared_ptr<PredictedWindEvaluator> vector_difference_wind_evaluator =
         VectorDifferenceWindEvaluator::GetInstance(Units::KnotsSpeed(1));
   ASSERT_TRUE(vector_difference_wind_evaluator->ArePredictedWindsAccurate(
         aircraft_state, weather_prediction, reference_cas, reference_altitude, sensed_atmosphere.get()));

   aircraft_state.m_Vwx = 1.1;
   ASSERT_FALSE(vector_difference_wind_evaluator->ArePredictedWindsAccurate(
         aircraft_state, weather_prediction, reference_cas, reference_altitude, sensed_atmosphere.get()));
}

TEST(EuclideanWaypointMonitor, passed_waypoint_first_update) {
   const Waypoint test_waypoint("monitor_me", Units::DegreesAngle(38), Units::DegreesAngle(-77));
   aaesim::Wgs84PrecalcWaypoint point_to_monitor;
   point_to_monitor.m_position = aaesim::LatitudeLongitudePoint::CreateFromWaypoint(test_waypoint);
   std::shared_ptr<aaesim::open_source::EuclideanWaypointMonitor> test_monitor =
         aaesim::open_source::EuclideanWaypointMonitor::OfWgs84PrecalcWaypoint(point_to_monitor);

   aaesim::LatitudeLongitudePoint test_point = point_to_monitor.m_position.ProjectDistanceAlongCourse(
         Units::MetersLength(100), Units::SignedDegreesAngle(80));

   // This is the tested method
   test_monitor->Update(test_point, Units::SignedDegreesAngle(0));

   EXPECT_TRUE(test_monitor->IsPassedWaypoint());
}

TEST(EuclideanWaypointMonitor, passes_waypoint_second_call) {
   const Waypoint test_waypoint("monitor_me", Units::DegreesAngle(38), Units::DegreesAngle(-77));
   aaesim::Wgs84PrecalcWaypoint point_to_monitor;
   point_to_monitor.m_position = aaesim::LatitudeLongitudePoint::CreateFromWaypoint(test_waypoint);
   std::shared_ptr<aaesim::open_source::EuclideanWaypointMonitor> test_monitor =
         aaesim::open_source::EuclideanWaypointMonitor::OfWgs84PrecalcWaypoint(point_to_monitor);

   const aaesim::LatitudeLongitudePoint test_point1 = point_to_monitor.m_position.ProjectDistanceAlongCourse(
         Units::MetersLength(100), Units::SignedDegreesAngle(91));
   test_monitor->Update(test_point1, Units::SignedDegreesAngle(0));
   EXPECT_FALSE(test_monitor->IsPassedWaypoint());

   const aaesim::LatitudeLongitudePoint test_point2 = point_to_monitor.m_position.ProjectDistanceAlongCourse(
         Units::MetersLength(100), Units::SignedDegreesAngle(89));
   test_monitor->Update(test_point2, Units::SignedDegreesAngle(0));
   EXPECT_TRUE(test_monitor->IsPassedWaypoint());
}

TEST(FlightEnvelopeSpeedLimiter, limit_speed_command) {
   aaesim::open_source::bada_utils::FlightEnvelope flight_envelope;
   flight_envelope.V_mo = Units::KnotsSpeed(250);
   flight_envelope.M_mo = 1.0;
   flight_envelope.h_mo = Units::ZERO_LENGTH;
   flight_envelope.h_max = Units::ZERO_LENGTH;
   flight_envelope.G_w = Units::zero();
   flight_envelope.G_t = 0.0;

   aaesim::open_source::bada_utils::FlapSpeeds flap_speeds;
   flap_speeds.cas_approach_minimum = Units::KnotsSpeed(200);
   flap_speeds.cas_approach_maximum = Units::KnotsSpeed(210);
   flap_speeds.cas_landing_minimum = Units::KnotsSpeed(200);
   flap_speeds.cas_landing_maximum = Units::KnotsSpeed(210);
   flap_speeds.cas_gear_out_minimum = Units::KnotsSpeed(200);
   flap_speeds.cas_gear_out_maximum = Units::KnotsSpeed(210);
   flap_speeds.cas_takeoff_minimum = Units::KnotsSpeed(210);
   flap_speeds.cas_climb_minimum = Units::KnotsSpeed(200);
   flap_speeds.cas_cruise_minimum = Units::KnotsSpeed(210);

   FlightEnvelopeSpeedLimiter flight_envelope_speed_limiter(flap_speeds, flight_envelope);
   Units::Speed limited_speed = flight_envelope_speed_limiter.LimitSpeedCommand(
         Units::ZERO_SPEED, flap_speeds.cas_takeoff_minimum - Units::KnotsSpeed(1), Units::ZERO_SPEED,
         Units::ZERO_LENGTH, Units::ZERO_LENGTH, Units::ZERO_LENGTH,
         aaesim::open_source::bada_utils::FlapConfiguration::TAKEOFF);
   ASSERT_EQ(limited_speed, flap_speeds.cas_takeoff_minimum);

   limited_speed = flight_envelope_speed_limiter.LimitSpeedCommand(
         Units::ZERO_SPEED, flap_speeds.cas_cruise_minimum - Units::KnotsSpeed(1), Units::ZERO_SPEED,
         Units::ZERO_LENGTH, Units::ZERO_LENGTH, Units::ZERO_LENGTH, bada_utils::FlapConfiguration::CRUISE);
   ASSERT_EQ(limited_speed, flap_speeds.cas_cruise_minimum);

   limited_speed = flight_envelope_speed_limiter.LimitSpeedCommand(
         Units::ZERO_SPEED, flap_speeds.cas_approach_maximum + Units::KnotsSpeed(1), Units::ZERO_SPEED,
         Units::ZERO_LENGTH, Units::ZERO_LENGTH, Units::ZERO_LENGTH, bada_utils::FlapConfiguration::APPROACH);
   ASSERT_EQ(limited_speed, flap_speeds.cas_approach_maximum);

   limited_speed = flight_envelope_speed_limiter.LimitSpeedCommand(
         Units::ZERO_SPEED, FlightEnvelopeSpeedLimiter::MINIMUM_IAS_LIMIT - Units::KnotsSpeed(1), Units::ZERO_SPEED,
         Units::ZERO_LENGTH, Units::ZERO_LENGTH, Units::ZERO_LENGTH, bada_utils::FlapConfiguration::UNDEFINED);
   ASSERT_EQ(limited_speed, FlightEnvelopeSpeedLimiter::MINIMUM_IAS_LIMIT);
}

TEST(FlightEnvelopeSpeedLimiter, limit_mach_command) {
   aaesim::open_source::bada_utils::FlightEnvelope flight_envelope;
   flight_envelope.V_mo = Units::KnotsSpeed(10);
   flight_envelope.M_mo = 1.0;
   flight_envelope.h_mo = Units::ZERO_LENGTH;
   flight_envelope.h_max = Units::ZERO_LENGTH;
   flight_envelope.G_w = Units::zero();
   flight_envelope.G_t = 0.0;

   aaesim::open_source::bada_utils::FlapSpeeds flap_speeds;
   flap_speeds.cas_approach_minimum = Units::KnotsSpeed(1);
   flap_speeds.cas_approach_maximum = Units::KnotsSpeed(10);
   flap_speeds.cas_landing_minimum = Units::KnotsSpeed(1);
   flap_speeds.cas_landing_maximum = Units::KnotsSpeed(10);
   flap_speeds.cas_gear_out_minimum = Units::KnotsSpeed(1);
   flap_speeds.cas_gear_out_maximum = Units::KnotsSpeed(10);
   flap_speeds.cas_takeoff_minimum = Units::KnotsSpeed(10);
   flap_speeds.cas_climb_minimum = Units::KnotsSpeed(1);
   flap_speeds.cas_cruise_minimum = Units::KnotsSpeed(10);

   FlightEnvelopeSpeedLimiter flight_envelope_speed_limiter(flap_speeds, flight_envelope);
   BoundedValue<double, 0, 2> current_mach(flight_envelope.M_mo + BoundedValue<double, 0, 2>(0.1));

   BoundedValue<double, 0, 2> limited_mach = flight_envelope_speed_limiter.LimitMachCommand(
         BoundedValue<double, 0, 2>(0), current_mach, BoundedValue<double, 0, 2>(0), Units::ZERO_MASS,
         Units::ZERO_LENGTH, WeatherPrediction());
   ASSERT_EQ(limited_mach, flight_envelope.M_mo);

   current_mach = FlightEnvelopeSpeedLimiter::MINIMUM_MACH_LIMIT - BoundedValue<double, 0, 2>(0.1);
   limited_mach = flight_envelope_speed_limiter.LimitMachCommand(BoundedValue<double, 0, 2>(0), current_mach,
                                                                 BoundedValue<double, 0, 2>(0), Units::ZERO_MASS,
                                                                 Units::ZERO_LENGTH, WeatherPrediction());
   ASSERT_EQ(limited_mach, FlightEnvelopeSpeedLimiter::MINIMUM_MACH_LIMIT);

   current_mach = FlightEnvelopeSpeedLimiter::MINIMUM_MACH_LIMIT + BoundedValue<double, 0, 2>(0.1);
   limited_mach = flight_envelope_speed_limiter.LimitMachCommand(BoundedValue<double, 0, 2>(0), current_mach,
                                                                 BoundedValue<double, 0, 2>(0), Units::ZERO_MASS,
                                                                 Units::ZERO_LENGTH, WeatherPrediction());
   ASSERT_EQ(limited_mach, current_mach);
}

}  // namespace open_source
}  // namespace test
}  // namespace aaesim
