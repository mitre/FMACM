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

#include "gtest/gtest.h"
#include "imalgs/IMAlgorithmLoader.h"
#include "imalgs/IMTimeBasedAchieve.h"
#include "imalgs/IMUtils.h"

using namespace std;
using namespace aaesim::open_source;

namespace aaesim
{
   namespace test
   {
      namespace imalgo
      {

         class IMAchieveTest : public IMAchieve, public ::testing::Test
         {
            // A mock implementation that allows us to get at protected methods in IMAchieve
         protected:
            void SetAssignedSpacingGoal(const IMClearance &clearance) override
            {
               // Not needed for testing, only to build properly
            }

            void SetMaxSpeedDeviationPercentage(const double max_speed_deviation_factor) override
            {
               IMAlgorithm::SetMaxSpeedDeviationPercentage(max_speed_deviation_factor);
            }

         public:
            const double GetSpacingError() const override
            {
               // Not used for testing, only to build
               return 0;
            }
         };

         TEST(IMAlgorithmLoader, getAlgorithmType)
         {
            IMAlgorithmLoader imloader;
            IMUtils::IMAlgorithmTypes none = imloader.GetAlgorithmType("none");
            EXPECT_EQ(IMUtils::IMAlgorithmTypes::NONE, none);

            /* Tests removed: see AAES-1129
            IMUtils::IMAlgorithmTypes precalcAchieve = imloader.GetAlgorithmType("precalcAchieve");
            EXPECT_EQ(IMUtils::IMAlgorithmTypes::KINETICACHIEVE, precalcAchieve);

            IMUtils::IMAlgorithmTypes precalcTargetAchieve = imloader.GetAlgorithmType("precalcTargetAchieve");
            EXPECT_EQ(IMUtils::IMAlgorithmTypes::KINETICTARGETACHIEVE, precalcTargetAchieve);
             */

            IMUtils::IMAlgorithmTypes timeBasedAchieve = imloader.GetAlgorithmType("timeBasedAchieve");
            EXPECT_EQ(IMUtils::IMAlgorithmTypes::TIMEBASEDACHIEVE, timeBasedAchieve);

            IMUtils::IMAlgorithmTypes distBasedAchieve = imloader.GetAlgorithmType("distBasedAchieve");
            EXPECT_EQ(IMUtils::IMAlgorithmTypes::DISTANCEBASEDACHIEVE, distBasedAchieve);
         }

         TEST_F(IMAchieveTest, SetMaxSpeedDeviationPercentage)
         {
            SetMaxSpeedDeviationPercentage(25);
            const double nominal3(3);
            EXPECT_DOUBLE_EQ(HighLimit(nominal3), 3.75);
            EXPECT_DOUBLE_EQ(LowLimit(nominal3), 2.25);
         }

         TEST_F(IMAchieveTest, errorThreshold)
         {

            // Set the distance from ABP at which error threshold becomes constant
            m_error_distance = Units::NauticalMilesLength(0.0);

            // Set the constant term of the error threshold time
            m_time_threshold = Units::SecondsTime(0.0);

            // A negative slope would cause error threshold to increase as ownship approaches ABP
            Units::SecondsPerNauticalMileInvertedSpeed slope_value = m_slope;
            if (slope_value.value() < 0.0)
            {
               FAIL();
            }

            Units::SecondsTime test_time = GetErrorThreshold(Units::NauticalMilesLength(0.0));
            EXPECT_DOUBLE_EQ(test_time.value(), 0.0);

            test_time = GetErrorThreshold(Units::NauticalMilesLength(50.0));
            EXPECT_DOUBLE_EQ(test_time.value(), 12.5);

            test_time = GetErrorThreshold(Units::NauticalMilesLength(-10.0));
            Units::SecondsTime time_threshold = m_time_threshold;
            EXPECT_DOUBLE_EQ(test_time.value(), time_threshold.value());
         }

         TEST(IMUtils, projTargetPosition)
         {
            /*
             * Test the projection method using unit lengths. This geometry is
             * easily diagrammed on paper and then tested.
             */
            vector<HorizontalPath> ownship_horizontal_traj;
            HorizontalPath hp;
            hp.m_segment_type = HorizontalPath::SegmentType::STRAIGHT;
            hp.SetXYPositionMeters(0, 0);
            hp.m_path_length_cumulative_meters = 0;
            hp.m_path_course = 0;
            ownship_horizontal_traj.push_back(hp);
            hp.SetXYPositionMeters(1, 0);
            hp.m_path_length_cumulative_meters = 1;
            ownship_horizontal_traj.push_back(hp);

            // Define the geometry so that expected conditions are mathematically valid
            const Units::MetersLength target_dtg(0.5);
            const Units::Length target_x = Units::MetersLength(0.5);
            const Units::Length target_y = target_x;

            // Test expectations
            const Units::MetersLength tol(1e-10);
            const Units::MetersLength xExpect(0.5), yExpect(0.0), dtgExpect(0.5);

            // test
            Units::MetersLength xProjected, yProjected, dtgProjected;
            bool b1 = IMUtils::ProjectTargetPosition(target_x, target_y, ownship_horizontal_traj,
                                                     xProjected, yProjected, dtgProjected);
            EXPECT_TRUE(b1);
            EXPECT_NEAR(xExpect.value(), xProjected.value(), tol.value());
            EXPECT_NEAR(yExpect.value(), yProjected.value(), tol.value());
            EXPECT_NEAR(dtgExpect.value(), dtgProjected.value(), tol.value());

            /*
             * Check again with a very large cross track error (make target_y big). Note that cte
             * can't be larger than 2.5 NM according to AircraftCalculations algorithms.
             */
            const Units::Length target_y_large = target_x + Units::NauticalMilesLength(2.0);
            bool b2 = IMUtils::ProjectTargetPosition(target_x, target_y_large, ownship_horizontal_traj,
                                                     xProjected, yProjected, dtgProjected);
            EXPECT_TRUE(b2);
            EXPECT_NEAR(xExpect.value(), xProjected.value(), tol.value());
            EXPECT_NEAR(yExpect.value(), yProjected.value(), tol.value());
            EXPECT_NEAR(dtgExpect.value(), dtgProjected.value(), tol.value());

            /*
             * Check again with a state off the back of the route and with minor cross track error.
             */
            const Units::Length target_x_large = Units::MetersLength(hp.GetXPositionMeters()) + Units::NauticalMilesLength(2.0);
            bool b3 = IMUtils::ProjectTargetPosition(target_x_large, target_y, ownship_horizontal_traj,
                                                     xProjected, yProjected, dtgProjected);
            EXPECT_TRUE(b3);
            EXPECT_NEAR(Units::MetersLength(hp.GetXPositionMeters()).value(), xProjected.value(), tol.value());
            EXPECT_NEAR(Units::MetersLength(hp.GetYPositionMeters()).value(), yProjected.value(), tol.value());
            EXPECT_NEAR(Units::MetersLength(target_x_large).value(), dtgProjected.value(), tol.value());

            /*
             * Check again with a state off the back of the route and with significant cross track error.
             */
            const Units::Length large_cte = target_y + Units::NauticalMilesLength(3.0);
            bool b4 = IMUtils::ProjectTargetPosition(target_x_large, large_cte, ownship_horizontal_traj,
                                                     xProjected, yProjected, dtgProjected);
            EXPECT_FALSE(b4);
            EXPECT_LT(dtgProjected, Units::ZERO_LENGTH);
         }

         TEST(IMUtils, projTargetState_simple)
         {
            // Generate hpath
            //   const Units::MetersLength unity = Units::MetersLength(1);

            vector<HorizontalPath> ownship_horizontal_traj;
            HorizontalPath hp0, hp1, hp2;
            hp0.m_segment_type = HorizontalPath::SegmentType::STRAIGHT;
            hp0.SetXYPositionMeters(1, 1);
            hp0.m_path_length_cumulative_meters = 0;
            hp0.m_path_course = PI / 4;
            ownship_horizontal_traj.push_back(hp0);
            hp1.m_segment_type = HorizontalPath::SegmentType::STRAIGHT;
            hp1.SetXYPositionMeters(2, 2);
            hp1.m_path_length_cumulative_meters = hp0.m_path_length_cumulative_meters + sqrt(2);
            hp1.m_path_course = PI / 4;
            ownship_horizontal_traj.push_back(hp1);
            hp2.m_segment_type = HorizontalPath::SegmentType::STRAIGHT;
            hp2.SetXYPositionMeters(3, 3);
            hp2.m_path_length_cumulative_meters = hp1.m_path_length_cumulative_meters + sqrt(2);
            hp2.m_path_course = PI / 4;
            ownship_horizontal_traj.push_back(hp2);

            // Generate test state vector at constant gs that is
            // near the hpath, but not "on"
            const Units::Speed xd = Units::MetersPerSecondSpeed(-0.5), yd = Units::MetersPerSecondSpeed(-0.5);
            const Units::SecondsTime tstep(1);
            interval_management::AircraftState asv;
            asv.Create(
               1000,
               Units::zero(),
               EarthModel::LocalPositionEnu::Of(Units::MetersLength(hp1.GetXPositionMeters()), Units::MetersLength(hp1.GetYPositionMeters()), Units::zero()),
               Units::FeetPerSecondSpeed(xd),
               Units::FeetPerSecondSpeed(yd),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero()
            );

            Units::Length x, y, dtg;
            IMUtils::ProjectTargetPosition(asv.GetPositionX(), 
                                           asv.GetPositionY(), 
                                           ownship_horizontal_traj, 
                                           x,
                                           y, 
                                           dtg);

            EXPECT_NEAR(hp1.m_path_length_cumulative_meters, Units::MetersLength(dtg).value(), 0.1);
            EXPECT_NEAR(hp1.GetXPositionMeters(), Units::MetersLength(x).value(), 0.1);
            EXPECT_NEAR(hp1.GetYPositionMeters(), Units::MetersLength(y).value(), 0.1);
         }

         TEST(IMUtils, getCrossingTime)
         {
            vector<HorizontalPath> traj;
            HorizontalPath hp;
            hp.m_segment_type = HorizontalPath::SegmentType::STRAIGHT;
            hp.SetXYPositionMeters(1000, 1000);
            hp.m_path_length_cumulative_meters = 0;
            traj.push_back(hp);
            hp.SetXYPositionMeters(1100, 1000);
            hp.m_path_length_cumulative_meters = 100;
            hp.m_path_course = atan2(3, 4);
            traj.push_back(hp);
            hp.SetXYPositionMeters(1200, 1075);
            hp.m_path_length_cumulative_meters = hp.m_path_length_cumulative_meters + sqrt(100 * 100 + 75 * 75);
            traj.push_back(hp);

            Units::SecondsTime crossingTime;
            bool valid;

            vector<interval_management::AircraftState> states;

            // test with empty states vector (invalid). Case E in the algorithm
            valid = IMUtils::GetCrossingTime(Units::MetersLength(150),
                                             states, traj, crossingTime);
            EXPECT_FALSE(valid);
            interval_management::AircraftState s0;
            s0.Create(
               0,
               Units::SecondsTime(55),
               EarthModel::LocalPositionEnu::Of(Units::MetersLength(1140), Units::MetersLength(1030), Units::zero()),
               Units::MetersPerSecondSpeed(10),
               Units::MetersPerSecondSpeed(7.5),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero()
            );
            states.push_back(s0);

            // Case D in the algorithm
            // test with single-element states vector (valid, will extrapolate, see AAES-633)
            valid = IMUtils::GetCrossingTime(Units::MetersLength(100),
                                             states, traj, crossingTime);
            EXPECT_TRUE(valid);
            EXPECT_NEAR(crossingTime.value(), 59, 0.01);
            interval_management::AircraftState s1;
            s1.Create(0,
               Units::SecondsTime(60.1),
               EarthModel::LocalPositionEnu::Of(Units::MetersLength(1099), Units::MetersLength(1000), Units::zero()),
               Units::MetersPerSecondSpeed(10),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero());
            states.push_back(s1);

            interval_management::AircraftState s2;
            s2.Create(
               0,
               Units::SecondsTime(65),
               EarthModel::LocalPositionEnu::Of(Units::MetersLength(1050), Units::MetersLength(1000), Units::zero()),
               Units::MetersPerSecondSpeed(10),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero()
            );
            states.push_back(s2);

            // Case A in the algorithm
            valid = IMUtils::GetCrossingTime(Units::MetersLength(hp.m_path_length_cumulative_meters),
                                             states, traj, crossingTime);
            EXPECT_TRUE(valid);
            EXPECT_NEAR(crossingTime.value(), 50, 1.0);

            // Case B in the algorithm
            valid = IMUtils::GetCrossingTime(Units::MetersLength(150),
                                             states, traj, crossingTime);
            EXPECT_TRUE(valid);
            EXPECT_NEAR(crossingTime.value(), 55, 0.01);

            // Case B in the algorithm
            valid = IMUtils::GetCrossingTime(Units::MetersLength(100),
                                             states, traj, crossingTime);
            EXPECT_TRUE(valid);
            EXPECT_NEAR(crossingTime.value(), 60, 0.01);

            // Case B in the algorithm
            valid = IMUtils::GetCrossingTime(Units::MetersLength(50),
                                             states, traj, crossingTime);
            EXPECT_TRUE(valid);
            EXPECT_NEAR(crossingTime.value(), 65, 0.01);

            // Case C in the algoirthm
            // 2 seconds after last state, should return true with an extrapolated crossing time
            valid = IMUtils::GetCrossingTime(Units::MetersLength(30),
                                             states, traj, crossingTime);
            EXPECT_TRUE(valid);
            EXPECT_DOUBLE_EQ(67, crossingTime.value());
         }

         TEST(IMUtils, calcTimeBasedExtrapolateForward)
         {
            vector<HorizontalPath> ownship_horizontal_traj;
            HorizontalPath hp;
            hp.m_segment_type = HorizontalPath::SegmentType::STRAIGHT;
            hp.SetXYPositionMeters(0, 0);
            hp.m_path_length_cumulative_meters = 0;
            ownship_horizontal_traj.push_back(hp);
            hp.SetXYPositionMeters(100, 0);
            hp.m_path_length_cumulative_meters = 100;
            ownship_horizontal_traj.push_back(hp);

            // Define the geometry so that expected conditions are mathematically valid
            const Units::Speed gs = Units::MetersPerSecondSpeed(50);
            const Units::MetersLength ownship_dtg(100.0);
            const Units::MetersLength target_dtg(150.0);
            const Units::SecondsTime expectedTime = (target_dtg - ownship_dtg) / gs;
            const Units::Length target_x =
                                    Units::MetersLength(hp.GetXPositionMeters()) + gs * expectedTime,
                                target_y = Units::ZERO_LENGTH;
            interval_management::AircraftState target_state;
            target_state.Create(
               0,
               Units::zero(),
               EarthModel::LocalPositionEnu::Of(target_x, target_y, Units::zero()),
               gs,
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero()
            );

            // test with single-element states vector (valid, will extrapolate, see AAES-633)
            const Units::SecondsTime tol(0.001);
            Units::Length x, y, dtg;
            Units::SecondsTime extrapTime;
            IMUtils::CalculateTimeBasedExtrapolate(ownship_dtg,
                                                   target_state,
                                                   ownship_horizontal_traj,
                                                   extrapTime, x, y, dtg);
            EXPECT_NEAR(expectedTime.value(), extrapTime.value(), tol.value());
         }

         TEST(IMUtils, getTimeBasedExtrapolateStateForward)
         {
            /*
             * Place the target behind the ownship and calculate when it will get to ownship's position and what that state will be.
             */

            vector<HorizontalPath> ownship_horizontal_traj;
            HorizontalPath hp;
            hp.m_segment_type = HorizontalPath::SegmentType::STRAIGHT;
            hp.SetXYPositionMeters(0, 0);
            hp.m_path_length_cumulative_meters = 0;
            ownship_horizontal_traj.push_back(hp);
            hp.SetXYPositionMeters(100, 0);
            hp.m_path_length_cumulative_meters = 100;
            ownship_horizontal_traj.push_back(hp);

            // Define the geometry so that expected conditions are mathematically valid
            interval_management::AircraftState ownship_state;
            ownship_state.Create(
               0,
               Units::zero(),
               EarthModel::LocalPositionEnu::Of(Units::MetersLength(hp.GetXPositionMeters()), Units::MetersLength(hp.GetYPositionMeters()), Units::zero()),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero()
            );

            const Units::Speed target_gs = Units::MetersPerSecondSpeed(50);
            const Units::MetersLength ownship_dtg(hp.m_path_length_cumulative_meters);
            const Units::SecondsTime timediff(-1.0); // make them this number of seconds apart, negative means target further away
            const Units::MetersLength target_dtg =
                ownship_dtg - Units::FeetPerSecondSpeed(target_gs) * timediff; // make target further away based on gs
            const Units::SecondsTime expectedTime = (target_dtg - ownship_dtg) / target_gs;
            const Units::Length target_x = Units::MetersLength(hp.GetXPositionMeters()) + (target_dtg - ownship_dtg), target_y = Units::ZERO_LENGTH;
            interval_management::AircraftState target_state;
            target_state.Create(
               0,
               Units::zero(),
               EarthModel::LocalPositionEnu::Of(target_x, target_y, Units::zero()),
               target_gs,
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero()
            );

            // get the estimated state
            Units::SecondsTime msi;
            interval_management::AircraftState actual;
            IMUtils::GetTimeBasedExtrapolateState(ownship_state,
                                                  target_state,
                                                  ownship_horizontal_traj,
                                                  ownship_horizontal_traj,
                                                  msi,
                                                  actual);

            // Asserts
            const Units::SecondsTime toltime(0.001);
            const Units::MetersLength tolxy(1e-5);
            const Units::MetersLength tolxyd(1e-5);
            EXPECT_NEAR(ownship_state.m_x, actual.m_x, tolxy.value());
            EXPECT_NEAR(ownship_state.m_y, actual.m_y, tolxy.value());
            EXPECT_NEAR(target_state.m_xd, actual.m_xd, tolxyd.value());
            EXPECT_NEAR(target_state.m_yd, actual.m_yd, tolxyd.value());
            EXPECT_NEAR(ownship_dtg.value(), actual.m_distance_to_go_meters, tolxy.value());
            EXPECT_NEAR(expectedTime.value(), actual.GetTimeStamp().value(), toltime.value());
            EXPECT_NEAR(timediff.value(), msi.value(), toltime.value());
         }

         TEST(IMUtils, calcTimeBasedExtrapolateBackward)
         {
            vector<HorizontalPath> ownship_horizontal_traj;
            HorizontalPath hp;
            hp.m_segment_type = HorizontalPath::SegmentType::STRAIGHT;
            hp.SetXYPositionMeters(0, 0);
            hp.m_path_length_cumulative_meters = 0;
            ownship_horizontal_traj.push_back(hp);
            hp.SetXYPositionMeters(100, 0);
            hp.m_path_length_cumulative_meters = 100;
            ownship_horizontal_traj.push_back(hp);

            // Define the geometry so that expected conditions are mathematically valid
            const Units::Speed target_gs = Units::MetersPerSecondSpeed(50);
            const Units::MetersLength ownship_dtg(100.0);
            const Units::MetersLength target_dtg(50.0);
            const Units::SecondsTime expectedTime = (target_dtg - ownship_dtg) / target_gs;
            const Units::Length target_x =
                                    Units::MetersLength(hp.GetXPositionMeters()) + target_gs * expectedTime,
                                target_y = Units::ZERO_LENGTH;
            interval_management::AircraftState target_state;
            target_state.Create(
               0,
               Units::zero(),
               EarthModel::LocalPositionEnu::Of(target_x, target_y, Units::zero()),
               target_gs,
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero()
            );

            // test with single-element states vector (valid, will extrapolate, see AAES-633)
            const Units::SecondsTime tol(0.001);
            Units::SecondsTime extrapTime;
            Units::Length x, y, dtg;
            IMUtils::CalculateTimeBasedExtrapolate(ownship_dtg,
                                                   target_state,
                                                   ownship_horizontal_traj,
                                                   extrapTime, x, y, dtg);
            EXPECT_NEAR(expectedTime.value(), extrapTime.value(), tol.value());
         }

         TEST(IMUtils, getMergePoint)
         {
            Units::MetersLength xAC0(0.0), yAC0(-18660.0), xFAP(0.0), yFAP(0.0), xAC1(5000), yAC1(-18660), xM(0), yM(0);
            Units::DegreesAngle courseAC1(120);
            IMUtils::CalculateMergePoint(xAC0, yAC0, xFAP, yFAP, xAC1, yAC1, xM, yM, courseAC1);
            EXPECT_NEAR((Units::MetersLength(xM)).value(), 0.0, 0.5);
            EXPECT_NEAR(yM.value(), -10000.0, 0.5);

            xAC0 = Units::MetersLength(13660);
            yAC0 = Units::MetersLength(5000);
            xFAP = Units::MetersLength(-5000);
            yFAP = Units::MetersLength(5000);
            xAC1 = Units::MetersLength(13660);
            yAC1 = Units::MetersLength(10000);
            courseAC1 = Units::DegreesAngle(210);
            IMUtils::CalculateMergePoint(xAC0, yAC0, xFAP, yFAP, xAC1, yAC1, xM, yM, courseAC1);
            EXPECT_NEAR(xM.value(), 5000.0, 0.5);
            EXPECT_NEAR(yM.value(), 5000.0, 0.5);
         }

         TEST(IMUtils, GetMergePointAllQuadrants)
         {

            const Units::MetersLength tolerance(1e-5);

            // Quadrant 1
            Units::MetersLength xAC0(0.0), yAC0(0.0), xFAP(10.0), yFAP(10.0), xAC1(5.0), yAC1(0.0), xMerge(0), yMerge(0);
            Units::MetersLength expected_merge_x(5.0), expected_merge_y(5.0);
            Units::SignedDegreesAngle courseAC1(90.0);
            IMUtils::CalculateMergePoint(xAC0, yAC0, xFAP, yFAP, xAC1, yAC1, xMerge, yMerge, courseAC1);
            EXPECT_NEAR(expected_merge_x.value(), xMerge.value(), tolerance.value());
            EXPECT_NEAR(expected_merge_y.value(), yMerge.value(), tolerance.value());

            // Quadrant 2
            xAC0 = Units::MetersLength(0.0);
            yAC0 = Units::MetersLength(0.0);
            xFAP = Units::MetersLength(-10.0);
            yFAP = Units::MetersLength(10.0);
            xAC1 = Units::MetersLength(-5.0);
            yAC1 = Units::MetersLength(0.0);
            xMerge = Units::MetersLength(0);
            yMerge = Units::MetersLength(0);
            expected_merge_x = Units::MetersLength(-5.0);
            expected_merge_y = Units::MetersLength(5.0);
            courseAC1 = Units::SignedDegreesAngle(90.0);
            IMUtils::CalculateMergePoint(xAC0, yAC0, xFAP, yFAP, xAC1, yAC1, xMerge, yMerge, courseAC1);
            EXPECT_NEAR(expected_merge_x.value(), xMerge.value(), tolerance.value());
            EXPECT_NEAR(expected_merge_y.value(), yMerge.value(), tolerance.value());

            // Quadrant 3
            xAC0 = Units::MetersLength(0.0);
            yAC0 = Units::MetersLength(0.0);
            xFAP = Units::MetersLength(-10.0);
            yFAP = Units::MetersLength(-10.0);
            xAC1 = Units::MetersLength(-5.0);
            yAC1 = Units::MetersLength(0.0);
            xMerge = Units::MetersLength(0);
            yMerge = Units::MetersLength(0);
            expected_merge_x = Units::MetersLength(-5.0);
            expected_merge_y = Units::MetersLength(-5.0);
            courseAC1 = Units::SignedDegreesAngle(270.0);
            IMUtils::CalculateMergePoint(xAC0, yAC0, xFAP, yFAP, xAC1, yAC1, xMerge, yMerge, courseAC1);
            EXPECT_NEAR(expected_merge_x.value(), xMerge.value(), tolerance.value());
            EXPECT_NEAR(expected_merge_y.value(), yMerge.value(), tolerance.value());

            // Quadrant 4
            xAC0 = Units::MetersLength(0.0);
            yAC0 = Units::MetersLength(0.0);
            xFAP = Units::MetersLength(10.0);
            yFAP = Units::MetersLength(-10.0);
            xAC1 = Units::MetersLength(5.0);
            yAC1 = Units::MetersLength(0.0);
            xMerge = Units::MetersLength(0);
            yMerge = Units::MetersLength(0);
            expected_merge_x = Units::MetersLength(5.0);
            expected_merge_y = Units::MetersLength(-5.0);
            courseAC1 = Units::SignedDegreesAngle(270.0);
            IMUtils::CalculateMergePoint(xAC0, yAC0, xFAP, yFAP, xAC1, yAC1, xMerge, yMerge, courseAC1);
            EXPECT_NEAR(expected_merge_x.value(), xMerge.value(), tolerance.value());
            EXPECT_NEAR(expected_merge_y.value(), yMerge.value(), tolerance.value());
         }

         TEST(IMUtils, GetTargetState_aaes798)
         {
            // Generate hpath
            const Units::MetersLength basic_segment_length = -IMUtils::BEYOND_END_OF_ROUTE_TOL;
            vector<HorizontalPath> ownship_horizontal_traj;
            HorizontalPath hp0, hp1, hp2;
            hp0.m_segment_type = HorizontalPath::SegmentType::STRAIGHT;
            hp0.SetXYPositionMeters(0, 0);
            hp0.m_path_length_cumulative_meters = 0;
            hp0.m_path_course = PI / 4;
            ownship_horizontal_traj.push_back(hp0);
            hp1.m_segment_type = HorizontalPath::SegmentType::STRAIGHT;
            hp1.SetXYPositionMeters(basic_segment_length.value(), basic_segment_length.value());
            hp1.m_path_length_cumulative_meters = hp0.m_path_length_cumulative_meters + basic_segment_length.value() * sqrt(2);
            hp1.m_path_course = PI / 4;
            ownship_horizontal_traj.push_back(hp1);
            hp2.m_segment_type = HorizontalPath::SegmentType::STRAIGHT;
            hp2.SetXYPositionMeters(2 * basic_segment_length.value(), 2 * basic_segment_length.value());
            hp2.m_path_length_cumulative_meters = hp1.m_path_length_cumulative_meters + basic_segment_length.value() * sqrt(2);
            hp2.m_path_course = PI / 4;
            ownship_horizontal_traj.push_back(hp2);

            // Generate test state vector at constant gs that is
            // near the hpath, but not "on". The ground speed is chosen
            // to ensure the staes go "off" the path
            const Units::SecondsTime tstep(1);
            const Units::Speed xd = -0.75 * basic_segment_length / tstep, yd = -0.75 * basic_segment_length / tstep;
            vector<interval_management::AircraftState> asv(5);
            EarthModel::LocalPositionEnu enu_pos;
            enu_pos.x = Units::MetersLength(hp2.GetXPositionMeters()) - Units::FeetLength(0.1);
            enu_pos.y = Units::MetersLength(hp2.GetYPositionMeters()) - Units::FeetLength(0.2);
            enu_pos.z = Units::zero();
            asv[0].Create(
               5,
               Units::SecondsTime(1000),
               enu_pos,
               Units::FeetPerSecondSpeed(xd),
               Units::FeetPerSecondSpeed(yd),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero()
            );

            for (int i = 1; i < asv.size(); ++i)
            {
               enu_pos.x = asv[i - 1].GetPositionX() + Units::FeetLength(xd * tstep);
               enu_pos.y = asv[i - 1].GetPositionY() + Units::FeetLength(yd * tstep);
               enu_pos.z = Units::zero();
               asv[i].Create(
                  asv[i - 1].GetId(),
                  asv[i - 1].GetTimeStamp() + tstep,
                  enu_pos,
                  Units::FeetPerSecondSpeed(xd),
                  Units::FeetPerSecondSpeed(yd),
                  Units::zero(),
                  Units::zero(),
                  Units::zero(),
                  Units::zero(),
                  Units::zero(),
                  Units::zero(),
                  Units::zero(),
                  Units::zero()
               );
            }

            // Tests
            bool state_valid;
            // time valid, but no state on h_traj
            interval_management::AircraftState interp_result = IMUtils::GetProjectedTargetState(asv, ownship_horizontal_traj,
                                                                           Units::SecondsTime(asv.back().GetTimeStamp().value() - 0.2),
                                                                           Units::Angle(),
                                                                           state_valid); // interpolate
            EXPECT_FALSE(state_valid);

            interval_management::AircraftState extrap_result = IMUtils::GetProjectedTargetState(asv, ownship_horizontal_traj,
                                                                           Units::SecondsTime(asv.back().GetTimeStamp().value() + 0.2),
                                                                           Units::Angle(),
                                                                           state_valid); // extrapolate
            EXPECT_FALSE(state_valid);
         }

         TEST(IMUtils, TargetState)
         {
            vector<HorizontalPath> ownship_horizontal_traj;
            HorizontalPath hp0, hp1, hp2;
            hp0.m_segment_type = HorizontalPath::SegmentType::STRAIGHT;
            hp0.SetXYPositionMeters(1, 1);
            hp0.m_path_length_cumulative_meters = 0;
            hp0.m_path_course = PI / 4;
            ownship_horizontal_traj.push_back(hp0);
            hp1.m_segment_type = HorizontalPath::SegmentType::STRAIGHT;
            hp1.SetXYPositionMeters(2, 2);
            hp1.m_path_length_cumulative_meters = hp0.m_path_length_cumulative_meters + sqrt(2);
            hp1.m_path_course = PI / 4;
            ownship_horizontal_traj.push_back(hp1);
            hp2.m_segment_type = HorizontalPath::SegmentType::STRAIGHT;
            hp2.SetXYPositionMeters(3, 3);
            hp2.m_path_length_cumulative_meters = hp1.m_path_length_cumulative_meters + sqrt(2);
            hp2.m_path_course = PI / 4;
            ownship_horizontal_traj.push_back(hp2);

            // Generate test state vector at constant gs that is
            // near the hpath, but not "on"
            const Units::Speed xd = Units::MetersPerSecondSpeed(-0.5), yd = Units::MetersPerSecondSpeed(-0.5);
            const Units::SecondsTime tstep(1);
            vector<interval_management::AircraftState> asv(5);
            EarthModel::LocalPositionEnu enu_pos;
            enu_pos.x = Units::MetersLength(hp2.GetXPositionMeters()) - Units::FeetLength(0.1);
            enu_pos.y = Units::MetersLength(hp2.GetYPositionMeters()) - Units::FeetLength(0.2);
            enu_pos.z = Units::zero();
            asv[0].Create(
               5,
               Units::zero(),
               enu_pos,
               Units::FeetPerSecondSpeed(xd),
               Units::FeetPerSecondSpeed(yd),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero()
            );

            for (int i = 1; i < asv.size(); ++i)
            {
               EarthModel::LocalPositionEnu enu_pos;
               enu_pos.x = asv[i - 1].GetPositionX() + Units::FeetLength(xd * tstep);
               enu_pos.y = asv[i - 1].GetPositionY() + Units::FeetLength(yd * tstep);
               enu_pos.z = Units::zero();
               asv[i].Create(
                  5,
                  asv[i - 1].GetTimeStamp() + tstep,
                  enu_pos,
                  Units::FeetPerSecondSpeed(xd),
                  Units::FeetPerSecondSpeed(yd),
                  Units::zero(),
                  Units::zero(),
                  Units::zero(),
                  Units::zero(),
                  Units::zero(),
                  Units::zero(),
                  Units::zero(),
                  Units::zero()
               );

            }

            // Tests
            bool built;
            interval_management::AircraftState current_ownship_state;
            current_ownship_state.Create(
               5,
               Units::zero(),
               enu_pos,
               xd,
               yd,
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero()
            );

            // time-too-early, hp2 extrapolated
            interval_management::AircraftState result = IMUtils::GetProjectedTargetState(
               asv, 
               ownship_horizontal_traj, 
               Units::SecondsTime(asv.front().GetTimeStamp().value() - 0.5),
               current_ownship_state.GetHeadingCcwFromEastRadians(),
               built);
            EXPECT_TRUE(built);
            EXPECT_DOUBLE_EQ(asv.front().GetTimeStamp().value() - 0.5, result.GetTimeStamp().value());
            const Units::FeetLength expectxbefore =
                Units::MetersLength(hp2.GetXPositionMeters()) - xd *
                                                                    cos(current_ownship_state.GetHeadingCcwFromEastRadians()) * Units::SecondsTime(0.5);
            const Units::FeetLength expectybefore =
                Units::MetersLength(hp2.GetYPositionMeters()) - yd *
                                                                    sin(current_ownship_state.GetHeadingCcwFromEastRadians()) * Units::SecondsTime(0.5);

            EXPECT_NEAR(expectxbefore.value(), Units::FeetLength(result.GetPositionX()).value(), 0.2);
            EXPECT_NEAR(expectybefore.value(), Units::FeetLength(result.GetPositionY()).value(), 0.2);

            // time at beginning, should be basically hp2
            result = IMUtils::GetProjectedTargetState(asv, ownship_horizontal_traj, asv.front().GetTimeStamp(),
                                                      Units::Angle(),
                                                      built);
            EXPECT_TRUE(built);
            EXPECT_DOUBLE_EQ(asv.front().GetTimeStamp().value(), result.GetTimeStamp().value());
            EXPECT_NEAR(Units::FeetLength(Units::MetersLength(hp2.GetXPositionMeters())).value(), Units::FeetLength(result.GetPositionX()).value(), 0.2);
            EXPECT_NEAR(Units::FeetLength(Units::MetersLength(hp2.GetYPositionMeters())).value(), Units::FeetLength(result.GetPositionY()).value(), 0.2);

            // time between elements 0 and 1
            const Units::SecondsTime test1_tstep(0.5);
            const Units::Length xboundupper = Units::FeetLength(asv[0].m_x), xboundlower = Units::FeetLength(asv[1].m_x);
            const Units::Length yboundupper = Units::FeetLength(asv[0].m_y), yboundlower = Units::FeetLength(asv[1].m_y);
            result = IMUtils::GetProjectedTargetState(asv, ownship_horizontal_traj, Units::SecondsTime(asv[0].GetTimeStamp().value() + .5),
                                                      Units::Angle(),
                                                      built);
            EXPECT_TRUE(built);
            EXPECT_DOUBLE_EQ(Units::SecondsTime(asv[0].GetTimeStamp().value() + .5).value(), result.GetTimeStamp().value());
            EXPECT_LT(Units::FeetLength(result.GetPositionX()).value(), Units::FeetLength(xboundupper).value());
            EXPECT_GT(Units::FeetLength(result.GetPositionX()).value(), Units::FeetLength(xboundlower).value());
            EXPECT_LT(Units::FeetLength(result.GetPositionY()).value(), Units::FeetLength(yboundupper).value());
            EXPECT_GT(Units::FeetLength(result.GetPositionY()).value(), Units::FeetLength(yboundlower).value());

            // time at element 2, should be basically hp1
            result = IMUtils::GetProjectedTargetState(asv, ownship_horizontal_traj, asv[2].GetTimeStamp(),
                                                      Units::Angle(), built);
            EXPECT_TRUE(built);
            EXPECT_DOUBLE_EQ(asv[2].GetTimeStamp().value(), result.GetTimeStamp().value());
            EXPECT_NEAR(Units::FeetLength(Units::MetersLength(hp1.GetXPositionMeters())).value(), Units::FeetLength(result.GetPositionX()).value(), 0.2);
            EXPECT_NEAR(Units::FeetLength(Units::MetersLength(hp1.GetYPositionMeters())).value(), Units::FeetLength(result.GetPositionY()).value(), 0.2);

            // time at last element, should be basically hp0
            result = IMUtils::GetProjectedTargetState(asv, ownship_horizontal_traj, asv.back().GetTimeStamp(),
                                                      Units::Angle(), built);
            EXPECT_TRUE(built);
            EXPECT_DOUBLE_EQ(asv.back().GetTimeStamp().value(), result.GetTimeStamp().value());
            EXPECT_NEAR(Units::FeetLength(Units::MetersLength(hp0.GetXPositionMeters())).value(), Units::FeetLength(result.GetPositionX()).value(), 0.2);
            EXPECT_NEAR(Units::FeetLength(Units::MetersLength(hp0.GetYPositionMeters())).value(), Units::FeetLength(result.GetPositionY()).value(), 0.2);

            // time after last element, hp0 extrapolated
            result = IMUtils::GetProjectedTargetState(asv, ownship_horizontal_traj, Units::SecondsTime(asv.back().GetTimeStamp().value() + 0.5),
                                                      Units::Angle(), built);
            EXPECT_FALSE(built);
            EXPECT_DOUBLE_EQ(asv.back().GetTimeStamp().value() + 0.5, result.GetTimeStamp().value());
            const Units::FeetLength expectx = Units::MetersLength(hp0.GetXPositionMeters()) + xd * Units::SecondsTime(0.5);
            const Units::FeetLength expecty = Units::MetersLength(hp0.GetYPositionMeters()) + yd * Units::SecondsTime(0.5);
            EXPECT_NEAR(expectx.value(), Units::FeetLength(result.GetPositionX()).value(), 0.2);
            EXPECT_NEAR(expecty.value(), Units::FeetLength(result.GetPositionY()).value(), 0.2);
         }

         TEST(IMUtils, GetTargetStateAtGivenDtgOnOwnshipsPath)
         {

            // Generate straight-line path in one dimension
            vector<HorizontalPath> ownship_horizontal_traj;
            HorizontalPath hp0, hp1; //, hp2;
            hp0.m_segment_type = HorizontalPath::SegmentType::STRAIGHT;
            hp0.SetXYPositionMeters(0, 0);
            hp0.m_path_length_cumulative_meters = 0;
            hp0.m_path_course = 0;
            ownship_horizontal_traj.push_back(hp0);
            hp1.m_segment_type = HorizontalPath::SegmentType::STRAIGHT;
            hp1.SetXYPositionMeters(1, 0);
            hp1.m_path_length_cumulative_meters = 1;
            hp1.m_path_course = 0;
            ownship_horizontal_traj.push_back(hp1);

            // Generate test state vector at constant gs that is
            // near the hpath, but not "on"
            const Units::Speed xd = Units::MetersPerSecondSpeed(-0.2);
            const Units::SecondsTime time_step(1);

            std::vector<interval_management::AircraftState> aircraft_state_history(6);
            EarthModel::LocalPositionEnu enu_pos;
            enu_pos.x = Units::MetersLength(hp1.GetXPositionMeters());
            enu_pos.y = Units::FeetLength(0.5);
            enu_pos.z = Units::zero();
            aircraft_state_history[0].Create(
               5,
               Units::zero(),
               enu_pos,
               Units::FeetPerSecondSpeed(xd),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero()
            );
            aircraft_state_history[0].m_distance_to_go_meters = Units::MetersLength(hp1.GetXPositionMeters()).value();

            for (int i = 1; i < aircraft_state_history.size(); ++i)
            {
               enu_pos.x = aircraft_state_history[i - 1].GetPositionX() + Units::FeetLength(xd * time_step);
               enu_pos.y = Units::FeetLength(0.5);
               enu_pos.z = Units::zero();
               aircraft_state_history[i].Create(
                  aircraft_state_history[i - 1].GetId(),
                  aircraft_state_history[i - 1].GetTimeStamp() + time_step,
                  enu_pos,
                  Units::FeetPerSecondSpeed(xd),
                  Units::zero(),
                  Units::zero(),
                  Units::zero(),
                  Units::zero(),
                  Units::zero(),
                  Units::zero(),
                  Units::zero(),
                  Units::zero(),
                  Units::zero()
               );
               aircraft_state_history[i].m_distance_to_go_meters =
                   Units::MetersLength(aircraft_state_history[i].GetPositionX()).value();

            }

            interval_management::AircraftState result;

            // the y component of the ownship route is zero, so we expect all y values to get projected to 0.
            Units::Length expected_y_value = Units::ZERO_LENGTH;

            interval_management::AircraftState current_state;
            current_state.m_x = Units::FeetLength(Units::MetersLength(ownship_horizontal_traj.back().GetXPositionMeters())).value();
            current_state.m_y = 0;
            current_state.m_xd = Units::FeetPerSecondSpeed(xd).value();
            current_state.m_yd = 0;

            // target distance beyond beginning of ownship route
            Units::MetersLength distance_before_beginning_of_route(1.5);
            result = IMUtils::GetTargetStateOnOwnshipPathForDtg(aircraft_state_history, ownship_horizontal_traj,
                                                                distance_before_beginning_of_route,
                                                                current_state.GetHeadingCcwFromEastRadians());

            Units::Length target_x =
                distance_before_beginning_of_route -
                Units::MetersLength(ownship_horizontal_traj.back().m_path_length_cumulative_meters);
            EXPECT_DOUBLE_EQ(Units::FeetLength(target_x).value(), Units::FeetLength(result.m_x).value());
            EXPECT_NEAR(Units::MetersLength(hp1.GetYPositionMeters()).value(),
                        Units::MetersLength(Units::FeetLength(result.m_y)).value(), 0.0001);

            // distance between elements 0 and 1
            const Units::Length xboundupper = Units::MetersLength(aircraft_state_history[0].m_distance_to_go_meters);
            const Units::Length xboundlower = Units::MetersLength(aircraft_state_history[1].m_distance_to_go_meters);
            result = IMUtils::GetTargetStateOnOwnshipPathForDtg(aircraft_state_history, ownship_horizontal_traj,
                                                                Units::MetersLength(
                                                                    aircraft_state_history[0].m_distance_to_go_meters - 0.1),
                                                                Units::Angle());
            EXPECT_DOUBLE_EQ(Units::SecondsTime(aircraft_state_history[0].GetTimeStamp().value() + 0.5).value(), result.GetTimeStamp().value());
            EXPECT_LT(result.m_x, Units::FeetLength(xboundupper).value());
            EXPECT_GT(result.m_x, Units::FeetLength(xboundlower).value());
            EXPECT_DOUBLE_EQ(Units::FeetLength(expected_y_value).value(), Units::FeetLength(result.m_y).value());

            // distance at element 2
            result = IMUtils::GetTargetStateOnOwnshipPathForDtg(aircraft_state_history, ownship_horizontal_traj,
                                                                Units::MetersLength(aircraft_state_history[2].m_distance_to_go_meters),
                                                                Units::Angle());
            EXPECT_DOUBLE_EQ(aircraft_state_history[2].GetTimeStamp().value(), result.GetTimeStamp().value());
            EXPECT_NEAR(Units::FeetLength(aircraft_state_history[2].m_x).value(), Units::FeetLength(result.m_x).value(), 1e-10);
            EXPECT_DOUBLE_EQ(Units::FeetLength(expected_y_value).value(), Units::FeetLength(result.m_y).value());

            // target at end of route
            result = IMUtils::GetTargetStateOnOwnshipPathForDtg(aircraft_state_history, ownship_horizontal_traj,
                                                                Units::ZERO_LENGTH, Units::Angle());
            EXPECT_DOUBLE_EQ(aircraft_state_history.back().GetTimeStamp().value(), result.GetTimeStamp().value());
            EXPECT_NEAR(Units::FeetLength(aircraft_state_history.back().m_x).value(), Units::FeetLength(result.m_x).value(), 1e-10);
            EXPECT_DOUBLE_EQ(Units::FeetLength(expected_y_value).value(), Units::FeetLength(result.m_y).value());

            // target distance past end of ownship route
            Units::MetersLength distance_beyond_end_of_route(-0.5);
            result = IMUtils::GetTargetStateOnOwnshipPathForDtg(aircraft_state_history, ownship_horizontal_traj,
                                                                distance_beyond_end_of_route, Units::Angle());

            EXPECT_DOUBLE_EQ(Units::FeetLength(distance_beyond_end_of_route).value(), Units::FeetLength(result.m_x).value());
            EXPECT_DOUBLE_EQ(0, Units::FeetLength(result.m_y).value());
         }

         TEST(AircraftState, Create) {
            auto actual_id = 100;
            auto actual_x = Units::MetersLength(10);
            auto actual_y = Units::FeetLength(20);
            auto actual_z = Units::NauticalMilesLength(1);
            auto actual_position = EarthModel::LocalPositionEnu::Of(actual_x, actual_y, actual_z);
            auto actual_xd = Units::MetersPerSecondSpeed(10);
            auto actual_yd = Units::FeetPerSecondSpeed(20);
            auto actual_zd = Units::KnotsSpeed(1);
            auto actual_gamma = Units::DegreesAngle(2);
            auto actual_time = Units::MinutesTime(1);
            
            interval_management::AircraftState* test_state = new interval_management::AircraftState();
            test_state->Create(
               actual_id,
               actual_time,
               actual_position,
               actual_xd,
               actual_yd,
               actual_zd,
               actual_gamma,
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero(),
               Units::zero()
            );

            EXPECT_EQ(actual_id, test_state->GetId());
            EXPECT_DOUBLE_EQ(Units::SecondsTime(actual_time).value(), test_state->GetTimeStamp().value());
            EXPECT_DOUBLE_EQ(Units::MetersLength(actual_x).value(), Units::MetersLength(test_state->GetPositionX()).value());
            EXPECT_DOUBLE_EQ(Units::MetersLength(actual_y).value(), Units::MetersLength(test_state->GetPositionY()).value());
            EXPECT_DOUBLE_EQ(Units::MetersLength(actual_z).value(), Units::MetersLength(test_state->GetPositionZ()).value());
            EXPECT_DOUBLE_EQ(Units::MetersPerSecondSpeed(actual_xd).value(), Units::MetersPerSecondSpeed(test_state->GetSpeedXd()).value());
            EXPECT_DOUBLE_EQ(Units::MetersPerSecondSpeed(actual_yd).value(), Units::MetersPerSecondSpeed(test_state->GetSpeedYd()).value());
            EXPECT_DOUBLE_EQ(Units::MetersPerSecondSpeed(actual_zd).value(), Units::MetersPerSecondSpeed(test_state->GetSpeedZd()).value());
            EXPECT_DOUBLE_EQ(Units::RadiansAngle(actual_gamma).value(), Units::RadiansAngle(test_state->GetGamma()).value());
         }

         TEST(IMUtils, ConvertAircraftStates){
            auto actual_id = 100;
            auto actual_x = Units::MetersLength(10);
            auto actual_y = Units::FeetLength(20);
            auto actual_z = Units::NauticalMilesLength(1);
            auto actual_position = EarthModel::LocalPositionEnu::Of(actual_x, actual_y, actual_z);
            auto actual_xd = Units::MetersPerSecondSpeed(10);
            auto actual_yd = Units::FeetPerSecondSpeed(20);
            auto actual_zd = Units::KnotsSpeed(1);
            auto actual_gamma = Units::DegreesAngle(2);
            auto actual_time = Units::MinutesTime(1);
            auto actual_psi = Units::SignedRadiansAngle(Units::arctan2(actual_xd.value(), actual_yd.value()));
            auto actual_sensed_veast = Units::MetersPerSecondSpeed(10);
            auto actual_sensed_vnorth = Units::MetersPerSecondSpeed(5);
            auto actual_sensed_wind_parallel = Units::MetersPerSecondSpeed(3);
            auto actual_sensed_wind_perpendicular = Units::MetersPerSecondSpeed(1);
            auto actual_sensed_temperature = Units::CelsiusTemperature(22);
            interval_management::AircraftState* test_state = new interval_management::AircraftState();
            test_state->Create(
               actual_id,
               actual_time,
               actual_position,
               actual_xd,
               actual_yd,
               actual_zd,
               actual_gamma,
               actual_sensed_veast,
               actual_sensed_vnorth,
               actual_sensed_wind_parallel,
               actual_sensed_wind_perpendicular,
               actual_sensed_temperature,
               actual_psi
            );

            // This is the tested method
            // convert to aaesim::open_source::AircraftState
            const aaesim::open_source::AircraftState converted_state_1 = IMUtils::ConvertToAaesimAircraftState(*test_state);

            // assert data transferred
            EXPECT_EQ(actual_id, converted_state_1.m_id);
            EXPECT_DOUBLE_EQ(Units::SecondsTime(actual_time).value(), converted_state_1.m_time);
            EXPECT_DOUBLE_EQ(Units::MetersLength(actual_x).value(), Units::MetersLength(converted_state_1.GetPositionX()).value());
            EXPECT_DOUBLE_EQ(Units::MetersLength(actual_y).value(), Units::MetersLength(converted_state_1.GetPositionY()).value());
            EXPECT_DOUBLE_EQ(Units::MetersLength(actual_z).value(), Units::MetersLength(converted_state_1.GetPositionZ()).value());
            EXPECT_DOUBLE_EQ(Units::MetersPerSecondSpeed(actual_xd).value(), Units::MetersPerSecondSpeed(converted_state_1.GetSpeedXd()).value());
            EXPECT_DOUBLE_EQ(Units::MetersPerSecondSpeed(actual_yd).value(), Units::MetersPerSecondSpeed(converted_state_1.GetSpeedYd()).value());
            EXPECT_DOUBLE_EQ(Units::MetersPerSecondSpeed(actual_zd).value(), Units::MetersPerSecondSpeed(converted_state_1.GetSpeedZd()).value());
            EXPECT_DOUBLE_EQ(Units::RadiansAngle(actual_gamma).value(), converted_state_1.m_gamma);
            EXPECT_DOUBLE_EQ(Units::RadiansAngle(actual_psi).value(), Units::RadiansAngle(converted_state_1.m_psi).value());
            EXPECT_DOUBLE_EQ(Units::CelsiusTemperature(actual_sensed_temperature).value(), Units::CelsiusTemperature(converted_state_1.m_sensed_temperature).value());
            EXPECT_DOUBLE_EQ(Units::MetersPerSecondSpeed(actual_sensed_veast).value(), converted_state_1.m_Vwx);
            EXPECT_DOUBLE_EQ(Units::MetersPerSecondSpeed(actual_sensed_vnorth).value(), converted_state_1.m_Vwy);
            EXPECT_DOUBLE_EQ(Units::MetersPerSecondSpeed(actual_sensed_wind_parallel).value(), converted_state_1.m_Vw_para);
            EXPECT_DOUBLE_EQ(Units::MetersPerSecondSpeed(actual_sensed_wind_perpendicular).value(), converted_state_1.m_Vw_perp);

            // This is the tested method
            // convert to interval_management::AircraftState
            const interval_management::AircraftState converted_state_2 = IMUtils::ConvertToIntervalManagementAircraftState(converted_state_1);

            // assert data transferred
            EXPECT_EQ(actual_id, converted_state_2.GetId());
            EXPECT_DOUBLE_EQ(Units::SecondsTime(actual_time).value(), converted_state_2.GetTimeStamp().value());
            EXPECT_DOUBLE_EQ(Units::MetersLength(actual_x).value(), Units::MetersLength(converted_state_2.GetPositionX()).value());
            EXPECT_DOUBLE_EQ(Units::MetersLength(actual_y).value(), Units::MetersLength(converted_state_2.GetPositionY()).value());
            EXPECT_DOUBLE_EQ(Units::MetersLength(actual_z).value(), Units::MetersLength(converted_state_2.GetPositionZ()).value());
            EXPECT_DOUBLE_EQ(Units::MetersPerSecondSpeed(actual_xd).value(), Units::MetersPerSecondSpeed(converted_state_2.GetSpeedXd()).value());
            EXPECT_DOUBLE_EQ(Units::MetersPerSecondSpeed(actual_yd).value(), Units::MetersPerSecondSpeed(converted_state_2.GetSpeedYd()).value());
            EXPECT_DOUBLE_EQ(Units::MetersPerSecondSpeed(actual_zd).value(), Units::MetersPerSecondSpeed(converted_state_2.GetSpeedZd()).value());
            EXPECT_DOUBLE_EQ(Units::RadiansAngle(actual_gamma).value(), Units::RadiansAngle(converted_state_2.GetGamma()).value());
            EXPECT_DOUBLE_EQ(Units::RadiansAngle(actual_psi).value(), Units::RadiansAngle(converted_state_2.GetPsi()).value());
            EXPECT_DOUBLE_EQ(Units::CelsiusTemperature(actual_sensed_temperature).value(), Units::CelsiusTemperature(converted_state_2.GetSensedTemperature()).value());
            EXPECT_DOUBLE_EQ(Units::MetersPerSecondSpeed(actual_sensed_veast).value(), Units::MetersPerSecondSpeed(converted_state_2.GetSensedWindEastComponent()).value());
            EXPECT_DOUBLE_EQ(Units::MetersPerSecondSpeed(actual_sensed_vnorth).value(), Units::MetersPerSecondSpeed(converted_state_2.GetSensedWindNorthComponent()).value());
            EXPECT_DOUBLE_EQ(Units::MetersPerSecondSpeed(actual_sensed_wind_parallel).value(), Units::MetersPerSecondSpeed(converted_state_2.GetSensedWindParallelComponent()).value());
            EXPECT_DOUBLE_EQ(Units::MetersPerSecondSpeed(actual_sensed_wind_perpendicular).value(), Units::MetersPerSecondSpeed(converted_state_2.GetSensedWindPerpendicularComponent()).value());
         }
      }
   }
}