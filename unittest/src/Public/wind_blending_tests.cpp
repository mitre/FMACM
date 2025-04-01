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

#include "public/WeatherPrediction.h"
#include "public/BlendWindsVerticallyByAltitude.h"

using namespace std;
using namespace aaesim::open_source;

namespace aaesim::test {

TEST(BlendWindsVerticallyByAltitude, update_predicted_winds_at_altitude_from_sensed_wind) {

   // Get expected values
   const int predicted_matrix_rows = 4;
   const Units::Speed predicted_wind_at_20k_x(Units::KnotsSpeed(20));
   const Units::Speed predicted_wind_at_10k_x(Units::KnotsSpeed(10));
   const Units::Speed predicted_wind_at_5k_x = predicted_wind_at_10k_x;  // make 5k and 10k match
   const Units::Speed predicted_wind_at_0k_x(Units::KnotsSpeed(5));

   // Set up test objects that will have an obvious expected output
   const Units::Speed Vwx(Units::KnotsSpeed(20));
   const Units::Speed Vwy = Vwx;
   const auto test_state =
         AircraftState::Builder(0, 0).AltitudeMsl(Units::FeetLength(7500))->SensedWindComponents(Vwx, Vwy)->Build();

   WindStack predicted_wind_x(1, predicted_matrix_rows);  // set up bounds just like in main simulation
   predicted_wind_x.Insert(1, Units::FeetLength(20000), predicted_wind_at_20k_x);
   predicted_wind_x.Insert(2, Units::FeetLength(10000), predicted_wind_at_10k_x);
   predicted_wind_x.Insert(3, Units::FeetLength(5000), predicted_wind_at_5k_x);
   predicted_wind_x.Insert(4, Units::FeetLength(0), predicted_wind_at_0k_x);
   predicted_wind_x.SortAltitudesAscending();

   WeatherPrediction predicted_wind;
   predicted_wind.east_west() = predicted_wind_x;
   predicted_wind.north_south() = predicted_wind_x;  // copy

   // Test
   aaesim::open_source::BlendWindsVerticallyByAltitude wind_blender{};
   wind_blender.BlendSensedWithPredicted(test_state, predicted_wind);

   // Assert
   // Expecting an update to occur between rows 1 & 2. Row 3 values should be unaffected, but it will be shifted to
   // row 4.
   EXPECT_EQ(predicted_matrix_rows + 1, predicted_wind.east_west().GetMaxRow());  // There should be one new row
   EXPECT_EQ(Units::FeetLength(test_state.GetAltitudeMsl()).value(),
             predicted_wind.east_west().GetAltitude(3).value());  // test that new altitude is in correct location
   EXPECT_EQ(
         Units::KnotsSpeed(Vwx).value(),
         predicted_wind.east_west().GetSpeed(3).value());  // test that sensed Vwx is in correct location & same value
   EXPECT_EQ(
         Units::KnotsSpeed(Vwy).value(),
         predicted_wind.north_south().GetSpeed(3).value());  // test that sensed Vwy is in correct location & same value
   EXPECT_EQ(Units::KnotsSpeed(predicted_wind_at_10k_x).value() * 1.5,
             predicted_wind.east_west().GetSpeed(4).value());  // test that blending occurred correctly at the next
                                                               // altitude band up
   EXPECT_EQ(Units::KnotsSpeed(predicted_wind_at_10k_x).value() * 1.5,
             predicted_wind.north_south().GetSpeed(4).value());  // test that blending occurred correctly at the next
                                                                 // altitude band up
   EXPECT_EQ(Units::KnotsSpeed(predicted_wind_at_5k_x).value() * 1.5,
             predicted_wind.east_west().GetSpeed(2).value());  // test that blending occurred correctly at the next
                                                               // altitude band up
   EXPECT_EQ(Units::KnotsSpeed(predicted_wind_at_5k_x).value() * 1.5,
             predicted_wind.north_south().GetSpeed(2).value());  // test that blending occurred correctly at the next
                                                                 // altitude band up
   EXPECT_EQ(Units::KnotsSpeed(predicted_wind_at_20k_x).value(),
             predicted_wind.east_west().GetSpeed(predicted_matrix_rows + 1).value());  // test that blending DID NOT
                                                                                       // occur at the 20k row
   EXPECT_EQ(Units::KnotsSpeed(predicted_wind_at_20k_x).value(),
             predicted_wind.north_south().GetSpeed(predicted_matrix_rows + 1).value());  // test that blending DID NOT
                                                                                         // occur at the 20k row
   EXPECT_EQ(Units::KnotsSpeed(predicted_wind_at_0k_x).value(),
             predicted_wind.east_west().GetSpeed(1).value());  // test that blending DID NOT occur at the 0k row
   EXPECT_EQ(Units::KnotsSpeed(predicted_wind_at_0k_x).value(),
             predicted_wind.north_south().GetSpeed(1).value());  // test that blending DID NOT occur at the 0k row
}

TEST(BlendWindsVerticallyByAltitude, update_predicted_winds_at_higher_altitude) {

   // Set expected values
   const int predicted_matrix_rows = 4;
   const Units::Speed predicted_wind_at_20k_x(Units::KnotsSpeed(20));
   const Units::Speed predicted_wind_at_10k_x(Units::KnotsSpeed(10));
   const Units::Speed predicted_wind_at_5k_x = predicted_wind_at_10k_x;  // make 5k and 10k match
   const Units::Speed predicted_wind_at_0k_x(Units::KnotsSpeed(5));

   // Set up test objects that will have an obvious expected output
   const Units::Speed Vwx(Units::KnotsSpeed(20));
   const Units::Speed Vwy = Vwx;
   const auto test_state =
         AircraftState::Builder(0, 0).AltitudeMsl(Units::FeetLength(25000))->SensedWindComponents(Vwx, Vwy)->Build();

   WindStack predicted_wind_x(1, predicted_matrix_rows);  // set up bounds just like in main simulation
   predicted_wind_x.Insert(1, Units::FeetLength(20000), predicted_wind_at_20k_x);
   predicted_wind_x.Insert(2, Units::FeetLength(10000), predicted_wind_at_10k_x);
   predicted_wind_x.Insert(3, Units::FeetLength(5000), predicted_wind_at_5k_x);
   predicted_wind_x.Insert(4, Units::FeetLength(0), predicted_wind_at_0k_x);
   predicted_wind_x.SortAltitudesAscending();

   WeatherPrediction predicted_wind;
   predicted_wind.east_west() = predicted_wind_x;
   predicted_wind.north_south() = predicted_wind_x;  // copy

   // Test
   aaesim::open_source::BlendWindsVerticallyByAltitude wind_blender{};
   wind_blender.BlendSensedWithPredicted(test_state, predicted_wind);

   // Assert
   // Expecting an update to occur at max_row
   EXPECT_EQ(predicted_matrix_rows + 1, predicted_wind.east_west().GetMaxRow());  // There should be one new row
   EXPECT_EQ(
         Units::FeetLength(test_state.GetAltitudeMsl()).value(),
         predicted_wind.east_west().GetAltitude(predicted_wind.east_west().GetMaxRow()).value());  // test that new
                                                                                                   // altitude is in
                                                                                                   // correct location
}

}  // namespace aaesim::test
