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

/**
 * Tests here are for code in the AircraftDynamicsTestFramework source folder only.
 */

#include <framework/WeatherTruthByTime.h>
#include "gtest/gtest.h"
#include <framework/WeatherTruthByDistanceToGo.h>

using namespace aaesim::test::utils;

namespace aaesim {
namespace test {
namespace open_source {

TEST(WeatherTruthByDistanceToGo, SetWeatherFromDtg) {
   std::shared_ptr<WeatherTruthByDistanceToGo> test_weather = (new WeatherTruthByDistanceToGo)->GetSharedPtr();
   test_weather->LoadEnvFile("resources/test_env_file.csv");
   test_weather->SetWeatherFromDtg(Units::MetersLength(900.0));

   Units::KelvinTemperature actual_temp = test_weather->GetTemperature(Units::negInfinity());
   const Units::KelvinTemperature expected_temp(226.0);
   EXPECT_NEAR(actual_temp.value(), expected_temp.value(), 1e-10);

   //   actual_temp = test_weather->InterpolateTemperature(Units::infinity(), Units::infinity(), Units::infinity());
   //   EXPECT_NEAR(actual_temp.value(), expected_temp.value(), 1e-10);
}

TEST(WeatherTruthByTime, SetWeatherFromTime) {
   std::shared_ptr<WeatherTruthByTime> test_weather = (new WeatherTruthByTime)->GetSharedPtr();
   test_weather->LoadEnvFile("resources/test_env_file.csv");
   test_weather->SetWeatherFromTime(Units::SecondsTime(1.0));

   Units::KelvinTemperature actual_temp = test_weather->GetTemperature(Units::zero());
   const Units::KelvinTemperature expected_temp(226.0);
   EXPECT_NEAR(actual_temp.value(), expected_temp.value(), 1e-10);

   //   actual_temp = test_weather->InterpolateTemperature(Units::zero(), Units::negInfinity(), Units::infinity());
   //   EXPECT_NEAR(actual_temp.value(), expected_temp.value(), 1e-10);
}
}  // namespace open_source
}  // namespace test
}  // namespace aaesim