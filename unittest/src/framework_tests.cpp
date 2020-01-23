// ****************************************************************************
// NOTICE
//
// This is the copyright work of The MITRE Corporation, and was produced
// for the U. S. Government under Contract Number DTFAWA-10-C-00080, and
// is subject to Federal Aviation Administration Acquisition Management
// System Clause 3.5-13, Rights In Data-General, Alt. III and Alt. IV
// (Oct. 1996).  No other use other than that granted to the U. S.
// Government, or to those acting on behalf of the U. S. Government,
// under that Clause is authorized without the express written
// permission of The MITRE Corporation. For further information, please
// contact The MITRE Corporation, Contracts Office, 7515 Colshire Drive,
// McLean, VA  22102-7539, (703) 983-6000. 
//
// Copyright 2020 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

/**
 * Tests here are for code in the AircraftDynamicsTestFramework source folder only.
 */

#include <framework/WeatherTruthByTime.h>
#include "gtest/gtest.h"
#include "utils/WindSpeedUtils.h"
#include "utils/PreparatorsUtils.h"
#include <framework/WeatherTruthByDistanceToGo.h>

using namespace aaesim::test::utils;

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
