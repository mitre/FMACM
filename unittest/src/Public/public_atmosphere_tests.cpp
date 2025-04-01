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

#include "public/USStandardAtmosphere1976.h"

using namespace std;
using namespace aaesim::open_source;

namespace aaesim {
namespace test {

TEST(USStandardAtmosphere1976, temperature) {
   Atmosphere *atm = new USStandardAtmosphere1976();
   // check for discontinuity at tropopause
   Units::MetersLength half_eps(1e-6);
   Units::KelvinTemperature delta(1e-3);
   Units::KelvinTemperature temp_pre_tropo = atm->GetTemperature(atm->GetTropopauseHeight() - half_eps);
   Units::KelvinTemperature temp_post_tropo = atm->GetTemperature(atm->GetTropopauseHeight() + half_eps);
   EXPECT_NEAR(temp_pre_tropo.value(), temp_post_tropo.value(), delta.value());

   // check for temperature below absolute zero
   Units::KelvinTemperature temp_very_high_alt = atm->GetTemperature(Units::MetersLength(1e8));
   EXPECT_GE(temp_very_high_alt.value(), 0);
   EXPECT_EQ(Units::CelsiusTemperature(0), static_cast<USStandardAtmosphere1976 *>(atm)->GetTemperatureOffset());
   delete atm;
}

TEST(USStandardAtmosphere1976, density_low_altitude) {
   Atmosphere *atm = new USStandardAtmosphere1976();

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

TEST(USStandardAtmosphere1976, density_above_htrop) {
   Atmosphere *atm = new USStandardAtmosphere1976();

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

TEST(USStandardAtmosphere1976, speed) {
   Atmosphere *atm = new USStandardAtmosphere1976();

   const Units::Length alt = Units::MetersLength(3000.0);
   Units::Speed cas = Units::MetersPerSecondSpeed(400.0);
   Units::Speed tas = atm->CAS2TAS(cas, alt);
   EXPECT_NEAR(Units::MetersPerSecondSpeed(tas).value(), 464.32, .02);
   Units::Speed cas2 = atm->TAS2CAS(tas, alt);
   EXPECT_DOUBLE_EQ(Units::MetersPerSecondSpeed(cas).value(), Units::MetersPerSecondSpeed(cas2).value());
   delete atm;
}

TEST(USStandardAtmosphere1976, mach_ias_transition_isa) {
   // Test mach/ias conversion at altitude using mach of unity.
   const std::vector<double> test_machs = {.6, .65, .713, .868};
   const std::vector<Units::Length> test_altitudes = {Units::FeetLength(27800.0), Units::FeetLength(30000.0),
                                                      Units::FeetLength(35000.0)};
   const Units::FeetLength tolerance(50.0);

   for (double test_mach : test_machs) {
      for (Units::FeetLength test_altitude : test_altitudes) {
         // zero temp offset
         const USStandardAtmosphere1976 atmosphere_0;
         Units::Speed ias_at_test_mach = atmosphere_0.MachToIAS(test_mach, test_altitude);
         Units::FeetLength actual_alt_trans_sea_level_old =
               atmosphere_0.GetMachIASTransition(ias_at_test_mach, test_mach);
         Units::FeetLength actual_alt_trans_sea_level = atmosphere_0.GetMachIASTransition(ias_at_test_mach, test_mach);
         EXPECT_NEAR(test_altitude.value(), actual_alt_trans_sea_level_old.value(), tolerance.value());
         EXPECT_NEAR(test_altitude.value(), actual_alt_trans_sea_level.value(), tolerance.value());
      }
   }
}

}  // namespace test
}  // namespace aaesim
