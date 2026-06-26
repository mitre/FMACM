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
// (c) 2026 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>

#include "public/AircraftControl.h"
#include "public/FixedMassAircraftPerformance.h"
#include "public/NullPositionEstimator.h"
#include "public/SimulationTime.h"
#include "public/ThreeDOFDynamics.h"
#include "public/USStandardAtmosphere1976.h"
#include "public/WeatherTruth.h"
#include "public/WindZero.h"
#include "public/ZeroWindTrueWeatherOperator.h"

using namespace aaesim::open_source;

namespace aaesim {
namespace test {
namespace {

class SimulationTimeStepGuard {
  public:
   explicit SimulationTimeStepGuard(Units::Time time_step)
      : original_time_step_(SimulationTime::GetSimulationTimeStep()) {
      SimulationTime::SetSimulationTimeStep(time_step);
   }
   ~SimulationTimeStepGuard() { SimulationTime::SetSimulationTimeStep(original_time_step_); }

  private:
   Units::SecondsTime original_time_step_;
};

class IdealGliderPerformance final : public FixedMassAircraftPerformance {
  public:
   void GetDragCoefficients(const Units::Speed &calibrated_airspeed, const Units::Length &altitude_msl,
                            const bada_utils::FlapConfiguration &current_flap_configuration, double &cd0, double &cd2,
                            double &gear, bada_utils::FlapConfiguration &flap_configuration) const override {
      SetZeroDrag(cd0, cd2, gear);
      flap_configuration = current_flap_configuration;
   }

   void GetDragCoefficientsAndIncrementFlapConfiguration(const Units::Speed &calibrated_airspeed,
                                                         const Units::Length &altitude_msl, double &cd0, double &cd2,
                                                         double &gear,
                                                         bada_utils::FlapConfiguration &updated_flap_setting) override {
      SetZeroDrag(cd0, cd2, gear);
      updated_flap_setting = bada_utils::FlapConfiguration::CRUISE;
   }

   void GetCurrentDragCoefficients(double &cd0, double &cd2, double &gear) const override {
      SetZeroDrag(cd0, cd2, gear);
   }

   void GetConfigurationForIncreasedDrag(const Units::Speed &calibrated_airspeed, const Units::Length &altitude_msl,
                                         bada_utils::FlapConfiguration &updated_flap_setting) override {
      updated_flap_setting = bada_utils::FlapConfiguration::CRUISE;
   }

   Units::NewtonsForce GetMaxThrust(const Units::Length &altitude_msl, bada_utils::FlapConfiguration flap_configuration,
                                    bada_utils::EngineThrustMode engine_thrust_mode,
                                    Units::AbsCelsiusTemperature temperature_offset) const override {
      return Units::NewtonsForce(0);
   }

   void GetCoefficientsForFlapConfiguration(bada_utils::FlapConfiguration flap_configuration, double &cd0, double &cd2,
                                            double &gear) const override {
      SetZeroDrag(cd0, cd2, gear);
   }

   bada_utils::FlapConfiguration GetFlapConfigurationForState(
         const Units::Speed &calibrated_airspeed, const Units::Length &altitude_msl,
         const bada_utils::FlapConfiguration &current_flap_configuration) const override {
      return current_flap_configuration;
   }

   Units::Mass GetAircraftMass() const override { return Units::KilogramsMass(50000.0); }

   double GetAircraftMassPercentile() const override { return 0.5; }

   bada_utils::FlapSpeeds GetFlapSpeeds() const override { return bada_utils::FlapSpeeds{}; }

   bada_utils::FlapConfiguration GetCurrentFlapConfiguration() const override {
      return bada_utils::FlapConfiguration::CRUISE;
   }

   void UpdateMassFraction(BoundedValue<double, 0, 1> mass_fraction) override {}

   bada_utils::AircraftType GetAircraftTypeInformation() const override { return bada_utils::AircraftType{}; }

   bada_utils::Mass GetAircraftMassInformation() const override { return bada_utils::Mass{}; }

   bada_utils::FlightEnvelope GetFlightEnvelopeInformation() const override { return bada_utils::FlightEnvelope{}; }

   bada_utils::Aerodynamics GetAerodynamicsInformation() const override {
      bada_utils::Aerodynamics aerodynamics{};
      aerodynamics.S = Units::MetersArea(100.0);
      aerodynamics.cruise.V_stall = Units::MetersPerSecondSpeed(1.0);
      return aerodynamics;
   }

   bada_utils::EngineThrust GetEngineThrustInformation() const override { return bada_utils::EngineThrust{}; }

   bada_utils::FuelFlow GetFuelFlowInformation() const override { return bada_utils::FuelFlow{}; }

   bada_utils::GroundMovement GetGroundMovementInformation() const override { return bada_utils::GroundMovement{}; }

   bada_utils::Procedure GetProcedureInformation(unsigned int index) const override { return bada_utils::Procedure{}; }

   bada_utils::AircraftPerformance GetAircraftPerformanceInformation() const override {
      return bada_utils::AircraftPerformance{};
   }

   std::string GetAircraftTypeIdentifier() const override { return "IDEAL_GLIDER"; }

  private:
   static void SetZeroDrag(double &cd0, double &cd2, double &gear) {
      cd0 = 0.0;
      cd2 = 0.0;
      gear = 0.0;
   }
};

std::shared_ptr<TrueWeatherOperator> MakeZeroWindStandardAtmosphereOperator() {
   auto atmosphere = std::make_shared<USStandardAtmosphere1976>();
   auto wind = std::make_shared<WindZero>(atmosphere);
   auto weather_truth = std::make_shared<WeatherTruth>(wind, atmosphere, true);
   return std::make_shared<ZeroWindTrueWeatherOperator>(weather_truth);
}

std::shared_ptr<AircraftControl> MakeNullAircraftControl(
      const std::shared_ptr<FixedMassAircraftPerformance> &aircraft_performance) {
   auto aircraft_control = AircraftControl::Builder()
                                 .WithCruiseDescentLateralController(std::make_shared<NoTurnLateralController>())
                                 .WithCruiseDescentVerticalController(std::make_shared<NullVerticalController>())
                                 .Build();
   aircraft_control->Initialize(aircraft_performance);
   return aircraft_control;
}

Guidance MakeCruiseDescentGuidance(Units::Speed true_airspeed, Units::Length altitude_msl,
                                   Units::SignedAngle track_enu) {
   Guidance guidance;
   guidance.m_active_guidance_phase = GuidanceFlightPhase::CRUISE_DESCENT;
   guidance.m_ground_speed = true_airspeed;
   guidance.m_ias_command = true_airspeed;
   guidance.m_reference_altitude = altitude_msl;
   guidance.m_enu_track_angle = track_enu;
   guidance.m_reference_bank_angle = Units::zero();
   guidance.m_vertical_speed = Units::zero();
   guidance.m_cross_track_error = Units::zero();
   guidance.m_use_cross_track = false;
   return guidance;
}

ThreeDOFDynamics MakeInitializedGlider(const std::shared_ptr<IdealGliderPerformance> &aircraft_performance,
                                       Units::Length initial_altitude_msl, Units::Speed initial_true_airspeed,
                                       Units::SignedAngle initial_track_enu,
                                       const EarthModel::LocalPositionEnu &initial_position_enu) {
   ThreeDOFDynamics dynamics;
   dynamics.Initialize(SimulationTime::Of(Units::ZERO_TIME), aircraft_performance,
                       EarthModel::GeodeticPosition::Of(Units::ZERO_ANGLE, Units::ZERO_ANGLE), initial_position_enu,
                       initial_altitude_msl, initial_true_airspeed, initial_track_enu, 0.5,
                       std::make_shared<NullPositionEstimator>(), MakeZeroWindStandardAtmosphereOperator());
   return dynamics;
}

AircraftState RunUpdates(ThreeDOFDynamics &dynamics, const Guidance &guidance,
                         const std::shared_ptr<AircraftControl> &aircraft_control, int update_count) {
   AircraftState state;
   for (int update_index = 1; update_index <= update_count; ++update_index) {
      state = dynamics.Update(42, SimulationTime::Of(Units::SecondsTime(update_index)), guidance, aircraft_control);
   }
   return state;
}

}  // namespace

TEST(ThreeDofGliderKinematics, level_eastbound_motion_matches_constant_velocity_solution) {
   const SimulationTimeStepGuard time_step_guard(Units::SecondsTime(1.0));
   const auto aircraft_performance = std::make_shared<IdealGliderPerformance>();
   const auto aircraft_control = MakeNullAircraftControl(aircraft_performance);

   const Units::MetersLength initial_x(1200.0);
   const Units::MetersLength initial_y(-300.0);
   const Units::MetersLength initial_altitude_msl(3000.0);
   const Units::MetersPerSecondSpeed initial_true_airspeed(210.0);
   const Units::SignedDegreesAngle initial_track_enu(0.0);
   const int update_count = 5;

   auto dynamics =
         MakeInitializedGlider(aircraft_performance, initial_altitude_msl, initial_true_airspeed, initial_track_enu,
                               EarthModel::LocalPositionEnu::Of(initial_x, initial_y, Units::zero()));

   const auto state =
         RunUpdates(dynamics, MakeCruiseDescentGuidance(initial_true_airspeed, initial_altitude_msl, initial_track_enu),
                    aircraft_control, update_count);

   const Units::SecondsTime elapsed_time(update_count * SimulationTime::GetSimulationTimeStep().value());
   const Units::MetersLength expected_x = initial_x + initial_true_airspeed * elapsed_time;

   EXPECT_NEAR(expected_x.value(), Units::MetersLength(state.GetPositionEnuX()).value(), 1e-9);
   EXPECT_NEAR(initial_y.value(), Units::MetersLength(state.GetPositionEnuY()).value(), 1e-9);
   EXPECT_NEAR(initial_altitude_msl.value(), Units::MetersLength(state.GetAltitudeMsl()).value(), 1e-9);
   EXPECT_NEAR(initial_true_airspeed.value(), Units::MetersPerSecondSpeed(state.GetTrueAirspeed()).value(), 1e-9);
   EXPECT_NEAR(initial_true_airspeed.value(), Units::MetersPerSecondSpeed(state.GetGroundSpeed()).value(), 1e-9);
   EXPECT_NEAR(0.0, Units::MetersPerSecondSpeed(state.GetVerticalSpeed()).value(), 1e-12);
   EXPECT_NEAR(
         0.0,
         Units::MetersSecondAcceleration(dynamics.GetEquationsOfMotionStateDerivative().true_airspeed_deriv).value(),
         1e-12);
   EXPECT_NEAR(0.0,
               Units::RadiansPerSecondAngularSpeed(dynamics.GetEquationsOfMotionStateDerivative().gamma_deriv).value(),
               1e-12);
   EXPECT_NEAR(
         0.0, Units::RadiansPerSecondAngularSpeed(dynamics.GetEquationsOfMotionStateDerivative().heading_deriv).value(),
         1e-12);
}

TEST(ThreeDofGliderKinematics, level_motion_resolves_three_four_five_heading_components) {
   const SimulationTimeStepGuard time_step_guard(Units::SecondsTime(1.0));
   const auto aircraft_performance = std::make_shared<IdealGliderPerformance>();
   const auto aircraft_control = MakeNullAircraftControl(aircraft_performance);

   const Units::MetersLength initial_x(-75.0);
   const Units::MetersLength initial_y(40.0);
   const Units::MetersLength initial_altitude_msl(1800.0);
   const Units::MetersPerSecondSpeed initial_true_airspeed(250.0);
   const Units::SignedRadiansAngle initial_track_enu(std::atan2(4.0, 3.0));
   const int update_count = 4;

   auto dynamics =
         MakeInitializedGlider(aircraft_performance, initial_altitude_msl, initial_true_airspeed, initial_track_enu,
                               EarthModel::LocalPositionEnu::Of(initial_x, initial_y, Units::zero()));

   const auto state =
         RunUpdates(dynamics, MakeCruiseDescentGuidance(initial_true_airspeed, initial_altitude_msl, initial_track_enu),
                    aircraft_control, update_count);

   const Units::SecondsTime elapsed_time(update_count * SimulationTime::GetSimulationTimeStep().value());
   const Units::MetersLength expected_x = initial_x + initial_true_airspeed * elapsed_time * 3.0 / 5.0;
   const Units::MetersLength expected_y = initial_y + initial_true_airspeed * elapsed_time * 4.0 / 5.0;

   EXPECT_NEAR(expected_x.value(), Units::MetersLength(state.GetPositionEnuX()).value(), 1e-9);
   EXPECT_NEAR(expected_y.value(), Units::MetersLength(state.GetPositionEnuY()).value(), 1e-9);
   EXPECT_NEAR(initial_altitude_msl.value(), Units::MetersLength(state.GetAltitudeMsl()).value(), 1e-9);
   EXPECT_NEAR(initial_true_airspeed.value(), Units::MetersPerSecondSpeed(state.GetTrueAirspeed()).value(), 1e-9);
   EXPECT_NEAR(initial_true_airspeed.value(), Units::MetersPerSecondSpeed(state.GetGroundSpeed()).value(), 1e-9);
   EXPECT_NEAR(Units::RadiansAngle(initial_track_enu).value(),
               Units::RadiansAngle(state.GetHeadingCcwFromEastRadians()).value(), 1e-12);
}

}  // namespace test
}  // namespace aaesim
