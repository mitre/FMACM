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

#pragma once

#include "public/BadaUtils.h"
#include "public/EquationsOfMotionState.h"
#include "public/FixedMassAircraftPerformance.h"
#include "public/Guidance.h"
#include "public/TrueWeatherOperator.h"
#include "scalar/Angle.h"
#include "scalar/Force.h"
#include "scalar/Speed.h"
#include "utility/BoundedValue.h"

namespace aaesim::open_source {
struct VerticalController {
   virtual void Initialize(
         std::shared_ptr<aaesim::open_source::FixedMassAircraftPerformance> &performance_calculator) = 0;
   virtual void ComputeVerticalCommands(const Guidance &guidance,
                                        const EquationsOfMotionState &equations_of_motion_state,
                                        std::shared_ptr<const aaesim::open_source::TrueWeatherOperator> &sensed_weather,
                                        Units::Force &thrust_command, Units::Angle &gamma_command,
                                        Units::Speed &tas_command, BoundedValue<double, 0, 1> &speed_brake_command,
                                        aaesim::open_source::bada_utils::FlapConfiguration &flap_command) = 0;

   virtual Units::Frequency GetGammaGain() const { return gain_flight_path_angle_; };
   virtual Units::Frequency GetThrustGain() const { return thrust_gain_; };
   virtual double GetSpeedBrakeGain() const = 0;

  protected:
   inline static const Units::Frequency natural_frequency_{Units::HertzFrequency(0.20)};
   inline static Units::Frequency CalculateThrustGain() {
      const double zeta = 0.88;
      return 2 * zeta * natural_frequency_;
   }
   inline static const Units::Frequency gain_flight_path_angle_{Units::HertzFrequency(0.40)};
   inline static const Units::Frequency thrust_gain_{CalculateThrustGain()};
   inline static void ConfigureFlapsAndEstimateKineticForces(
         const EquationsOfMotionState &equations_of_motion_state,
         std::shared_ptr<const aaesim::open_source::TrueWeatherOperator> &sensed_weather,
         std::shared_ptr<FixedMassAircraftPerformance> aircraft_performance, Units::Force &lift, Units::Force &drag,
         aaesim::open_source::bada_utils::FlapConfiguration &flap_configuration) {
      Units::Speed calibrated_airspeed = sensed_weather->GetTrueWeather()->TAS2CAS(
            Units::MetersPerSecondSpeed(equations_of_motion_state.true_airspeed),
            Units::MetersLength(equations_of_motion_state.altitude_msl));

      Units::KilogramsMeterDensity rho{};
      Units::Pressure pressure{};
      sensed_weather->GetTrueWeather()->getAtmosphere()->AirDensity(equations_of_motion_state.altitude_msl, rho,
                                                                    pressure);

      double cd0{0}, cd2{0}, gear{0};
      aircraft_performance->GetDragCoefficientsAndIncrementFlapConfiguration(
            calibrated_airspeed, equations_of_motion_state.altitude_msl, cd0, cd2, gear, flap_configuration);

      const auto ac_mass = aircraft_performance->GetAircraftMass();
      const auto wing_area = aircraft_performance->GetAerodynamicsInformation().S;
      double cL =
            (2. * ac_mass * Units::ONE_G_ACCELERATION) / (rho * Units::sqr(equations_of_motion_state.true_airspeed) *
                                                          wing_area * cos(equations_of_motion_state.phi));
      double cD = cd0 + gear + cd2 * pow(cL, 2);
      if (equations_of_motion_state.speed_brake_percentage != 0.0) {
         cD = (1.0 + 0.6 * equations_of_motion_state.speed_brake_percentage) * cD;
      }
      drag = 1. / 2. * rho * cD * Units::sqr(equations_of_motion_state.true_airspeed) * wing_area;
      lift = 1. / 2. * rho * cL * Units::sqr(equations_of_motion_state.true_airspeed) * wing_area;
   };
};

class NullVerticalController final : public VerticalController {
  public:
   NullVerticalController() = default;
   ~NullVerticalController() = default;
   void Initialize(
         std::shared_ptr<aaesim::open_source::FixedMassAircraftPerformance> &performance_calculator) override {}
   void ComputeVerticalCommands(const Guidance &guidance, const EquationsOfMotionState &equations_of_motion_state,
                                std::shared_ptr<const aaesim::open_source::TrueWeatherOperator> &sensed_weather,
                                Units::Force &thrust_command, Units::Angle &gamma_command, Units::Speed &tas_command,
                                BoundedValue<double, 0, 1> &speed_brake_command,
                                aaesim::open_source::bada_utils::FlapConfiguration &flap_command) override {
      thrust_command = equations_of_motion_state.thrust;
      gamma_command = equations_of_motion_state.gamma;
      tas_command = equations_of_motion_state.true_airspeed;
      speed_brake_command = equations_of_motion_state.speed_brake_percentage;
      flap_command = equations_of_motion_state.flap_configuration;
   }
   double GetSpeedBrakeGain() const override { return 0.0; };
};

}  // namespace aaesim::open_source