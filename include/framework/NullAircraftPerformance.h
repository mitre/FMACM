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

#include "public/FixedMassAircraftPerformance.h"

#include <stdexcept>

namespace fmacm {
class NullAircraftPerformance final : public aaesim::open_source::FixedMassAircraftPerformance {
  public:
   NullAircraftPerformance() = default;
   ~NullAircraftPerformance() = default;
   void GetDragCoefficients(const Units::Speed &calibrated_airspeed, const Units::Length &altitude_msl,
                            const aaesim::open_source::bada_utils::FlapConfiguration &current_flap_configuration,
                            double &cd0, double &cd2, double &gear,
                            aaesim::open_source::bada_utils::FlapConfiguration &flap_configuration) const override {
      throw std::runtime_error("intentionally unimplemented code: please implement BADA v3.7");
   }

   void GetDragCoefficientsAndIncrementFlapConfiguration(
         const Units::Speed &calibrated_airspeed, const Units::Length &altitude_msl, double &cd0, double &cd2,
         double &gear, aaesim::open_source::bada_utils::FlapConfiguration &updated_flap_setting) override {
      throw std::runtime_error("intentionally unimplemented code: please implement BADA v3.7");
   }

   void GetConfigurationForIncreasedDrag(
         const Units::Speed &calibrated_airspeed, const Units::Length &altitude_msl,
         aaesim::open_source::bada_utils::FlapConfiguration &updated_flap_setting) override {
      throw std::runtime_error("intentionally unimplemented code: please implement BADA v3.7");
   }

   Units::NewtonsForce GetMaxThrust(const Units::Length &altitude_msl,
                                    aaesim::open_source::bada_utils::FlapConfiguration flap_configuration,
                                    aaesim::open_source::bada_utils::EngineThrustMode engine_thrust_mode,
                                    Units::AbsCelsiusTemperature temperature_offset) const override {
      throw std::runtime_error("intentionally unimplemented code: please implement BADA v3.7");
   }

   void GetCoefficientsForFlapConfiguration(aaesim::open_source::bada_utils::FlapConfiguration flap_configuration,
                                            double &cd0, double &cd2, double &gear) const override {
      throw std::runtime_error("intentionally unimplemented code: please implement BADA v3.7");
   }

   aaesim::open_source::bada_utils::FlapConfiguration GetFlapConfigurationForState(
         const Units::Speed &calibrated_airspeed, const Units::Length &altitude_msl,
         const aaesim::open_source::bada_utils::FlapConfiguration &current_flap_configuration) const override {
      throw std::runtime_error("intentionally unimplemented code: please implement BADA v3.7");
   }

   Units::Mass GetAircraftMass() const override {
      throw std::runtime_error("intentionally unimplemented code: please implement BADA v3.7");
   }

   double GetAircraftMassPercentile() const override {
      throw std::runtime_error("intentionally unimplemented code: please implement BADA v3.7");
   }

   aaesim::open_source::bada_utils::FlapSpeeds GetFlapSpeeds() const override {
      throw std::runtime_error("intentionally unimplemented code: please implement BADA v3.7");
   }

   aaesim::open_source::bada_utils::FlapConfiguration GetCurrentFlapConfiguration() const override {
      throw std::runtime_error("intentionally unimplemented code: please implement BADA v3.7");
   }

   void UpdateMassFraction(BoundedValue<double, 0, 1> mass_fraction) override {
      throw std::runtime_error("intentionally unimplemented code: please implement BADA v3.7");
   }

   aaesim::open_source::bada_utils::AircraftType GetAircraftTypeInformation() const override {
      throw std::runtime_error("intentionally unimplemented code: please implement BADA v3.7");
   }

   aaesim::open_source::bada_utils::Mass GetAircraftMassInformation() const override {
      throw std::runtime_error("intentionally unimplemented code: please implement BADA v3.7");
   }

   aaesim::open_source::bada_utils::FlightEnvelope GetFlightEnvelopeInformation() const override {
      throw std::runtime_error("intentionally unimplemented code: please implement BADA v3.7");
   }

   aaesim::open_source::bada_utils::Aerodynamics GetAerodynamicsInformation() const override {
      throw std::runtime_error("intentionally unimplemented code: please implement BADA v3.7");
   }

   aaesim::open_source::bada_utils::EngineThrust GetEngineThrustInformation() const override {
      throw std::runtime_error("intentionally unimplemented code: please implement BADA v3.7");
   }

   aaesim::open_source::bada_utils::FuelFlow GetFuelFlowInformation() const override {
      throw std::runtime_error("intentionally unimplemented code: please implement BADA v3.7");
   }

   aaesim::open_source::bada_utils::GroundMovement GetGroundMovementInformation() const override {
      throw std::runtime_error("intentionally unimplemented code: please implement BADA v3.7");
   }

   aaesim::open_source::bada_utils::Procedure GetProcedureInformation(unsigned int index) const override {
      throw std::runtime_error("intentionally unimplemented code: please implement BADA v3.7");
   }

   aaesim::open_source::bada_utils::AircraftPerformance GetAircraftPerformanceInformation() const override {
      throw std::runtime_error("intentionally unimplemented code: please implement BADA v3.7");
   }

   std::string GetAircraftTypeIdentifier() const override {
      throw std::runtime_error("intentionally unimplemented code: please implement BADA v3.7");
   }
};
}  // namespace fmacm
