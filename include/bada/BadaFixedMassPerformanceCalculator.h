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

#include <memory>
#include "public/FixedMassAircraftPerformance.h"
#include "bada/BadaPerformanceData.h"
#include "bada/BadaPerformanceInitialConditions.h"
#include "bada/FlapProgressionCalculator.h"
#include "bada/FlapState.h"
#include "public/Logging.h"

namespace aaesim {
namespace bada {

class BadaFixedMassPerformanceCalculator final : public aaesim::open_source::FixedMassAircraftPerformance {
  public:
   inline static const Units::Length FAF_ALTITUDE_MAXIMUM{Units::FeetLength(15000.0)};

   BadaFixedMassPerformanceCalculator(std::shared_ptr<const BadaPerformanceData> bada_data,
                                      const BadaPerformanceInitialConditions &initial_conditions,
                                      const aaesim::open_source::bada_utils::FlapSpeeds &flap_speeds,
                                      std::shared_ptr<const FlapProgressionCalculator> flap_progression_calculator);
   ~BadaFixedMassPerformanceCalculator() = default;

   void GetDragCoefficients(const Units::Speed &calibrated_airspeed, const Units::Length &altitude_msl,
                            const aaesim::open_source::bada_utils::FlapConfiguration &current_flap_configuration,
                            double &cd0, double &cd2, double &gear,
                            aaesim::open_source::bada_utils::FlapConfiguration &flap_configuration) const override;

   void GetDragCoefficientsAndIncrementFlapConfiguration(
         const Units::Speed &calibrated_airspeed, const Units::Length &altitude_msl, double &cd0, double &cd2,
         double &gear, aaesim::open_source::bada_utils::FlapConfiguration &updated_flap_setting) override;

   void GetConfigurationForIncreasedDrag(
         const Units::Speed &calibrated_airspeed, const Units::Length &altitude_msl,
         aaesim::open_source::bada_utils::FlapConfiguration &updated_flap_setting) override;

   Units::NewtonsForce GetMaxThrust(const Units::Length &altitude_msl,
                                    aaesim::open_source::bada_utils::FlapConfiguration flap_configuration,
                                    aaesim::open_source::bada_utils::EngineThrustMode engine_thrust_mode,
                                    Units::AbsCelsiusTemperature temperature_offset) const override;

   void GetCoefficientsForFlapConfiguration(open_source::bada_utils::FlapConfiguration flap_configuration, double &cd0,
                                            double &cd2, double &gear) const override;

   void GetCurrentDragCoefficients(double &cd0, double &cd2, double &gear) const override;

   aaesim::open_source::bada_utils::FlapConfiguration GetFlapConfigurationForState(
         const Units::Speed &calibrated_airspeed, const Units::Length &altitude_msl,
         const aaesim::open_source::bada_utils::FlapConfiguration &current_flap_configuration) const override;

   Units::Mass GetAircraftMass() const override;

   double GetAircraftMassPercentile() const override;

   open_source::bada_utils::FlapSpeeds GetFlapSpeeds() const override;

   open_source::bada_utils::FlapConfiguration GetCurrentFlapConfiguration() const override;

   void UpdateMassFraction(BoundedValue<double, 0, 1> mass_fraction) override;

   aaesim::open_source::bada_utils::AircraftType GetAircraftTypeInformation() const override {
      return bada_data_->GetAircraftTypeInformation();
   }

   aaesim::open_source::bada_utils::Mass GetAircraftMassInformation() const override {
      return bada_data_->GetAircraftMassInformation();
   }

   aaesim::open_source::bada_utils::FlightEnvelope GetFlightEnvelopeInformation() const override {
      return bada_data_->GetFlightEnvelopeInformation();
   }

   aaesim::open_source::bada_utils::Aerodynamics GetAerodynamicsInformation() const override {
      return bada_data_->GetAerodynamicsInformation();
   }

   aaesim::open_source::bada_utils::EngineThrust GetEngineThrustInformation() const override {
      return bada_data_->GetEngineThrustInformation();
   }

   aaesim::open_source::bada_utils::FuelFlow GetFuelFlowInformation() const override {
      return bada_data_->GetFuelFlowInformation();
   }

   aaesim::open_source::bada_utils::GroundMovement GetGroundMovementInformation() const override {
      return bada_data_->GetGroundMovementInformation();
   }

   aaesim::open_source::bada_utils::Procedure GetProcedureInformation(unsigned int index) const override {
      return bada_data_->GetProcedureInformation(index);
   }

   aaesim::open_source::bada_utils::AircraftPerformance GetAircraftPerformanceInformation() const override {
      return bada_data_->GetAircraftPerformanceInformation();
   }

   std::string GetAircraftTypeIdentifier() const override { return bada_data_->GetBadaAircraftTypeCode(); }

   // Visible for testing
   Units::Length GetAltitudeAtFaf() const;

  private:
   static log4cplus::Logger logger_;
   BadaFixedMassPerformanceCalculator() = default;
   bool CloseToFinalApproachFixAltitude(const Units::Length &altitude_msl) const;

   FlapState flapState_;
   std::shared_ptr<const BadaPerformanceData> bada_data_;
   std::shared_ptr<const FlapProgressionCalculator> flap_progression_calculator_;
   open_source::bada_utils::FlapConfiguration flap_configuration_;
   open_source::bada_utils::FlapSpeeds flap_speeds_;
   Units::Mass aircraft_mass_;
   double mass_percentile_;
   Units::Length faf_altitude_msl_;
};
}  // namespace bada
}  // namespace aaesim

inline Units::Mass aaesim::bada::BadaFixedMassPerformanceCalculator::GetAircraftMass() const { return aircraft_mass_; }

inline double aaesim::bada::BadaFixedMassPerformanceCalculator::GetAircraftMassPercentile() const {
   return mass_percentile_;
}

inline aaesim::open_source::bada_utils::FlapSpeeds aaesim::bada::BadaFixedMassPerformanceCalculator::GetFlapSpeeds()
      const {
   return flap_speeds_;
}

inline aaesim::open_source::bada_utils::FlapConfiguration
      aaesim::bada::BadaFixedMassPerformanceCalculator::GetCurrentFlapConfiguration() const {
   return flap_configuration_;
}

inline Units::Length aaesim::bada::BadaFixedMassPerformanceCalculator::GetAltitudeAtFaf() const {
   return faf_altitude_msl_;
}

inline void aaesim::bada::BadaFixedMassPerformanceCalculator::UpdateMassFraction(
      BoundedValue<double, 0, 1> mass_fraction) {
   mass_percentile_ = mass_fraction;
   aircraft_mass_ = (bada_data_->GetAircraftMassInformation().m_max - bada_data_->GetAircraftMassInformation().m_min) *
                          mass_percentile_ +
                    bada_data_->GetAircraftMassInformation().m_min;
}
