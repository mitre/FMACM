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

#include "public/AircraftIntent.h"
#include "public/WeatherPrediction.h"
#include "rtaalgs/DeltaSpeedAircraftIntent.h"
#include "avionics/Wgs84KineticDescentPredictor.h"

namespace required_time_of_arrival {
struct AircraftIntentMutator {
   virtual void ModifyAircraftIntent(const AircraftIntent &aircraft_intent,
                                     const aaesim::open_source::WeatherPrediction &weather_prediction) = 0;
   virtual AircraftIntent GetAircraftIntent() const = 0;
   virtual std::shared_ptr<aaesim::Wgs84KineticDescentPredictor> GetDescentPredictor() const = 0;
};

class DeltaIasMutator : public AircraftIntentMutator {
  public:
   DeltaIasMutator(std::shared_ptr<aaesim::Wgs84KineticDescentPredictor> predictor, const Units::Speed delta_ias)
      : kinetic_predictor_(predictor), delta_ias_(delta_ias) {}
   void ModifyAircraftIntent(const AircraftIntent &aircraft_intent,
                             const aaesim::open_source::WeatherPrediction &weather_prediction) override {
      kinetic_predictor_->GetVertPredictor()->SetIasAtEndOfRoute(
            kinetic_predictor_->GetVertPredictor()->GetIasAtEndOfRoute() + delta_ias_);
      kinetic_predictor_->GetVertPredictor()->SetTransitionIas(
            kinetic_predictor_->GetVertPredictor()->GetTransitionIas() + delta_ias_);
      const Units::Speed adjusted_cruise_ias =
            weather_prediction.MachToCAS(kinetic_predictor_->GetVertPredictor()->GetTransitionMach(),
                                         kinetic_predictor_->GetVertPredictor()->GetCruiseAltitude()) +
            delta_ias_;
      const double adjusted_cruise_mach = weather_prediction.CAS2Mach(
            adjusted_cruise_ias, kinetic_predictor_->GetVertPredictor()->GetCruiseAltitude());
      aircraft_intent_ = DeltaSpeedAircraftIntent(aircraft_intent, delta_ias_, adjusted_cruise_mach);
   }
   AircraftIntent GetAircraftIntent() const override { return aircraft_intent_; }
   std::shared_ptr<aaesim::Wgs84KineticDescentPredictor> GetDescentPredictor() const override {
      return kinetic_predictor_;
   }

  private:
   std::shared_ptr<aaesim::Wgs84KineticDescentPredictor> kinetic_predictor_;
   AircraftIntent aircraft_intent_;
   Units::Speed delta_ias_;
};

class DeltaMachMutator : public AircraftIntentMutator {
  public:
   DeltaMachMutator(std::shared_ptr<aaesim::Wgs84KineticDescentPredictor> predictor,
                    const BoundedValue<double, -1, 1> &delta_mach)
      : kinetic_predictor_(predictor), delta_mach_(delta_mach) {}
   void ModifyAircraftIntent(const AircraftIntent &aircraft_intent,
                             const aaesim::open_source::WeatherPrediction &weather_prediction) override {
      const double adjusted_cruise_mach = aircraft_intent.GetPlannedCruiseMach() + delta_mach_;
      Units::Length adjusted_transition_altitude = weather_prediction.GetForecastAtmosphere()->GetMachIASTransition(
            kinetic_predictor_->GetVertPredictor()->GetTransitionIas(), adjusted_cruise_mach);
      if (kinetic_predictor_->GetVertPredictor()->GetAltitudeAtEndOfRoute() >= adjusted_transition_altitude) {
         Units::Speed mach_as_ias = weather_prediction.MachToCAS(
               adjusted_cruise_mach, kinetic_predictor_->GetVertPredictor()->GetAltitudeAtEndOfRoute());
         kinetic_predictor_->GetVertPredictor()->SetIasAtEndOfRoute(mach_as_ias);
      }
      aircraft_intent_ = DeltaSpeedAircraftIntent(aircraft_intent, Units::ZERO_SPEED, adjusted_cruise_mach);
   }
   AircraftIntent GetAircraftIntent() const override { return aircraft_intent_; }
   std::shared_ptr<aaesim::Wgs84KineticDescentPredictor> GetDescentPredictor() const override {
      return kinetic_predictor_;
   }

  private:
   std::shared_ptr<aaesim::Wgs84KineticDescentPredictor> kinetic_predictor_;
   AircraftIntent aircraft_intent_;
   BoundedValue<double, -1, 1> delta_mach_;
};
}  // namespace required_time_of_arrival