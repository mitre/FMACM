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

#include "public/ScenarioEntity.h"

#include "public/SimulationTime.h"
#include "public/AircraftIntent.h"
#include "public/WeatherPrediction.h"
#include "avionics/Wgs84KineticUnionPredictor.h"

namespace aaesim {
class KineticPredictionEntity final : public open_source::ScenarioEntity {

  public:
   bool Update(const aaesim::open_source::SimulationTime &simulation_time) override;
   const int GetStartTime() const override;
   bool IsFinished() const override;
   std::shared_ptr<const aaesim::Wgs84KineticUnionPredictor> GetKineticPredictor() const { return m_kinetic_predictor; }
   const std::string &GetAircraftId() const { return m_aircraft_id; }
   const AircraftIntent &GetAircraftIntent() const { return m_aircraft_intent; }

   class Builder {
     public:
      Builder(const std::string &aircraft_id, const std::string &bada_performance_name,
              const AircraftIntent &aircraft_intent, const aaesim::open_source::WeatherPrediction &weather_prediction,
              std::shared_ptr<aaesim::Wgs84KineticUnionPredictor> &kinetic_predictor)
         : aircraft_id_(aircraft_id),
           bada_performance_name_(bada_performance_name),
           aircraft_intent_(aircraft_intent),
           weather_prediction_(weather_prediction),
           kinetic_predictor_(kinetic_predictor){};
      ~Builder() = default;
      std::shared_ptr<aaesim::KineticPredictionEntity> Build() const;

      std::string GetBadaAircraftPerformancName() const { return bada_performance_name_; }
      AircraftIntent GetAircraftIntent() const { return aircraft_intent_; }
      aaesim::open_source::WeatherPrediction GetWeatherPrediction() const { return weather_prediction_; }
      std::shared_ptr<aaesim::Wgs84KineticUnionPredictor> GetKineticPredictor() const { return kinetic_predictor_; }
      std::string GetAircraftId() const { return aircraft_id_; }

     private:
      std::string aircraft_id_;
      std::string bada_performance_name_;
      AircraftIntent aircraft_intent_;
      aaesim::open_source::WeatherPrediction weather_prediction_;
      std::shared_ptr<aaesim::Wgs84KineticUnionPredictor> kinetic_predictor_;
   };

   KineticPredictionEntity(const KineticPredictionEntity::Builder &builder);

  private:
   KineticPredictionEntity()
      : m_kinetic_predictor(),
        m_has_run_once(false),
        m_bada_performance(),
        m_aircraft_intent(),
        m_weather_prediction(),
        m_aircraft_id(){};
   std::shared_ptr<aaesim::Wgs84KineticUnionPredictor> m_kinetic_predictor;
   bool m_has_run_once;
   std::string m_bada_performance;
   AircraftIntent m_aircraft_intent;
   aaesim::open_source::WeatherPrediction m_weather_prediction;
   std::string m_aircraft_id;
};
}  // namespace aaesim