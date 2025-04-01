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

#include "public/LoggingLoadable.h"

#include "aaesim/KineticPredictionEntity.h"
#include "aaesim/ClimbPredictorLoader.h"
#include "aaesim/DescentPredictorLoader.h"
#include "public/AircraftIntent.h"
#include "aaesim/WindAltitudes.h"
#include "aaesim/ScenarioConfiguration.h"
#include "avionics/Wgs84KineticUnionPredictor.h"

namespace aaesim {
namespace loaders {
class KineticPredictionEntityLoader final : public LoggingLoadable {
  public:
   KineticPredictionEntityLoader()
      : m_is_loaded(false),
        m_kinetic_descent_loader(),
        m_kinetic_climb_loader(),
        m_aircraft_intent_loader(),
        m_ac_type(),
        m_predicted_wind_altitude_loader(),
        m_predicted_temp_offset(INT32_MIN),
        m_aircraft_id() {}

   ~KineticPredictionEntityLoader() = default;
   bool load(DecodedStream *input) override;
   std::shared_ptr<aaesim::KineticPredictionEntity> BuildKineticPredictorEntity(
         const aaesim::ScenarioConfiguration &configuration);
   bool IsLoaded() const { return m_is_loaded; }

  private:
   static log4cplus::Logger m_logger;

   void RegisterLoadableVariables();
   std::shared_ptr<aaesim::Wgs84KineticUnionPredictor> BuildKineticPredictor();

   aaesim::open_source::WeatherPrediction BuildPredictedWeatherForAvionics(
         const std::string &forecast_wind_filename,
         const aaesim::open_source::PredictedWindOption &predicted_wind_option);

   bool m_is_loaded;
   aaesim::loaders::DescentPredictorLoader m_kinetic_descent_loader;
   aaesim::loaders::ClimbPredictorLoader m_kinetic_climb_loader;
   AircraftIntent m_aircraft_intent_loader;
   std::string m_ac_type;
   WindAltitudes m_predicted_wind_altitude_loader;
   double m_predicted_temp_offset;
   std::string m_aircraft_id;
};
}  // namespace loaders
}  // namespace aaesim