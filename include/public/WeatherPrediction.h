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
#include "public/WeatherEstimate.h"

namespace aaesim {
namespace test {
class Wind_populate_predicted_wind_matrices_Test;
class TrajectoryPredictor_updateWeatherPrediction_Test;
class TrajectoryPredictor_startAltitudeInDescentAltList_Test;
class TrajectoryPredictor_startAndEndAltitudeInDescentAltList_Test;
}  // namespace test

namespace open_source {

enum PredictedWindOption { SINGLE_DTG = 0, MULTIPLE_DTG_ALONG_ROUTE = 1 };

class WeatherPrediction final : public WeatherEstimate {
   friend class aaesim::test::Wind_populate_predicted_wind_matrices_Test;

   friend class aaesim::test::TrajectoryPredictor_updateWeatherPrediction_Test;

   friend class aaesim::test::TrajectoryPredictor_startAltitudeInDescentAltList_Test;

   friend class aaesim::test::TrajectoryPredictor_startAndEndAltitudeInDescentAltList_Test;

  public:
   inline static const PredictedWindOption PWOValues[2] = {SINGLE_DTG, MULTIPLE_DTG_ALONG_ROUTE};
   static aaesim::open_source::WeatherPrediction CreateZeroWindPrediction(std::shared_ptr<Atmosphere> atmosphere);

   WeatherPrediction() = default;
   WeatherPrediction(PredictedWindOption option, std::shared_ptr<Wind> wind, std::shared_ptr<Atmosphere> atmosphere);
   virtual ~WeatherPrediction() = default;

   PredictedWindOption GetPredictedWindOption() const;

   std::shared_ptr<Wind> GetForecastWind() const;

   std::shared_ptr<Atmosphere> GetForecastAtmosphere() const;

   int IncrementUpdateCount();

   int GetUpdateCount() const;

  private:
   PredictedWindOption m_predicted_wind_option{SINGLE_DTG};
   int m_update_count{0};
};

inline PredictedWindOption WeatherPrediction::GetPredictedWindOption() const { return m_predicted_wind_option; }

inline int WeatherPrediction::IncrementUpdateCount() {
   m_update_count++;
   return m_update_count;
}

inline int WeatherPrediction::GetUpdateCount() const { return m_update_count; }

inline std::shared_ptr<Wind> WeatherPrediction::GetForecastWind() const { return getWind(); }

inline std::shared_ptr<Atmosphere> WeatherPrediction::GetForecastAtmosphere() const { return getAtmosphere(); }
}  // namespace open_source
}  // namespace aaesim
