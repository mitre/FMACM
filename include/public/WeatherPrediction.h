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
// Copyright 2018 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#pragma once

#include <memory>
#include "public/WeatherEstimate.h"

enum PredictedWindOption
{
   SINGLE_DTG = 0,
   MULTIPLE_DTG_LEGACY = 1,
   MULTIPLE_DTG_ALONG_ROUTE = 2
};

class WeatherPrediction : public WeatherEstimate
{
   friend class Wind_populate_predicted_wind_matrices_Test;

   friend class TrajectoryPredictor_updateWeatherPrediction_Test;

   friend class TrajectoryPredictor_startAltitudeInDescentAltList_Test;

   friend class TrajectoryPredictor_startAndEndAltitudeInDescentAltList_Test;

public:
   static const PredictedWindOption PWOValues[];

   WeatherPrediction();

   WeatherPrediction(PredictedWindOption option,
                     std::shared_ptr<Wind> wind,
                     std::shared_ptr<Atmosphere> atmosphere);

   virtual ~WeatherPrediction();

   PredictedWindOption getPredictedWindOption() const;

   // for backward compatibility
   std::shared_ptr<Wind> getForecastWind() const;

   std::shared_ptr<Atmosphere> getForecastAtmosphere() const;

   void dump();

   int incrementUpdateCount();

   int getUpdateCount() const;

private:
   int updateCount;
   PredictedWindOption mPredictedWindOption;
};

