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

#include "public/WeatherPrediction.h"
#include "public/StandardAtmosphere.h"

WeatherPrediction::WeatherPrediction()
      :
      WeatherEstimate(),
      updateCount(0) {
}

WeatherPrediction::WeatherPrediction(PredictedWindOption option,
                                     std::shared_ptr<Wind> wind,
                                     std::shared_ptr<Atmosphere> atmosphere)
      :
      WeatherEstimate(wind, atmosphere),
      updateCount(0),
      mPredictedWindOption(option) {
}

PredictedWindOption WeatherPrediction::getPredictedWindOption() const {
   return mPredictedWindOption;
}

WeatherPrediction::~WeatherPrediction() {
}

const PredictedWindOption WeatherPrediction::PWOValues[3] = {SINGLE_DTG,
                                                             MULTIPLE_DTG_LEGACY, MULTIPLE_DTG_ALONG_ROUTE};

std::shared_ptr<Wind> WeatherPrediction::getForecastWind() const {
   return getWind();
}

std::shared_ptr<Atmosphere> WeatherPrediction::getForecastAtmosphere() const {
   return getAtmosphere();
}

void WeatherPrediction::dump() {
   for (int iAlt = east_west.get_min_row();
        iAlt <= east_west.get_max_row(); iAlt++) {
      std::cout << iAlt << ":  " <<
                east_west.getAltitude(iAlt) << " " <<
                east_west.getSpeed(iAlt) << " " <<
                north_south.getSpeed(iAlt) << std::endl;
   }
}

int WeatherPrediction::incrementUpdateCount() {
   updateCount++;
   return updateCount;
}

int WeatherPrediction::getUpdateCount() const {
   return updateCount;
}
