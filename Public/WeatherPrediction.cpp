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
// 2022 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#include "public/WeatherPrediction.h"
#include "public/StandardAtmosphere.h"

const PredictedWindOption WeatherPrediction::PWOValues[3] = {SINGLE_DTG, MULTIPLE_DTG_LEGACY, MULTIPLE_DTG_ALONG_ROUTE};

WeatherPrediction::WeatherPrediction() : WeatherEstimate(), m_predicted_wind_option(SINGLE_DTG), m_update_count(0) {}

WeatherPrediction::WeatherPrediction(PredictedWindOption option, std::shared_ptr<Wind> wind,
                                     std::shared_ptr<Atmosphere> atmosphere)
   : WeatherEstimate(std::move(wind), std::move(atmosphere)), m_predicted_wind_option(option), m_update_count(0) {
   // inhibit 3-D predicted temperature for now.
   m_temperature_checked = true;
   m_temperature_available = false;
}

WeatherPrediction::~WeatherPrediction() = default;

const void WeatherPrediction::Dump() const {
   for (int iAlt = east_west.GetMinRow(); iAlt <= east_west.GetMaxRow(); iAlt++) {
      std::cout << iAlt << ":  " << east_west.GetAltitude(iAlt) << " " << east_west.GetSpeed(iAlt) << " "
                << north_south.GetSpeed(iAlt) << std::endl;
   }
   std::cout << std::endl;
}
