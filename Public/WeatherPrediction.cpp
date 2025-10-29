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

#include "public/Atmosphere.h"
#include "public/WeatherPrediction.h"

using namespace aaesim::open_source;

WeatherPrediction::WeatherPrediction(std::shared_ptr<Wind> wind, std::shared_ptr<Atmosphere> atmosphere)
   : WeatherEstimate(std::move(wind), std::move(atmosphere)), m_update_count(0) {
   // inhibit 3-D predicted temperature for now.
   m_temperature_checked = true;
   m_temperature_available = false;
}

WeatherPrediction WeatherPrediction::CreateZeroWindPrediction(std::shared_ptr<Atmosphere> atmosphere) {
   aaesim::open_source::WindStack zeroWinds(1, 5);
   zeroWinds.Insert(1, Units::FeetLength(0.), Units::KnotsSpeed(0.));
   zeroWinds.Insert(2, Units::FeetLength(10000.), Units::KnotsSpeed(0.));
   zeroWinds.Insert(3, Units::FeetLength(20000.), Units::KnotsSpeed(0.));
   zeroWinds.Insert(4, Units::FeetLength(30000.), Units::KnotsSpeed(0.));
   zeroWinds.Insert(5, Units::FeetLength(50000.), Units::KnotsSpeed(0.));
   WeatherPrediction zeroWeather;
   zeroWeather.east_west() = zeroWinds;
   zeroWeather.north_south() = zeroWinds;
   zeroWeather.SetAtmosphere(atmosphere);
   return zeroWeather;
}
