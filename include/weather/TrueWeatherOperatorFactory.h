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

#include "public/TrueWeatherOperator.h"
#include "public/ZeroWindTrueWeatherOperator.h"
#include "public/FullWindTrueWeatherOperator.h"
#include "public/Wind.h"

namespace aaesim::weather {
class TrueWeatherOperatorFactory {
  public:
   static std::shared_ptr<aaesim::open_source::TrueWeatherOperator> BuildTrueWeatherOperator(
         std::shared_ptr<aaesim::open_source::WeatherTruth> &true_weather) {
      if (Wind::UseWind()) {
         return std::make_unique<aaesim::open_source::FullWindTrueWeatherOperator>(true_weather);
      }

      return std::make_unique<aaesim::open_source::ZeroWindTrueWeatherOperator>(true_weather);
   }
};
}  // namespace aaesim::weather
