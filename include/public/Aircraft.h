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

#pragma once

#include "public/WeatherTruth.h"
#include "public/LoggingLoadable.h"
#include "public/SimulationTime.h"

namespace aaesim {
namespace open_source {
struct Aircraft : public LoggingLoadable {
   Aircraft(void) = default;

   virtual ~Aircraft(void) = default;

   virtual void Initialize(const Units::Length adsb_reception_range_threshold, const WeatherTruth &weather_truth) = 0;

   virtual bool Update(const SimulationTime &simulation_time) = 0;
};
}  // namespace open_source

}  // namespace aaesim
