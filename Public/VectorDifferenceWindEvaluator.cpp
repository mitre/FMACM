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
// (c) 2026 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#include "public/VectorDifferenceWindEvaluator.h"

#include <map>
#include <memory>

#include "public/Environment.h"

using namespace aaesim::open_source;

std::map<Units::Speed, std::weak_ptr<PredictedWindEvaluator> > VectorDifferenceWindEvaluator::m_instances;

const std::shared_ptr<PredictedWindEvaluator> VectorDifferenceWindEvaluator::GetInstance(
      const Units::Speed maxSpeedDiff) {
   std::weak_ptr<PredictedWindEvaluator> cached = m_instances[maxSpeedDiff];
   std::shared_ptr<PredictedWindEvaluator> result = cached.lock();
   if (!result) {
      result = std::unique_ptr<PredictedWindEvaluator>(new VectorDifferenceWindEvaluator(maxSpeedDiff));
      cached = result;
      m_instances[maxSpeedDiff] = cached;
   }
   return result;
}

VectorDifferenceWindEvaluator::VectorDifferenceWindEvaluator(const Units::Speed &max_allowed_difference)
   : m_max_allowed_difference(max_allowed_difference) {}

VectorDifferenceWindEvaluator::~VectorDifferenceWindEvaluator() {}

bool VectorDifferenceWindEvaluator::ArePredictedWindsAccurate(
      const aaesim::open_source::AircraftState &state, const aaesim::open_source::WeatherPrediction &weather_prediction,
      const Units::Speed reference_cas, const Units::Length reference_altitude,
      const std::shared_ptr<Atmosphere> &sensed_atmosphere) const {
   Units::MetersPerSecondSpeed wind_east, wind_north;
   Units::Frequency dtmp;
   weather_prediction.east_west().CalculateWindGradientAtAltitude(Units::FeetLength(state.GetAltitudeMsl()), wind_east,
                                                                  dtmp);
   weather_prediction.north_south().CalculateWindGradientAtAltitude(Units::FeetLength(state.GetAltitudeMsl()),
                                                                    wind_north, dtmp);

   Units::Speed x_diff = state.GetSensedWindEast() - wind_east;
   Units::Speed y_diff = state.GetSensedWindNorth() - wind_north;

   return Units::sqr(x_diff) + Units::sqr(y_diff) <= Units::sqr(m_max_allowed_difference);
}
