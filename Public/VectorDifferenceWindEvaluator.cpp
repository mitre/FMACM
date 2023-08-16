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

#include "public/VectorDifferenceWindEvaluator.h"
#include "public/Environment.h"

using namespace aaesim::open_source;

std::map<Units::Speed, std::weak_ptr<PredictedWindEvaluator> > VectorDifferenceWindEvaluator::m_instances;

const std::shared_ptr<PredictedWindEvaluator> VectorDifferenceWindEvaluator::GetInstance(
      const Units::Speed maxSpeedDiff) {
   std::weak_ptr<PredictedWindEvaluator> cached = m_instances[maxSpeedDiff];
   std::shared_ptr<PredictedWindEvaluator> result = cached.lock();
   if (!result) {
      result = std::shared_ptr<PredictedWindEvaluator>(new VectorDifferenceWindEvaluator(maxSpeedDiff));
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
      const Atmosphere *sensed_atmosphere) const {

   Units::MetersPerSecondSpeed windeastcomp, windnorthcomp;  // units of mps as returned from AircraftCalculations
   Units::Frequency dtmp;
   weather_prediction.getAtmosphere()->CalculateWindGradientAtAltitude(
         Units::FeetLength(state.m_z), weather_prediction.east_west, windeastcomp, dtmp);
   weather_prediction.getAtmosphere()->CalculateWindGradientAtAltitude(
         Units::FeetLength(state.m_z), weather_prediction.north_south, windnorthcomp, dtmp);

   Units::Speed xDiff = Units::MetersPerSecondSpeed(state.m_Vwx) - windeastcomp;
   Units::Speed yDiff = Units::MetersPerSecondSpeed(state.m_Vwy) - windnorthcomp;

   return Units::sqr(xDiff) + Units::sqr(yDiff) <= Units::sqr(m_max_allowed_difference);
}
