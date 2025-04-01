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
#include "public/WeatherTruth.h"

namespace aaesim::open_source {
class AbstractTrueWeatherOperator : public TrueWeatherOperator {
  public:
   AbstractTrueWeatherOperator(std::shared_ptr<aaesim::open_source::WeatherTruth> true_weather)
      : m_true_weather{true_weather} {}
   ~AbstractTrueWeatherOperator() = default;
   Units::KelvinTemperature GetTemperature() const override { return m_true_weather->GetTemperature(); }
   Units::Density GetDensity() const override { return m_true_weather->GetDensity(); }
   Units::Pressure GetPressure() const override { return m_true_weather->GetPressure(); }
   std::shared_ptr<const Atmosphere> GetAtmosphere() const override {
      throw std::runtime_error("AAES-1545: Do not call this method. Design error that needs to be fixed!");
   }
   std::shared_ptr<const aaesim::open_source::WeatherTruth> GetTrueWeather() const override { return m_true_weather; }

  protected:
   std::shared_ptr<aaesim::open_source::WeatherTruth> m_true_weather{};
};
}  // namespace aaesim::open_source
