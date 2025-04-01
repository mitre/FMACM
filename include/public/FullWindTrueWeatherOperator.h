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

#include "public/AbstractTrueWeatherOperator.h"

namespace aaesim::open_source {
class FullWindTrueWeatherOperator final : public AbstractTrueWeatherOperator {
  public:
   FullWindTrueWeatherOperator(std::shared_ptr<aaesim::open_source::WeatherTruth> true_weather)
      : AbstractTrueWeatherOperator(true_weather) {}
   ~FullWindTrueWeatherOperator() = default;
   void CalculateEnvironmentalWind(const EarthModel::GeodeticPosition &position,
                                   const Units::Length &altitude_msl) override;
   Units::Speed GetWindSpeedEast() const override;
   Units::Speed GetWindSpeedNorth() const override;
   Units::Frequency GetWindSpeedVerticalDerivativeEast() const override;
   Units::Frequency GetWindSpeedVerticalDerivativeNorth() const override;

  private:
   Units::Speed m_wind_speed_east{Units::zero()};
   Units::Speed m_wind_speed_north{Units::zero()};
   Units::Frequency m_vertical_derivative_east{Units::zero()};
   Units::Frequency m_vertical_derivative_north{Units::zero()};
};
}  // namespace aaesim::open_source
