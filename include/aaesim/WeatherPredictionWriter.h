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

#include "aaesim/AircraftDataIterationWriter.h"

namespace aaesim {
class WeatherPredictionWriter final : public AircraftDataIterationWriter {
  public:
   WeatherPredictionWriter() : AircraftDataIterationWriter("", "_fms_predicted_weather.csv") {}

   void Finish() override;

   void Gather(const int &iteration_number, const aaesim::AircraftEntity &aircraft) override;

  private:
   static log4cplus::Logger m_logger;
   static std::vector<std::string> COLUMN_NAMES;

   struct DataToWrite {
      DataToWrite()
         : iteration_number(INT32_MIN),
           acid(),
           simtime(Units::negInfinity()),
           altitude_msl(Units::negInfinity()),
           enu_east_component(Units::negInfinity()),
           enu_north_component(Units::negInfinity()),
           wind_magnitude(Units::negInfinity()),
           enu_winds_to(Units::negInfinity()),
           ned_winds_from(Units::negInfinity()),
           temperature() {}

      int iteration_number;
      std::string acid;
      Units::SecondsTime simtime;
      Units::MetersLength altitude_msl;
      Units::MetersPerSecondSpeed enu_east_component;
      Units::MetersPerSecondSpeed enu_north_component;
      Units::MetersPerSecondSpeed wind_magnitude;
      Units::DegreesAngle enu_winds_to;
      Units::DegreesAngle ned_winds_from;
      Units::AbsCelsiusTemperature temperature;
   };

   std::vector<DataToWrite> m_data_to_write;
};
}  // namespace aaesim