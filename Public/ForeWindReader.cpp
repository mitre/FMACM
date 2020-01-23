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
// Copyright 2020 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

/*
 * ForeWindReader.cpp
 *
 *  Created on: Mar 23, 2019
 *      Author: klewis
 */

#include "public/ForeWindReader.h"

namespace testvector {

ForeWindReader::ForeWindReader(std::string file_name, int header_lines) :
DataReader(file_name, header_lines, 0) {
}

ForeWindReader::ForeWindReader(std::shared_ptr<std::istream> input_stream, int header_lines) :
               DataReader(input_stream, header_lines, 0) {
}

ForeWindReader::~ForeWindReader() {
}

bool ForeWindReader::ReadWind(WeatherPrediction &weather_prediction) {
   const int ALTITUDE_COUNT(5);

   weather_prediction.east_west.SetBounds(1, ALTITUDE_COUNT);
   weather_prediction.north_south.SetBounds(1, ALTITUDE_COUNT);
   for (int i = 1; i <= ALTITUDE_COUNT; i++) {
      Advance();
      if (GetColumnCount() == 0) {
         // end of stream
         return false;
      }
      Units::MetersLength altitude(GetDouble(0));
      Units::MetersPerSecondSpeed u(GetDouble(1));
      Units::MetersPerSecondSpeed v(GetDouble(2));
      weather_prediction.east_west.Set(i, altitude, u);
      weather_prediction.north_south.Set(i, altitude, v);
   }

   return true;
}

} // namespace testvector
