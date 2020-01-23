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
 * ForeWindReader.h
 *
 * Reads a ForeWind.csv file containing a list of altitudes and wind velocities
 *
 *  Created on: Mar 23, 2019
 *      Author: klewis
 */

#pragma once

#include "public/DataReader.h"
#include "public/WeatherPrediction.h"

namespace testvector {

class ForeWindReader : DataReader {
public:
   ForeWindReader(std::string file_name, int header_lines);
   ForeWindReader(std::shared_ptr<std::istream> input_stream, int header_lines);
   virtual ~ForeWindReader();
   bool ReadWind(WeatherPrediction &weather_prediction);

private:
};

} // namespace testvector

