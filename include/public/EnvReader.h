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
 * EnvReader.h
 *
 * Reads an ENV.csv file containing a sequence of local weather observations
 *
 *  Created on: Mar 23, 2019
 *      Author: klewis
 */

#pragma once

#include "public/DataReader.h"

namespace testvector {

class EnvReader : public DataReader {
public:
   static const size_t EXPECTED_ENV_COLUMN_COUNT;
   EnvReader(std::string file_name, int header_lines);
   EnvReader(std::shared_ptr<std::istream> input_stream, int header_lines);
   EnvReader();
   virtual ~EnvReader();
   virtual bool Advance();
   const Units::SecondsTime GetTime() const;

private:
   Units::SecondsTime m_time; // column 1

};

} // namespace testvector

