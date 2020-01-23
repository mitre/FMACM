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
 * RefReader.cpp
 *
 *  Created on: Mar 23, 2019
 *      Author: klewis
 */

#include "public/RefReader.h"

namespace testvector {

RefReader::RefReader(std::string file_name, int header_lines, size_t expected_columns) :
         DataReader(file_name, header_lines, expected_columns) {
}

RefReader::RefReader(std::shared_ptr<std::istream> input_stream, int header_lines, size_t expected_columns) :
         DataReader(input_stream, header_lines, expected_columns) {
}

RefReader::~RefReader() {
}

bool RefReader::Advance() {
   bool have_data = DataReader::Advance();
   if (have_data) {
      m_time_to_fly = Units::SecondsTime(GetDouble(0));
   }
   else {
      m_time_to_fly = UNDEFINED_TIME;
   }

   return have_data;
}

const Units::SecondsTime RefReader::GetTimeToFly() const {
   return m_time_to_fly;
}

} // namespace testvector
