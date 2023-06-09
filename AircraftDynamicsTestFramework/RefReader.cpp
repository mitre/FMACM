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

/*
 * RefReader.cpp
 *
 *  Created on: Mar 23, 2019
 *      Author: klewis
 */

#include "framework/RefReader.h"

namespace testvector {

RefReader::RefReader(std::string file_name, int header_lines, size_t expected_columns)
   : DataReader(file_name, header_lines, expected_columns) {}

RefReader::RefReader(std::shared_ptr<std::istream> input_stream, int header_lines, size_t expected_columns)
   : DataReader(input_stream, header_lines, expected_columns) {}

RefReader::~RefReader() {}

bool RefReader::Advance() {
   bool have_data = DataReader::Advance();
   if (have_data) {
      m_time_to_fly = Units::SecondsTime(GetDouble(0));
   } else {
      m_time_to_fly = UNDEFINED_TIME;
   }

   return have_data;
}

const Units::SecondsTime RefReader::GetTimeToFly() const { return m_time_to_fly; }

}  // namespace testvector
