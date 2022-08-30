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
 * EnvReader.cpp
 *
 *  Created on: Mar 23, 2019
 *      Author: klewis
 */

#include "public/EnvReader.h"
#include "utility/constants.h"
#include <scalar/Angle.h>
#include <scalar/Speed.h>
#include <scalar/Temperature.h>

namespace testvector {

const size_t EnvReader::EXPECTED_ENV_COLUMN_COUNT(7);

/*
 * FIXME -- There is currently an issue with the ENV files
 * in the test vectors.  Tv02 has 5-column files vs. all
 * other vectors have 7-columns.  For now, we are setting
 * expected_columns to 0 in the superclass constructor,
 * which inhibits column-count checking.
 */
EnvReader::EnvReader(std::string file_name, int header_lines) :
   DataReader(file_name, header_lines, 0 /* EXPECTED_ENV_COLUMN_COUNT */) {
}

EnvReader::EnvReader(std::shared_ptr<std::istream> input_stream, int header_lines) :
   DataReader(input_stream, header_lines, 0 /* EXPECTED_ENV_COLUMN_COUNT */) {
}

EnvReader::~EnvReader() {
}

bool EnvReader::Advance() {
   bool result = DataReader::Advance();
   if (result) {
      m_time = Units::SecondsTime(GetDouble(0));
   }
   else {
      m_time = DataReader::UNDEFINED_TIME;
   }
   return result;
}

const Units::SecondsTime EnvReader::GetTime() const {
   return m_time;
}

} // namespace testvector
