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
 * RefReader.h
 *
 * Reads a Ref.csv file containing a sequence of reference values.
 *
 *  Created on: Mar 23, 2019
 *      Author: klewis
 */

#pragma once

#include "public/DataReader.h"

namespace testvector {

class RefReader : public DataReader {
public:
   RefReader(std::string file_name, int header_lines, size_t expected_columns);
   RefReader(std::shared_ptr<std::istream> input_stream, int header_lines, size_t expected_columns);
   virtual ~RefReader();
   virtual bool Advance();
   const Units::SecondsTime GetTimeToFly() const;

private:
   Units::SecondsTime m_time_to_fly; // column 1

};


} // namespace testvector
