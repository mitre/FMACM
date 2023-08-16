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

/*
 * DataReader.h
 *
 * Reads any .csv file
 *
 *  Created on: Mar 23, 2019
 *      Author: klewis
 */

#pragma once

#include <map>
#include <memory>
#include <istream>
#include "utility/CsvParser.h"
#include "public/Logging.h"
#include <scalar/Time.h>

namespace aaesim {
namespace open_source {

class DataReader {
  public:
   static const Units::SecondsTime UNDEFINED_TIME;
   static log4cplus::Logger m_logger;
   DataReader() = default;
   DataReader(std::string file_name, int header_lines, size_t expected_columns);
   DataReader(std::shared_ptr<std::istream> input_stream, int header_lines, size_t expected_columns);
   virtual ~DataReader();
   void OpenFile(std::string file_name, int header_lines);
   void OpenStream(std::shared_ptr<std::istream> input_stream, int header_lines);
   virtual bool Advance();
   double GetDouble(int column) const;
   std::string GetString(int column) const;
   size_t GetColumnCount() const;

  protected:
   void BuildColumnIndex();
   int GetColumnNumber(const std::string &column_name);
   void SetExpectedColumnCount(size_t expected_column_count);
   void SkipLines(int header_lines);

  private:
   std::shared_ptr<std::istream> m_input_stream;
   size_t m_expected_column_count;
   CsvParser::CsvRow m_csv_row;
   std::map<std::string, int> m_column_index;
};

}  // namespace open_source
}  // namespace aaesim