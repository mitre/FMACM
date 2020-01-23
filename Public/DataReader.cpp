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
 * DataReader.cpp
 *
 *  Created on: Mar 23, 2019
 *      Author: klewis
 */

#include "public/DataReader.h"

namespace testvector {

const Units::SecondsTime DataReader::UNDEFINED_TIME(-9999);

log4cplus::Logger DataReader::m_logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("DataReader"));

DataReader::DataReader(std::string file_name, int header_lines, size_t expected_columns) :
         m_expected_column_count(expected_columns) {
   OpenFile(file_name, header_lines);
}

DataReader::DataReader(std::shared_ptr<std::istream> input_stream, int header_lines, size_t expected_columns) :
         m_expected_column_count(expected_columns) {
   OpenStream(input_stream, header_lines);
}

DataReader::~DataReader() {
}

void DataReader::OpenFile(std::string file_name, int header_lines) {
   std::shared_ptr<std::istream> input_stream(new std::ifstream(file_name));
   OpenStream(input_stream, header_lines);
}

void DataReader::OpenStream(std::shared_ptr<std::istream> input_stream,
      int header_lines) {
   m_input_stream = input_stream;
   SkipLines(header_lines);
}

void DataReader::SkipLines(int header_lines) {
   for (int i = 0; i < header_lines; i++) {
      m_input_stream->ignore(1000, '\n');
   }
}

bool DataReader::Advance() {
   m_csv_row.ReadNextRow(*m_input_stream);
   if (m_csv_row.Size() == 0) {
      // end of stream
      return false;
   }

   if ((m_expected_column_count != 0) && (m_csv_row.Size() != m_expected_column_count)) {
      std::cout << "Unexpected column count in line in data file:  " << m_csv_row.Size() << std::endl;
   }

   return true;
}

double DataReader::GetDouble(int column) const {
   std::istringstream iss(m_csv_row[column]);
   double val = 0;
   iss >> val;
   return val;
}

std::string DataReader::GetString(int column) const {
   return m_csv_row[column];
}

size_t DataReader::GetColumnCount() const {
   return m_csv_row.Size();
}

void DataReader::SetExpectedColumnCount(size_t expected_column_count) {
   m_expected_column_count = expected_column_count;
}

void DataReader::BuildColumnIndex() {
   // build the column index
   for (int i = 0; i < GetColumnCount(); i++) {
      std::string name(GetString(i));
      size_t crpos(name.rfind('\r'));
      if (crpos != std::string::npos) {
         name.erase(crpos, 1);
      }
      m_column_index[name] = i;
   }

   int column_count0 = m_column_index.size();
   if (column_count0 != GetColumnCount()) {
      throw std::runtime_error("Duplicate column names in CSV file");
   }
}

int DataReader::GetColumnNumber(const std::string& column_name) {
   int column_count0 = m_column_index.size();
   int index = m_column_index[column_name];
   int column_count1 = m_column_index.size();
   if (column_count0 != column_count1) {
      LOG4CPLUS_WARN(m_logger, "Column \"" << column_name << "\" not found.  Will use " << index);
   }
   return index;
}

} // namespace testvector
