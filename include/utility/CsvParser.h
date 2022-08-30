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

#pragma once

/*
 * Originally from http://stackoverflow.com/questions/1120140/how-can-i-read-and-parse-csv-files-in-c
 *
 * Use in an interator loop, like this:
    std::ifstream       file("testfile.csv");

    for(CSVIterator loop(file);loop != CSVIterator();++loop)
    {
        std::cout << "4th Element(" << (*loop)[3] << ")\n";
    }
 *
 */

#include <iterator>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

namespace CsvParser {

   class CsvRow
   {
   public:
      std::string const &operator[](std::size_t index) const {
         return m_data[index];
      }

      std::size_t Size() const {
         return m_data.size();
      }

      void ReadNextRow(std::istream &str) {
         std::string line;
         std::getline(str, line);

         std::stringstream lineStream(line);
         std::string cell;

         m_data.clear();
         while (std::getline(lineStream, cell, ',')) {
            m_data.push_back(cell);
         }
      }

   private:
      std::vector<std::string> m_data;
   };

   inline std::istream &operator>>(std::istream &str,
                                   CsvRow &data) {
      data.ReadNextRow(str);
      return str;
   }

   class CsvIterator
   {
   public:
      typedef std::input_iterator_tag iterator_category;
      typedef CsvRow value_type;
      typedef std::size_t difference_type;
      typedef CsvRow *pointer;
      typedef CsvRow &reference;

      CsvIterator(std::istream &str)
            : m_str(str.good() ? &str : NULL) {
         ++(*this);
      }

      CsvIterator()
            : m_str(NULL) {
      }

      // Pre Increment
      CsvIterator &operator++() {
         if (m_str) {
            (*m_str) >> m_row;
            m_str = m_str->good() ? m_str : NULL;
         }
         return *this;
      }

      // Post increment
      CsvIterator operator++(int) {
         CsvIterator tmp(*this);
         ++(*this);
         return tmp;
      }

      CsvRow const &operator*() const {
         return m_row;
      }

      CsvRow const *operator->() const {
         return &m_row;
      }

      bool operator==(CsvIterator const &rhs) {
         return ((this == &rhs) || ((this->m_str == NULL) && (rhs.m_str == NULL)));
      }

      bool operator!=(CsvIterator const &rhs) {
         return !((*this) == rhs);
      }

   private:
      std::istream *m_str;
      CsvRow m_row;
   };
}

