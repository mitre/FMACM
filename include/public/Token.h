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

#include <string>

class Token {
  public:
   Token(void) {}  //--------------------------------------------------------------

   inline void add_Data(char c) { data += c; }  //--------------------------------------------------------------
   inline void add_Data(const std::string &s) {
      data += s;
   }  //--------------------------------------------------------------

   inline void add_Format(char c) { format += c; }  //--------------------------------------------------------------
   inline void add_Format(const std::string &s) {
      format += s;
   }  //--------------------------------------------------------------
   inline void add_Format(const Token &t) {
      add_Format(t.get_Format());
      add_Format(t.get_Data());
   }  //--------------------------------------------------------------
   inline void merge_data_into_format() {
      format += data;
      data = "";
   }  //--------------------------------------------------------------
   inline std::string get_Data() const {
      return data;
   }  //--------------------------------------------------------------
   inline std::string get_Format() const {
      return format;
   }  //--------------------------------------------------------------

   inline std::string get_All() const {
      return format + data;
   }  //--------------------------------------------------------------

   inline void set_data(std::string const &nd) {
      data = nd;
   }  //--------------------------------------------------------------
   inline void set_format(std::string const &nf) {
      format = nf;
   }  //--------------------------------------------------------------

  private:
   std::string data;
   std::string format;
};
