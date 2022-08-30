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

#include "public/Token.h"
#include "utility/MoreStringFunctions.h"
#include "loader/HTMLDump.h"
#include <sstream>

template<class PARENT>

class FormatDifferentiatorStreamEcho : public PARENT
{
public:

   FormatDifferentiatorStreamEcho(void) {
   }//--------------------------------------------

   ~FormatDifferentiatorStreamEcho(void) {
   }//--------------------------------------------

   //highlight the format part in blue and the data part in yellow, mainly
   //used for debug purposes
   Token get_next() {
      Token out;
      out = PARENT::get_next();
      if (PARENT::get_echo()) {
         std::string data = out.get_Data();
         std::string format = out.get_Format();

         dump_file.highlight_on("blue");
         std::string enhanced_format = MoreStringFunctions::FindAndReplace(format, "\n",
                                                                           std::string("\n") + PARENT::get_Space());
         dump_file.dump(enhanced_format);
         dump_file.highlight_off();

         dump_file.highlight_on("yellow");
         dump_file.dump(data);
         dump_file.highlight_off();
      }

      return out;

   }//--------------------------------------------

   void report_error(std::string message) {
      dump_file.highlight_on("red");

      dump_file.dump(message);

      dump_file.highlight_off();

      PARENT::report_error(message);

   }//--------------------------------------------

   void report_warning(std::string message) {
      dump_file.highlight_on("green");

      dump_file.dump(message);

      dump_file.highlight_off();

      PARENT::report_warning(message);

   }//--------------------------------------------

   void print_to_dumpfile(std::string dump) {
      dump_file.dump(dump);
      PARENT::print_to_dumpfile(dump);
   }//--------------------------------------------

   FormatDifferentiatorStreamEcho &operator=(const FormatDifferentiatorStreamEcho &rhs) {
      PARENT::operator=(rhs);

      return *this;
   }//--------------------------------------------


private:

   HTMLDump dump_file; //the dump file

protected:

   bool set_output_file(const std::string &dump_file_name,
                        int i) {
      std::stringstream ss;
      ss << i;
      std::string html = ".html";
      if (i == 0) {
         dump_file.open((dump_file_name + html).c_str());
      } else {
         dump_file.open((dump_file_name + ss.str() + html).c_str());
      }
      bool opened = dump_file.is_open();
      return opened && PARENT::set_output_file(dump_file_name, i + 1);
   }//--------------------------------------------

};
