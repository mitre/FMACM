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
// Copyright 2019 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#pragma once

#include "public/Token.h"
#include <string>

template<class PARENT>
class SinglePushBackStream : public PARENT
{
public:
   SinglePushBackStream(void) {
      pushed = false;
   }

   //-------------------------------------------------------------------------------------------------------------------------

   Token get_next() {
      if (pushed) {
         pushed = false;
      } else {
         old_token = PARENT::get_next();
      }

      return old_token;
   }

   void push_back()// call this to reject the last token
   {
      pushed = true;
   }   // if you call this after you get a token you will get the same
   // token again when you call get_next Again

   bool set_html_output_file(const std::string &dump_file_name) {
      return PARENT::set_output_file(dump_file_name, 0);
   }

private:

   Token old_token;
   bool pushed;

};



