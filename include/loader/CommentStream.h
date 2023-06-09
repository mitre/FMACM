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
//#include "token_stream.h"
#include "public/Token.h"

template <class PARENT>
class CommentStream : public PARENT {
  public:
   CommentStream(void) {}

   ~CommentStream(void) {}

   Token get_next() {
      Token out;
      char first;
      std::string comm;
      char next_ch;
      bool maybe = false;
      bool yes_comm = false;

      // iterate through characters to look for delimiters before a block comment
      // add delimiters to the format part of token. if you find a / then check to see if the next is
      //  a *. If it is then you have a block comment and go through the comment stream, else put back
      //  the characters / and whatever was next and call get_next
      first = PARENT::get_char();

      if (PARENT::is_delimiter(first)) {
         while (PARENT::is_delimiter(first)) {
            if (PARENT::file.eof()) {
               break;
            }
            out.add_Format(first);
            first = PARENT::get_char();
         }
      }
      if (first == '/') {
         char second = PARENT::get_char();
         if (second == '*') {
            yes_comm = true;
         } else {
            PARENT::file.putback(second);
            PARENT::file.putback(first);
         }
      } else if (!PARENT::file.eof()) {
         PARENT::file.putback(first);
      }

      if (!yes_comm) {
         Token out1 = PARENT::get_next();
         out.add_Format(out1.get_Format());
         out.add_Data(out1.get_Data());

         return out;
      } else  // look for long comment delimiter
      {
         PARENT::comment_not_end = true;
         out.add_Format('/');
         out.add_Format('*');

         while (true) {
            next_ch = PARENT::get_char();  // get the next character

            if (maybe)  // if the last character was a * then look for / to end comment
            {
               if (next_ch == '/') {
                  out.add_Format(next_ch);
                  break;
               } else  // no / was found so reset maybe to false and loop through again.
               {
                  out.add_Format(next_ch);
                  maybe = false;
               }
            }

            if (next_ch == '*')  // look for potential beginning of end comment delimiter
            {
               maybe = true;
            }
            out.add_Format(next_ch);
         }

         Token out2 = PARENT::get_next();
         out.add_Format(out2.get_Format());
         out.add_Data(out2.get_Data());

         return out;
      }

      // not a comment so get next
   }
};
