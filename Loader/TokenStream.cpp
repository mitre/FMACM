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

#include "loader/TokenStream.h"

using namespace std;

// log4cplus::Logger TokenStream::logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("TokenStream"));

TokenStream::TokenStream(void) {
   error = false;
   indents = 0;
   echo = false;
   last_was_return = false;
   // echo = false;
   comment_not_end = false;
}

TokenStream::~TokenStream(void) {}

///////////////////////////////////////////////////////////////////////////////////
// public functions
///////////////////////////////////////////////////////////////////////////////////

// returns the next token, while not the end of file,
// get each character and check to see if its a comment,
// a quote, or a delimiter. Dont return at all if come across
//  a comment. Ensure you get end of quote if character
// is a quote. Return string if character is a delimiter
// otherwise keep getting characters

Token TokenStream::get_next() {
   Token out;
   char c;
   comment_not_end = false;

   assert(file.is_open());

   while (!file.eof()) {
      c = get_char();

      if (is_comment(c)) {
         while (true) {
            out.add_Format(c);
            c = get_char();
            if (c == '\n' || file.eof()) {
               break;
            }
         }
      }

      if (is_quote(c)) {
         return get_quote();
      }

      if (!is_delimiter(c)) {
         do {
            out.add_Data(c);
            c = get_char();
            if (is_delimiter(c) || is_comment(c)) {
               file.putback(c);
               break;
            }
         } while (!is_delimiter(c) && !file.eof());
         return out;
      }
      // must be a delimiter so add to format
      out.add_Format(c);
   }

   return out;
}

void TokenStream::close() { file.close(); }

void TokenStream::report_error(string message) {}

void TokenStream::report_warning(string message) {}

///////////////////////////////////////////////////////////////////////////////////
// protected functions
///////////////////////////////////////////////////////////////////////////////////

// returns the next character and echos to
// dump file and command line if echo=true
char TokenStream::get_char() {
   char out(0);
   file.get(out);
   if (file.eof() && comment_not_end) {
      report_error("\n\nERROR: End of file was reached before comment ended.");
      // LOG4CPLUS_ERROR(logger, "End of file was reached before comment ended.");
   }

   if (!file.eof()) {
      string out1;
      out1 = add_indent(out);
   }
   return out;
}

bool TokenStream::is_comment(char c) {
   if (c == ';') {
      return true;
   } else {
      return false;
   }
}

bool TokenStream::is_quote(char c) {
   if (c == '\"') {
      return true;
   } else {
      return false;
   }
}

bool TokenStream::is_char(char c) {
   if (!is_delimiter(c)) {
      return true;
   } else {
      return false;
   }
}

// a quote was found so the text inside the quote should
// be returned. Ensures that there is not a newline
// before the quote ends
Token TokenStream::get_quote() {
   Token out;
   char c;

   while (true) {
      c = get_char();
      if (c == '\n') {
         report_error("\n\nERROR: Newline before end of quote.");
         // LOG4CPLUS_ERROR(logger, "Newline before end of quote.");
      }
      if (is_quote(c)) {
         return out;
      }
      // out += c;
      out.add_Data(c);
   }
}

// returns true if a delimiter is found
bool TokenStream::is_delimiter(char ch) {
   switch (ch) {
      case ' ':  // space
      case ',':  // comma
      case 9:    // tab
      case 10:   // newline
      case -52:
         return true;
      default:
         return false;
   }
}

///////////////////////////////////////////////////////////////////////////////////
// private functions
///////////////////////////////////////////////////////////////////////////////////

string TokenStream::add_indent(char c) {
   string out;
   if (c == '\n') {
      last_was_return = true;
   } else {
      if (last_was_return) {
         for (int i = 0; i < indents; i++) {
            out = out + "   ";
         }
         last_was_return = false;
      }
   }
   out = out + c;
   return out;
}
