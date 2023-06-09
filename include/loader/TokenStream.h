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
#include <fstream>
#include "assert.h"
#include "public/Token.h"
#include "loader/IndentManagement.h"

/* #include "utility/Logging.h" */

class TokenStream : public IndentManagement {
  public:
   TokenStream(void);  // constructor

   virtual ~TokenStream() = 0;

   virtual bool set_output_file(const std::string &file_name,
                                int i)  // function to open dump file specified
   {
      return true;
   }  //------------------------------------------------

   virtual bool open_file(const std::string &file_name) {
      file.open(file_name.c_str());
      return file.is_open();
   }  //------------------------------------------------

   virtual void highlight_on() {}  //------------------------------------------------

   virtual void highlight_off() {}  //------------------------------------------------

   bool open_Archive(const std::string &new_file_name, const std::string &header_comment) {
      return true;
   }  //------------------------------------------------

   virtual void close_dump_file()  // function to close the dump file specified
   {}                              //------------------------------------------------

   Token get_next();                                  // get next token
   virtual void close();                              // close file
   virtual void report_error(std::string message);    // place error message in dump file
   virtual void report_warning(std::string message);  // place warning message in dump file

   // inline
   virtual bool ok()  // for testing/debugging
   {
      return !error;
   }  //------------------------------------------------

   virtual void set_echo(bool new_echo)  // set bool echo variable to print out to command line or not
   {
      echo = new_echo;
   }  //------------------------------------------------

   virtual bool get_echo()  // get the private variable echo to determine whether to print out to commane line or not
   {
      return echo;
   }  //------------------------------------------------

   virtual Token get_Next_From_Parent()  // this a do nothing function that
   {
      Token out;
      return out;
   }

   virtual inline char get_character() {
      char out = ' ';
      assert(false);
      return out;
   }  //------------------------------------------------

   TokenStream &operator=(const TokenStream &rhs) {
      IndentManagement::operator=(rhs);
      indents = rhs.indents;
      i = rhs.i;
      s = rhs.s;
      comment_not_end = rhs.comment_not_end;
      echo = rhs.echo;
      error = rhs.error;
      last_was_return = rhs.last_was_return;
      return *this;
   }

  protected:
   std::ifstream file;       // name of the input file
   char get_char();          // get next character from the file
   bool is_comment(char c);  // check if the character is a ;
   bool is_quote(char c);    // check if the characater is a "
   bool is_char(char c);     // check if the character is a regular non delimiter
   // Token skip_comment(); //dont return a comment to get next
   Token get_quote();  // get the quoted text

   virtual void echo_out(std::string dump)  // print the string- dump to cline and dump file
   {}                                       //------------------------------------------------

   virtual void print_to_dumpfile(std::string dump) {}  //------------------------------------------------

   int indents;

   bool is_delimiter(char ch);  // check to see if the character is one of the designated delimiters
   bool comment_not_end;
   bool echo;  // true or false for printing out to command line

  private:
   /* static log4cplus::Logger logger; */

   bool error;  // for error checking
   // bool borrowed_dump; //bool to ensure only original file deletes itself
   // void print_to_dumpfile(string dump); //print the string to the dump file
   // void print_to_cout(string dump); //print the string to the command line
   std::string add_indent(char c);

   bool last_was_return;
};
