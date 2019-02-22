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

template<class PARENT>

class ArchiveStreamEcho : public PARENT
{
public:

   ArchiveStreamEcho(void) {
   }

   ~ArchiveStreamEcho(void) {
      if (archive_file.is_open()) {
         archive_file.close();
      }
   }

   //opens the archive file and writes a header comment at the top
   bool open_Archive(const std::string &new_file_name,
                     const std::string &header_comment) {
      archive_file.open(new_file_name.c_str());
      if (archive_file.is_open()) {
         archive_file << ";";
         archive_file << header_comment;
         archive_file << "\n";
         return true;
      }
      return false;
   }

   //rewrites the include statement in the archive of the primary file
   void rewrite_Last_in_Archive(const std::string &s) {
      old_token.set_data(s);
   }

   //writes the tokens into the archive file that is open and calls get next
   Token get_next() {
      if (archive_file.is_open()) {
         archive_file << old_token.get_All();
      }
      old_token = PARENT::get_next();
      return old_token;
   }//--------------------------------------------

   ArchiveStreamEcho &operator=(const ArchiveStreamEcho &rhs) {
      PARENT::operator=(rhs);

      return *this;
   }//--------------------------------------------

   //makes sure that the old_token is still written into the secondary open archive file
   //before it is closed
   void write_Last_Token(std::string old_token) {
      if (archive_file.is_open()) {
         archive_file << old_token;
      }
   }

protected:

   std::ofstream archive_file;
   Token old_token;
};
