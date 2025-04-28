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

#pragma once

#include "loader/Token.h"

template <class PARENT>

class ArchiveStreamEcho : public PARENT {
  public:
   ArchiveStreamEcho(void) {}

   ~ArchiveStreamEcho(void) {
      if (archive_file.is_open()) {
         archive_file.close();
      }
   }

   // opens the archive file and writes a header comment at the top
   bool open_Archive(const std::string &new_file_name, const std::string &header_comment) {
      archive_file.open(new_file_name.c_str());
      if (archive_file.is_open()) {
         archive_file << ";";
         archive_file << header_comment;
         archive_file << "\n";
         return true;
      }
      return false;
   }

   // rewrites the include statement in the archive of the primary file
   void rewrite_Last_in_Archive(const std::string &s) { old_token.set_data(s); }

   // writes the tokens into the archive file that is open and calls get next
   Token get_next() {
      if (archive_file.is_open()) {
         archive_file << old_token.get_All();
      }
      old_token = PARENT::get_next();
      return old_token;
   }  //--------------------------------------------

   ArchiveStreamEcho &operator=(const ArchiveStreamEcho &rhs) {
      PARENT::operator=(rhs);

      return *this;
   }  //--------------------------------------------

   // makes sure that the old_token is still written into the secondary open archive file
   // before it is closed
   void write_Last_Token(const std::string &old_token) {
      if (archive_file.is_open()) {
         archive_file << old_token;
      }
   }

  protected:
   std::ofstream archive_file;
   Token old_token;
};
