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

#include "utility/FilePath.h"
#include <iostream>
#include <map>

class RunFileArchiveDirector
{
public:
   RunFileArchiveDirector(void);

   ~RunFileArchiveDirector(void);

   inline void set_Destination_Directory(const FilePath &destination_file_path) {
      destination = destination_file_path;
   }//------------------------------------------------------------------------

   inline std::string get_Destination() {
      return destination.GetFullPath();
   }

   std::string get_Place_to_Write_Archive(const FilePath &source_file) {
      if (destination.GetFullPath() == "") {
         return "";
      }
      std::string dest = get_New_Link_Name(source_file);
      if (dest == "") {
         return "";
      } else {
         FilePath dest1 = destination.Push(dest);
         return dest1.GetFullPath();
      }
   }//------------------------------------------------------------------------

   std::string
   get_New_Link_Name(const FilePath &source_file); //returns the file name that should be written into the archive copy of the file

   bool is_new_File(const FilePath &source_file) const;

   bool is_Name_in_Archive(const std::string &name) const {
      std::map<std::string, std::string>::const_iterator it;
      for (it = mapper.begin(); it != mapper.end(); it++) {
         std::string value = (*it).second;
         if (name == value) {
            return true;
         }
      }
      return false;
   }//look for name in the value column of the map and return true if found
   //------------------------------------------------------------------------

   //mainly for debug purposes
   void print_Map() {
      std::map<std::string, std::string>::iterator it;
      for (it = mapper.begin(); it != mapper.end(); it++) {
         std::cout << (*it).first << " => " << (*it).second << std::endl;
      }
   }
   //------------------------------------------------------------------------

private:

   FilePath destination;
   std::map<std::string, std::string> mapper;


};
