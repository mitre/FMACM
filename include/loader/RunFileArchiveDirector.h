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
// Copyright 2020 The MITRE Corporation. All Rights Reserved.
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
