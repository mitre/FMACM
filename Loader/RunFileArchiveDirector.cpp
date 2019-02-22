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

#include "loader/RunFileArchiveDirector.h"
#include <sstream>
#include <map>

using namespace std;

RunFileArchiveDirector::RunFileArchiveDirector(void) {
}

RunFileArchiveDirector::~RunFileArchiveDirector(void) {
}

string RunFileArchiveDirector::get_New_Link_Name(const FilePath &source_file) {
   static int i = 1;

   stringstream ss;
   ss << i;

   string out;
   bool not_in_while = false;;
   string file = source_file.GetName();//file of the source; so need to create a function in FilePath that returns the name of the file like foo.txt
   string extension = source_file.GetType();
   while (true) {
      if (!not_in_while) {
         if (extension == "") {
            out = file;
            not_in_while = true;
         } else {
            out = file + "." + extension;
            not_in_while = true;
         }
      }
      if (!is_Name_in_Archive(out)) {
         //add to the map
         pair<map<string, string>::iterator, bool> ret;
         ret = mapper.insert(pair<string, string>(source_file.GetFullPath(), out));
         if (!ret.second) {
            return out;
         }
         break;
      } else {
         if (extension == "") {
            out = file + "_" + ss.str();
         } else {
            out = file + "_" + ss.str() + "." + extension;
         }
         i++;
      }
   }
   return out;

}//-----------------------------------------------------------------------------------

bool RunFileArchiveDirector::is_new_File(const FilePath &source_file) const {
   if (destination.GetFullPath() == "") {
      return false;
   }

   return !(mapper.find(source_file.GetFullPath()) == mapper.end());
}


