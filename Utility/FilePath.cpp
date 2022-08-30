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

#include "utility/FilePath.h"

using namespace std;


FilePath::FilePath() {
}

FilePath::~FilePath() {
}

FilePath::FilePath(const string &name) {
   m_error = false;
   int end;
   int index;

   if (name.length() == 0) {
      m_error = true;
   }

   m_full_path = name;

   ExtractPop(name);
   end = ExtractType(name);
   index = ExtractDisk(name);

   ExtractPath(name, index, end);

   m_number_of_wildcards = GetNumberOfWildCards(name);

   if (m_list_of_directories.size() == 0) {
      m_file_name = "";
   } else {
      m_file_name = m_list_of_directories.at(m_list_of_directories.size() - 1);
   }

   size_t find_space;
   find_space = name.find_last_of(" ");
   if (find_space != string::npos) {
      if (find_space == name.length() - 1) {
         m_error = true;
      }
   }
}

bool FilePath::operator==(const FilePath &rhs) const {
   if (m_full_path == rhs.m_full_path) {
      return true;
   }

   if (m_number_of_wildcards > 0) {
      size_t found = m_full_path.find("*");
      if (found == 0) {
         std::string end = m_full_path.substr(1);
         if (rhs.m_full_path.substr((rhs.m_full_path.length() - end.length())) == end) {
            return true;
         }
      } else {
         if (rhs.m_full_path.find(m_full_path.substr(0, found - 1)) != std::string::npos) {
            return true;
         }
      }
   }

   if (rhs.m_number_of_wildcards > 0) {
      size_t found = rhs.m_full_path.find("*");
      if (found == 0) {
         std::string end = rhs.m_full_path.substr(1);
         if (m_full_path.substr((m_full_path.length() - end.length())) == end) {
            return true;
         }
      } else {
         if (rhs.m_full_path.find(m_full_path.substr(0, found - 1)) != std::string::npos) {
            return true;
         }
      }
   }
   return false;
}

string FilePath::GetFullPath() const {
   return m_full_path;
}

string FilePath::GetType() const {
   return m_type;
}

string FilePath::GetDisk() const {
   return m_drive;
}

string FilePath::RemoveLastDirectory() const {
   return m_last_directory;
}

int FilePath::GetNumberOfDirectories() const {
   return m_number_of_directories;
}

vector<string> FilePath::ListDirectories() const {
   return m_list_of_directories;
}

FilePath FilePath::Pop() const {
   FilePath out = FilePath(m_last_directory);
   return out;
}

FilePath FilePath::Push(const FilePath &more) const {
   string pushed;
   string last = m_full_path.substr(m_full_path.length() - 1, 1);
   if (last == "\\" || last == "/") {
      pushed = m_full_path + more.m_full_path;
   } else {
      pushed = m_full_path + "/" + more.m_full_path;
   }
   FilePath out = FilePath(pushed);
   return out;
}

FilePath FilePath::Cd(const FilePath &fp) const {
   string full = fp.m_full_path;
   string full1 = m_full_path;
   FilePath temp = FilePath(full1);
   FilePath out, temp2;
   if (full.substr(0, 2) == "..") {
      if (full == ".." || full == "../" || full == "..\\") {
         out = temp.Pop();
      } else if (full.substr(0, 3) == "..\\" || full.substr(0, 3) == "../") {
         temp2 = temp.Pop();
         out = temp2.Push(full.substr(3));
      } else {
         size_t col = full.find_first_of(":");
         int len = full.length();
         if (col != string::npos && col == len - 1) {
            out = fp;
         } else {
            out = temp;
         }
      }
   } else {
      out = temp.Push(fp);
   }

   return out;
}

int FilePath::ExtractDisk(const string &name) {
   if (name.find_first_of(":") != string::npos) {
      m_drive = name.substr(0, name.find_first_of(":"));
      if ((name.length() - 1) == name.find_first_of(":")) {
         m_error = true; //return error if path is just C:
         return -1; // added by ADM jan 24 2011
      } else {
         return name.find_first_of(":") + 1;
      }
   } else {
      m_drive = "";
      return 0;
   }

}

int FilePath::ExtractType(const string &name) {
   size_t found;
   found = name.find_last_of(".");
   if (found == string::npos) {
      m_type = "";
      return name.length();
   } else {
      //a path cannot have a period at the end
      if (found == name.length() - 1) {
         m_error = true;
      }
      m_type = name.substr(found + 1);
      return int(found);
   }
}

void FilePath::ExtractPath(const string &name,
                           int &index,
                           int end) {
   m_number_of_directories = 0;
   string temp = name;
   while (index < end) {
      size_t slash = temp.find_first_of("/");
      string directory;
      if (slash != string::npos) {
         directory = name.substr(index, slash + 1);
         m_list_of_directories.push_back(directory);
         m_number_of_directories++;
         index += directory.size();
         temp = name.substr(index);
      } else {
         size_t dot = temp.find_first_of(".");
         directory = name.substr(index, dot);
         m_list_of_directories.push_back(directory);
         m_number_of_directories++;
         index += directory.size();
         temp = name.substr(index);
      }
   }
}

void FilePath::ExtractPop(const string &rname) {
   size_t slash = rname.find_last_of("/");
   m_last_directory = rname.substr(0, slash + 1);
}

int FilePath::GetNumberOfWildCards(const string &name) {
   int len = m_full_path.length();
   int index = 0;
   int number = 0;
   while (index < len) {
      if (m_full_path.substr(index, 1) == "*") {
         number++;
      }
      index++;
   }
   return number;
}
