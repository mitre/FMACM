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

#include <string>
#include <vector>

class FilePath
{
public:
   FilePath();

   virtual ~FilePath();

   FilePath(const std::string &name);

   std::string GetFullPath() const;

   std::string GetType() const;

   std::string GetDisk() const;

   std::string RemoveLastDirectory() const;

   int GetNumberOfDirectories() const;

   std::vector<std::string> ListDirectories() const;

   std::string GetName() const;

   FilePath Pop() const;

   FilePath Push(const FilePath &more) const;

   FilePath Cd(const FilePath &fp) const;

   bool operator==(const FilePath &rhs) const;

private:
   int ExtractDisk(const std::string &name);

   int ExtractType(const std::string &name);

   void ExtractPath(const std::string &name,
                    int &index,
                    int end);

   void ExtractPop(const std::string &name);

   int GetNumberOfWildCards(const std::string &name);

   std::vector<std::string> m_list_of_directories;

   std::string m_full_path;
   std::string m_drive;
   std::string m_type;
   std::string m_last_directory;
   std::string m_file_name;

   int m_number_of_directories;
   int m_number_of_wildcards;
   bool m_error;
};

inline std::string FilePath::GetName() const {
   return m_file_name;
}
