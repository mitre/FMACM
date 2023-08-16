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

#include <string>
#include <vector>

class FilePath {
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

   void ExtractPath(const std::string &name, int &index, int end);

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

inline std::string FilePath::GetName() const { return m_file_name; }
