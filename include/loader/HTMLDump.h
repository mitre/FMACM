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
#include <fstream>

class HTMLDump {
  public:
   static void SetSoftwareVersion(const std::string &version);

   HTMLDump(void);

   ~HTMLDump(void);

   bool open(const std::string &file_name);

   void close();

   void dump(const std::string &data);

   void highlight_on(const std::string &color);

   void highlight_off();

   bool is_open() { return dump_file_name.is_open(); }

  private:
   static std::string SoftwareVersion;
   std::ofstream dump_file_name;
};
