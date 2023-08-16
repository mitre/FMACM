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

#include "loader/HTMLDump.h"
#include <assert.h>

using namespace std;

string HTMLDump::SoftwareVersion("unset");

void HTMLDump::SetSoftwareVersion(const string &softwareVersion) { SoftwareVersion = softwareVersion; }

HTMLDump::HTMLDump(void) {}

//-----------------------------------------------------------

HTMLDump::~HTMLDump(void) {
   if (dump_file_name.is_open()) {
      close();
   }
}

//-----------------------------------------------------------

bool HTMLDump::open(const string &file_name) {
   dump_file_name.open(file_name.c_str());

   if (dump_file_name.is_open()) {
      dump_file_name << "<html>\n<body bgcolor=dddddd>\n<pre>\n";
      dump_file_name << "running " << SoftwareVersion << endl;
   }

   return dump_file_name.is_open();
}

//-----------------------------------------------------------

void HTMLDump::close() {
   assert(dump_file_name.is_open());

   dump_file_name << "\n</pre>\n</body>\n </html>";

   dump_file_name.close();
}

//-----------------------------------------------------------

void HTMLDump::dump(const string &data) { dump_file_name << data; }

//-----------------------------------------------------------

void HTMLDump::highlight_on(const string &color) {
   string first = "<FONT style=\"BACKGROUND-COLOR: ";
   string last = "\"\\>";
   dump_file_name << first + color + last;
}

//-----------------------------------------------------------

void HTMLDump::highlight_off() { dump_file_name << "</FONT>"; }
