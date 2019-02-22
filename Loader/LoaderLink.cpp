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

#include "loader/LoaderLink.h"
#include "loader/LoadError.h"

using namespace std;

// log4cplus::Logger LoaderLink::logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("LoaderLink"));

LoaderLink::LoaderLink(void) {
   loaded = false;
   must_load = false;
   must_load_only_once = false;
   is_a_list = false;
}

LoaderLink::~LoaderLink(void) {

}

//---------------------------------------------------------

bool LoaderLink::load(DecodedStream *ds) {
   if (must_load_only_once && loaded) {
      string msg = "Attempted to load variable second time which is marked as load once";
      ds->report_error(string("\nERROR: ") + msg);
      // LOG4CPLUS_FATAL(logger, msg);
      throw LoadError(msg);
   }

   bool out = load_s(ds);

   if (!out) {
      string msg = "Problem loading variable ";
      ds->report_error(string("\nERROR: ") + msg);
      // LOG4CPLUS_FATAL(logger, msg);
      throw LoadError(msg);
   }

   if (loaded && !is_a_list) {

      ds->report_warning("\nWarning: This variable was already loaded once \n");
      // LOG4CPLUS_WARN(logger, "This variable was already loaded once");
   }

   loaded = true;

   return true;
}
