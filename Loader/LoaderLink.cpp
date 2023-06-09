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

LoaderLink::~LoaderLink(void) {}

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
