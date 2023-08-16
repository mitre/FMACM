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

#include "public/Logging.h"
#include <log4cplus/configurator.h>
#include <math.h>
#include <unistd.h>

static bool Logging_initialized = false;

void InitializeLogging() {
   if (Logging_initialized) {
      return;
   }
   Logging_initialized = true;

   log4cplus::initialize();
   log4cplus::Logger logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("logging.init"));
   char *prop_file = getenv("LOG4CPLUS_PROPERTIES");
   if (prop_file == NULL) {
      // fall back to log4cplus.properties
      prop_file = (char *)"log4cplus.properties";
   }
   if (access(prop_file, F_OK) == -1) {
      log4cplus::BasicConfigurator config;
      config.configure();
      LOG4CPLUS_TRACE(logger, "Cannot access LOG4CPLUS_PROPERTIES " << prop_file << ", using BasicConfigurator.");
   } else {
      log4cplus::PropertyConfigurator config(prop_file);
      config.configure();
      LOG4CPLUS_TRACE(logger, "LOG4CPLUS_PROPERTIES file is " << LOG4CPLUS_TEXT(prop_file));
   }
}
