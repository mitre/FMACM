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
// Copyright 2017 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#include "utility/Logging.h"
#include <log4cplus/configurator.h>
#include <math.h>
#include <unistd.h>

static bool Logging_initialized = false;

void init_logging() {
	if (Logging_initialized) return;
	Logging_initialized = true;

	log4cplus::initialize();
	log4cplus::Logger logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("logging.init"));
	char *prop_file = getenv("LOG4CPLUS_PROPERTIES");
    if (prop_file == NULL) {
        // fall back to log4cplus.properties
        prop_file = (char *) "log4cplus.properties";
    }
    if (access(prop_file, F_OK) == -1) {
		log4cplus::BasicConfigurator config;
		config.configure();
		LOG4CPLUS_WARN(logger, "Cannot access LOG4CPLUS_PROPERTIES " 
            << prop_file << ", using BasicConfigurator.");
	}
	else {
		log4cplus::PropertyConfigurator config(prop_file);
		config.configure();
		LOG4CPLUS_INFO(logger, "LOG4CPLUS_PROPERTIES file is " <<
				LOG4CPLUS_TEXT(prop_file));
	}

	// Example usage of the debug logger
//	LOG4CPLUS_DEBUG(logger, "Pi is " << (4 * atan(1)) << " approximately");
}
