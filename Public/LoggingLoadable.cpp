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
// Copyright 2018 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#include "public/LoggingLoadable.h"
#include "loader/LoadError.h"

using namespace std;

log4cplus::Logger LoggingLoadable::m_logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("LoggingLoadable"));

LoggingLoadable::LoggingLoadable(void) {
   stream = NULL;
   was_load_successful = false;
}

LoggingLoadable::~LoggingLoadable(void) {
   cleanup();
}

LoggingLoadable::LoggingLoadable(const LoggingLoadable &in) {
   was_load_successful = in.was_load_successful;
   stream = in.stream;
}

void LoggingLoadable::operator=(const LoggingLoadable &in) {
   was_load_successful = in.was_load_successful;
   stream = in.stream;
}

bool LoggingLoadable::test_load() {
   string error_message;
   bool good = true;

   for (map<string, shared_ptr<LoaderLink> >::iterator it = lookup_table.begin(); it != lookup_table.end(); it++) {
      if (!(*it).second->ok()) {
         error_message = "\nERROR: Did not load required tag '" + (*it).first + "'";
         stream->report_error(error_message + "\n");
         LOG4CPLUS_FATAL(m_logger, error_message);
         throw LoadError(error_message);

      } else {
         // It was okay, but is it default
         bool isdefault = (!((*it).second->is_a_must_load())) &&
                          (!(*it).second->get_loaded_status()); // NOT required && NOT loaded == must be using a default
         bool isDeprecated = (*it).second->get_deprecated_info().isDeprecated;
         if (isdefault && !isDeprecated) {
            string msg = "The optional tag \"" + (*it).first +
                         "\" was not found. Its default value will be used."; // unfortunately, the default value is unknown here and cannot be included in the message
            stream->report_warning(msg + "\n");
            // LOG4CPLUS_WARN(logger, msg);
         }
      }
   }

   return good;
}


bool LoggingLoadable::complete() {
   bool f;
   string token;
   string warning_message;

   while (true) {
      //dump();
      f = stream->get_datum(token); //read a token

      if (!f) // if there are no more jump out
      {
         break;
      }

      //-------------------------------------------------------------------

      token = clean_token(token);

      map<string, shared_ptr<LoaderLink> >::iterator it = lookup_table.find(token);

      if (it == lookup_table.end()) {
         // there was no match for the token in the lookup_table
         if (token == "}") {
            stream->push_back();
            was_load_successful = test_load();
            cleanup();
            return was_load_successful;
         } else {
            // Create warning statement since the token was not expected.
            warning_message = "\nThe unexpected tag \"" + token +
                              "\" appeared in the run file. \nThis tag was not registered in the load function; it will be ignored.";
            stream->report_warning(warning_message + "\n");
            LOG4CPLUS_WARN(m_logger, warning_message);
            stream->get_next(); // skip over the next token also, assuming it is a value associated with the unknown tag
            continue;
         }
      }

      // The token has been found in the lookup_table. Check its deprecated status.
      if ((*it).second->get_deprecated_info().isDeprecated) {
         // warn since this is a deprecated token
         warning_message = " The tag \"" + token + "\" appeared in a loaded file, but is deprecated. ";
         if (!((*it).second->get_deprecated_info().supersededByTagName.empty())) {
            if ((*it).second->get_deprecated_info().supersededByTagName == "unused") {
               warning_message += "This variable is unused.  ";
            } else if ((*it).second->get_deprecated_info().supersededByTagName == "hard coded") {
               warning_message += "This variable is hard coded to the default value: input not used. ";
            } else {
               warning_message +=
                     "It has been superseded by: " + (*it).second->get_deprecated_info().supersededByTagName;
               warning_message += "\nThe old tag will be used for now, but may be removed in the future. Please update your input files.\n";
            }
         } else {
            warning_message += "\nThe old tag will be used for now, but may be removed in the future. Please update your input files.\n";
         }

         stream->report_warning(warning_message);
         LOG4CPLUS_WARN(m_logger, warning_message);
      }


      bool f2 = (*it).second->load(stream);
      if (!f2) {
         string error_message = "\nERROR The tag \"" + token + "\" could not be loaded";
         stream->report_error(error_message);
         LOG4CPLUS_FATAL(m_logger, error_message);
         throw LoadError(error_message);
      }
   }

   was_load_successful = test_load();
   cleanup();

   return was_load_successful;
}

//--------------------------------------------------------

void LoggingLoadable::report_error(string error_message) {
   stream->report_error(error_message);
   LOG4CPLUS_FATAL(m_logger, error_message);
   throw LoadError(error_message);
}

void LoggingLoadable::report_warning(string warning_message) {
   stream->report_error(warning_message);
   LOG4CPLUS_WARN(m_logger, warning_message);
}
