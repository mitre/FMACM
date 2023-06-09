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

#include "public/OutputHandler.h"
#include <stdexcept>

log4cplus::Logger OutputHandler::m_logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("OutputHandler"));

OutputHandler::OutputHandler(const std::string &scenario_name, const std::string &file_suffix)
   : m_file_suffix(file_suffix), filename(scenario_name + file_suffix), os(), m_finished(false) {}

OutputHandler::~OutputHandler() {
   if (!m_finished) {
      /*
       * Either Finish() was never called, or it failed to set m_finished to true.
       *
       * Actually calling Finish() from here results in a pure virtual error
       * because by the time this destructor is called, the sub-destructors
       * have already completed, destroying the Finish() implementation.
       *
       * Second-best option:  log and throw
       */
      std::string message("Unfinished output; " + filename + " not written.");
      LOG4CPLUS_ERROR(m_logger, message);
      throw std::logic_error(message);
   }
}
