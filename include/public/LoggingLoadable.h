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

#pragma once

#include "loader/Loadable.h"
#include "utility/Logging.h"


class LoggingLoadable : public Loadable
{
public:
   LoggingLoadable();

   LoggingLoadable(const LoggingLoadable &in);

   virtual ~LoggingLoadable();

   virtual void operator=(const LoggingLoadable &in);


   /**
    * Call this after you finish registering things at it will load all the things it could and return false if it fails
    *
    * @return
    * @see loaded_successfully()
    */
   virtual bool complete();

   virtual void report_error(std::string error_message);

   virtual void report_warning(std::string warning_message);

protected:

   virtual bool test_load();


private:
   static log4cplus::Logger m_logger;
};
