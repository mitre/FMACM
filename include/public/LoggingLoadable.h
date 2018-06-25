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

#pragma once

#include "loader/Loadable.h"
#include "utility/Logging.h"


class LoggingLoadable : public Loadable
{
public:
   LoggingLoadable(void);
   LoggingLoadable(const LoggingLoadable &in);
   virtual ~LoggingLoadable(void);
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
   static log4cplus::Logger logger;
};
