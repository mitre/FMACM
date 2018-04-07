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

#include "public/InvalidIndexException.h"
#include <iostream>
#include "utility/Logging.h"

using namespace std;

// NEED: Why are we using _MSC_VER here?  If it is to see what version of the Visual Studio
// we are working with, it should be out here and probably out of all the places it appears.

#ifdef _MSC_VER
// default constructor, calls super constructor
InvalidIndexException::InvalidIndexException(void) : exception("attempted to access an invalid index")
{
}

// cosntructor that takes a string error message argument and passes message to superconstructor
InvalidIndexException::InvalidIndexException(char* value) : exception(value)
{
}
#else
// default constructor, calls super constructor
InvalidIndexException::InvalidIndexException(const int invalid, const int lowValid, const int highValid) : exception()
{
	log4cplus::Logger logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("InvalidIndexException"));
	LOG4CPLUS_ERROR(logger, "attempted to access index " << invalid
			<< " between " << lowValid << " and " << highValid);
  //cout << "attempted to access an invalid index" << endl;
}

// cosntructor that takes a string error message argument and passes message to superconstructor
InvalidIndexException::InvalidIndexException(char* value) : exception()
{
  cout << value << endl;
}
#endif
