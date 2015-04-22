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
// Copyright 2015 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#pragma once
#include "Token.h"
#include "MoreStringFunctions.h"

template<class PARENT>
class StdoutStreamEcho : public PARENT
{
public:

	StdoutStreamEcho(void)
	{
	}

	~StdoutStreamEcho(void)
	{
	}

	Token get_next()
	{
		Token out;
		out = PARENT::get_next();
		string outdata = out.get_Format();
		if(PARENT::get_echo())
		{
			string enhanced_format = MoreStringFunctions::find_and_replace(outdata, "\n", string("\n")+PARENT::get_Space());
			cout << enhanced_format << out.get_Data();
		}

		return out;
	}

	void report_error(string message)
	{
			cout << message;
			PARENT::report_error(message);
	}

	void report_warning(string message)
	{
			cout << message;
			PARENT::report_warning(message);
	}
};
