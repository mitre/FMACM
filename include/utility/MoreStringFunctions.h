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

#pragma once

#include <string>

class MoreStringFunctions
{
public:
  MoreStringFunctions(void) {}
  ~MoreStringFunctions(void) {}

	// note data is passed by value so we can modify it and return the new copy 

	static std::string find_and_replace(std::string data,const std::string &search, const std::string &replace)
	{
		size_t pos = 0;

		while(true)
		{
			pos = data.find(search, pos);
			
			if(pos == std::string::npos)
			{
				break;
			}

			data.replace(pos, search.length(), replace);
			pos += replace.length();
		}

		return data;
	}
};
