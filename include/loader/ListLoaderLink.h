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
#include "loader/LoaderLink.h"

template <class DATA>
class ListLoaderLink: public LoaderLink
{
public:

	ListLoaderLink(std::list<DATA> *address)
	{
		var_address = address;
		must_load_only_once = false;
		is_a_list = true;
	}

	//-------------------------------------------------

	bool load_s(DecodedStream *ds)
	{
		DATA temp;
		std::string token;
		bool f;

		f = ds->get_datum(token);
		
		if(!f)
		{
			return false;
		}

		if(token.compare("{") != 0)
		{
			return false;  
		}

		while(true)
		{
			f = ds->get_datum(token);

			if(!f)
			{
				return false;
			}
			
			if(token.compare("}") == 0) // look for the } at the end of the list 
			{
				return true; // dun 
			}
			else
			{
				ds->push_back(); // need to put the token back 
			}

			// load the entry 

			f = temp.load(ds);

			if(!f)
			{
				return false;
			}

			// add it to the list 

			var_address->push_back(temp);

		}
			
		return true;
	}

private:

	std::list<DATA> *var_address;
};
