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
#include "LoaderLink.h"

template <class DATA>
class NamedElementLoaderLink : public LoaderLink
{
public:

	NamedElementLoaderLink(list<DATA> *address)
	{
		var_address = address;
		must_load_only_once = false;
		is_a_list = true;
	}

	//-------------------------------------------------

	bool load_s(DecodedStream *ds)
	{
		DATA temp;
		string token;
		bool f;

		// read off the {

		f = ds->get_datum(token);
		
		if(!f)
		{
			ds->highlight_on();
			ds->report_error("\nload atempt failed\n");
			ds->highlight_off();
			exit(-207);  
		}

		if(token.compare("{") != 0)
		{
			ds->highlight_on();
			ds->report_error("\nload atempt failed\n");
			ds->highlight_off();
			exit(-208);
		}

		// load the entry ----------------------------------------------

		f = temp.load(ds);

		if(!f)
		{
			ds->highlight_on();
			ds->report_error("\nload atempt failed\n");
			ds->highlight_off();
			exit(-209);
		}

		// add it to the list ----------------------------------------------

		var_address->push_back(temp);

		// get the end of the entry ----------------------------------------------

		f = ds->get_datum(token);

		if(!f)
		{
			ds->highlight_on();
			ds->report_error("\nload atempt failed\n");
			ds->highlight_off();
			exit(-210);
		}

		// look for the } at the end of the list ----------------------------------------------

		if( token.compare("}") == 0)
		{
			return true;
		}
		else
		{
			ds->highlight_on();
			ds->report_error("\nload atempt failed\n");
			ds->highlight_off();
			exit(-211);
		}

		return true;
	}

private:
	list<DATA> *var_address;
};
