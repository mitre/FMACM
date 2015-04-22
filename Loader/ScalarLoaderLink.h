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

/*
#pragma once
#include "LoaderLink.h"

template <class DATA>
class ScalarLoaderLink :	public LoaderLink
{
public:

	ScalarLoaderLink(DATA *address, DATA (*factory)(double))
	{
		var_address = address;
		factory_pointer = factory;
	}

	//-------------------------------------------------

	ScalarLoaderLink(DATA *address, DATA (*factory)(double), bool required, bool no_reset)
	{
		var_address = address;
		must_load = required;
		must_load_only_once = no_reset;
		factory_pointer = factory;
	}

	//-------------------------------------------------

	bool load_s(DecodedStream *ds)
	{
		double temp;
		bool out = ds->get_datum(*temp);

		if(out)
		{
			*var_address = factory_pointer(temp);
		}

		return out;
	}

private:

	DATA *var_address;
	DATA DATA::(*factory_pointer)(double);
};
*/
