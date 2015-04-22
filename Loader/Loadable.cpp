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

#include "Loadable.h"

Loadable::Loadable(void)
{
	was_load_successful = false;
}

Loadable::~Loadable(void)
{
	cleanup();
}

Loadable::Loadable(const Loadable &in)
{
	was_load_successful = in.was_load_successful;
	stream = in.stream;
}

void Loadable::operator=(const Loadable &in)
{
	was_load_successful = in.was_load_successful;
	stream = in.stream;
}

void Loadable::cleanup()
{
	for(map<string, LoaderLink*>::iterator it = lookup_table.begin(); it != lookup_table.end(); it++)
	{
		LoaderLink* temp = (it->second);
		delete temp;
	}

	lookup_table.clear();
}

bool Loadable::test_load()
{
	string error_message;
	bool good = true;

	for(map<string, LoaderLink*>::iterator it = lookup_table.begin(); it != lookup_table.end(); it++)
	{
		if(!(*it).second->ok())
		{
			error_message = "\nERROR: Did not load required tag " + (*it).first + "\n";
			
			stream->report_error(error_message);
		
			exit(-1);

		} else {
			// It was okay, but is it default
			bool isdefault = (!((*it).second->is_a_must_load())) && (!(*it).second->get_loaded_status()); // NOT required && NOT loaded == must be using a default
			if (isdefault) {
				string msg = "The optional tag \"" + (*it).first + "\" was not found. Its default value will be used.\n"; // unfortunately, the default value is unknown here and cannot be included in the message
				stream->report_warning(msg);
			}
		}
	}

	return good;
}


bool Loadable::complete()
{
	bool f;
	string token;
	string warning_message;

	while(true)
	{
		//dump();
		f = stream->get_datum(token); //read a token

		if(!f) // if there are no more jump out 
		{
			break;
		}

		//-------------------------------------------------------------------

		token = clean_token(token);

		map<string, LoaderLink*>::iterator it = lookup_table.find(token);

		if(it == lookup_table.end())
		{
			// there was no match for the token in the lookup_table
			if(token == "}")
			{
				stream->push_back();
				was_load_successful = true;
				return true;
			}
			else
			{
				// Create warning statement since the token was not expected.
				warning_message = "\nThe unexpected tag \"" + token + "\" appeared in the run file. \nThis tag was not registered in the load function; it will be ignored.\n" ;
				stream->report_warning(warning_message);
				stream->get_next(); // skip over the next token also, assuming it is a value associated with the unknown tag
				continue;
			}
		}

		// The token has been found in the lookup_table. Check its deprecated status.
		if ((*it).second->get_deprecated_info().isDeprecated)
		{
			// warn since this is a deprecated token
			string warningMessage = "\nThe tag \"" + token + "\" appeared in a loaded file, but is deprecated. ";
			if (!((*it).second->get_deprecated_info().supersededByTagName.empty()))
				warningMessage += "It has been superseded by: " + (*it).second->get_deprecated_info().supersededByTagName;
			warningMessage += "The old tag will be used for now, but may be removed in the future. Please update your input files.\n.";
			stream->report_warning(warningMessage);
		}


		bool f2 = (*it).second->load(stream);

		if(!f2)
		{
			string error_message = "\nERROR The tag \"" + token + "\" could not be loaded" ;
			stream->report_error(error_message);
			exit(-102);
		}
	}

	was_load_successful = test_load();

	cleanup();

	return was_load_successful;
}

//--------------------------------------------------------

bool Loadable::loaded_successfully()
{
	return was_load_successful;
}

//--------------------------------------------------------

void Loadable::report_error(string error_message)
{
	stream->report_error(error_message);
}

void Loadable::report_warning(string warning_message)
{
	stream->report_error(warning_message);
}

bool Loadable::loadAngleDegrees(Units::Angle &angle) {
	double value;
	bool result = stream->get_datum(value);
	angle = Units::DegreesAngle(value);
	return result;
}

bool Loadable::loadLengthFeet(Units::Length &length) {
	double value;
	bool result = stream->get_datum(value);
	length = Units::FeetLength(value);
	return result;
}

bool Loadable::loadSpeedKnots(Units::Speed &speed) {
	double value;
	bool result = stream->get_datum(value);
	speed = Units::KnotsSpeed(value);
	return result;
}

bool Loadable::loadAccelerationKnotsPerSecond(Units::Acceleration &acceleration) {
	double value;
	bool result = stream->get_datum(value);
	acceleration = Units::KnotsPerSecondAcceleration(value);
	return result;
}
