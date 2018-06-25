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

#include "loader/DecodedStream.h"
/* #include "utility/Logging.h" */

/**
 * Meta info for the concept of deprecated in the Loadable system.
 */
struct LoaderDeprecatedMetaInfo {
	bool isDeprecated;
	std::string supersededByTagName;
//	std::string deprecatedInVersion; // keep this commented out until a final concept of versioning is implemented in AAESim
};

class LoaderLink
{
public:
	LoaderLink(void);
	
	virtual ~LoaderLink () = 0;

	bool load(DecodedStream *ds);

	virtual bool load_s(DecodedStream *ds) = 0;

	//-----------------------------------------------------------

	bool get_loaded_status()
	{
		return loaded;
	}

	//-----------------------------------------------------------

	bool is_a_must_load()
	{
		return must_load;
	}

	//-----------------------------------------------------------

	void set_must_load(bool b)
	{
		must_load = b;
	}
	
	//-----------------------------------------------------------

	bool was_loaded()
	{
		return loaded;
	}

	//-----------------------------------------------------------

	bool ok()
	{
		if(!must_load)
		{
			return true;
		}
		else if(loaded)
		{
			return true;
		}
		else
		{
			return false;
		}
	}	

	void set_deprecated_info(LoaderDeprecatedMetaInfo info) {
		deprecatedInfo = info;
	}

	LoaderDeprecatedMetaInfo get_deprecated_info() {
		return deprecatedInfo;
	}

protected:

	bool loaded;
	bool must_load; // if true you must load one or more times
	bool must_load_only_once; // if true you can only load it once or 0 times 
	bool is_a_list; // if set high the two above do not apply
	LoaderDeprecatedMetaInfo deprecatedInfo;

private:
	/* static log4cplus::Logger logger; */

};
