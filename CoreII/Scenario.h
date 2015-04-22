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

#include "Loadable.h"
#include "string"
#include "list"
#include <string>
#include <fstream>
#include <iostream>
#include <vector>

/**
 * This is a very basic Scenario implementation. Expanded implementations should not edit this class but inherit from it.
 */
class Scenario: public Loadable 
{
public:
	Scenario(void);
	~Scenario(void);
	virtual void process_one_scenario();
	virtual bool load(DecodedStream *input);

	// Scenario name getter/setters. Putting default impl here for simplicity.
	virtual string get_scenario_name() {
		return mScenarioName;
	}

	virtual void set_scenario_name(string in) {
		mScenarioName = in;

		// remove the leading directory structure if present (search for last instance of "/" or "\\")
		int index;
		index = mScenarioName.find_last_of("/\\");
		if( index != string::npos)
		{
		  mScenarioName = mScenarioName.substr(index+1); // sets the string to after the last "/" or "\\"
		}

		// check for .txt and remove it if present
		index = mScenarioName.find(".txt");
		if( index != string::npos)
		{
		  // erases the .txt from the string
		  mScenarioName.erase(index, 4);
		}
	}

private:

	string mScenarioName;

};
