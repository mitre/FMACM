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

#include <string>
#include <iostream>
#include <fstream>

class HTMLDump
{
public:
	HTMLDump(void);
	~HTMLDump(void);

	bool open(const std::string &file_name);
	void close();

	void dump(const std::string &data);
	void highlight_on(const std::string& color);
	void highlight_off();

	bool is_open()
	{
		return NULL != dump_file_name;
	}

private:

	std::ofstream dump_file_name;
};
