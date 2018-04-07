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

#include "loader/HTMLDump.h"
#include <assert.h>

using namespace std;

string HTMLDump::SoftwareVersion("unset");

void HTMLDump::SetSoftwareVersion(const string& softwareVersion) {
	SoftwareVersion = softwareVersion;
}

HTMLDump::HTMLDump(void)
{
}

//-----------------------------------------------------------

HTMLDump::~HTMLDump(void)
{
	if(dump_file_name.is_open())
	{
		close();
	}
}

//-----------------------------------------------------------

bool HTMLDump::open(const string &file_name)
{
	dump_file_name.open(file_name.c_str());

	if(dump_file_name.is_open())
	{
		dump_file_name << "<html>\n<body bgcolor=dddddd>\n<pre>\n";
		dump_file_name << "running " << SoftwareVersion << endl;
	}

	return dump_file_name.is_open();
}

//-----------------------------------------------------------

void HTMLDump::close()
{
	assert(dump_file_name != NULL);
	assert(dump_file_name.is_open());
	
	dump_file_name << "\n</pre>\n</body>\n </html>";

	dump_file_name.close();
}

//-----------------------------------------------------------

void HTMLDump::dump(const string &data)
{
	dump_file_name << data;
}

//-----------------------------------------------------------

void HTMLDump::highlight_on(const string& color)
{
	string first = "<FONT style=\"BACKGROUND-COLOR: ";
	string last = "\"\\>";
	 dump_file_name << first + color + last ; 
}

//-----------------------------------------------------------

void HTMLDump::highlight_off()
{
	dump_file_name << "</FONT>";
}

