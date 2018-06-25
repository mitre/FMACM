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
#include "public/Token.h"
#include "loader/HTMLDump.h"
#include <stdexcept>

template<class PARENT>
class EchoStream : public PARENT
{
public:

	EchoStream(void)
	{
	}

	~EchoStream(void)
	{
	}

	Token get_next()
	{
		Token out;

		while(true)
		{
			out = PARENT::get_next();
			
			if(out.get_Data() == "#echo_on")
			{
				PARENT::set_echo(true);
				continue;
			}

			if(out.get_Data() == "#echo_off")
			{
				PARENT::set_echo(false);
				continue;
			}
			if(out.get_Data() == "#echo_file")
			{
				std::string echo_file_name = PARENT::get_next().get_Data();

				// TODO fix this 

				bool out1 = open_echo_file(echo_file_name);
				if(!out1)
				{
					std::string msg = "Could not open specified dump file: " + echo_file_name;
					report_error(msg);
					throw std::runtime_error(msg);
				}
				continue;
			}
			if(out.get_Data() == "#end_echo_file")
			{
				close_echo_file();
				continue;
			}

			break;
		}
		
		// if echo is on and open write the echo
		if( echo_file.is_open() )
		{
			// logic for the output goes here.
			echo_file.dump(out.get_All());
		}

		return out;
	}


	bool open_echo_file(const std::string &my_echo_file)
	{
		echo_file.open(my_echo_file.c_str());

		if(echo_file.is_open())
		{
			return true;
		}
		return false;
	}

	void close_echo_file()
	{
		echo_file.close();
	}

	void report_error(std::string message)
	{
		PARENT::report_error(message); // reports the error to the parent class

		if( echo_file.is_open() )
		{
			// outputs the error to the echo_file if open
			echo_file.highlight_on("yellow");
			echo_file.dump(message);
			echo_file.highlight_off();
		}
	}

	void report_warning(std::string message)
	{
		PARENT::report_warning(message); // reports the warning to the parent class

		if( echo_file.is_open() )
		{
			// outputs the error to the echo_file if open
			echo_file.highlight_on("yellow");
			echo_file.dump(message);
			echo_file.highlight_off();
		}
	}

private:
	HTMLDump echo_file;

};
