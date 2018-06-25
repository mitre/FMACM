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
#include <string>

class Token
{
public:
	Token(void)
	{
	}//--------------------------------------------------------------

	inline void add_Data(char c)
	{
		data += c;
	} //--------------------------------------------------------------
	inline void add_Data(const std::string &s)
	{
		data += s;
	} //--------------------------------------------------------------


	inline void add_Format(char c)
	{
		format += c;
	}//--------------------------------------------------------------
	inline void add_Format(const std::string &s)
	{
		format += s;
	}//--------------------------------------------------------------
	inline void add_Format(const Token &t)
	{
		add_Format(t.get_Format());
		add_Format(t.get_Data());
	}//--------------------------------------------------------------
	inline void merge_data_into_format()
	{
		format += data;
		data = "";
	}//--------------------------------------------------------------
	inline std::string get_Data()const
	{
		return data;
	}//--------------------------------------------------------------
	inline std::string get_Format()const
	{
		return format;
	}//--------------------------------------------------------------

	inline std::string get_All()const
	{
		return format + data;
	}//--------------------------------------------------------------

	inline void set_data(std::string const &nd)
	{
		data = nd;
	}//--------------------------------------------------------------
	inline void set_format(std::string const &nf)
	{
		format = nf;
	}//--------------------------------------------------------------

private:
	std::string data;
	std::string format;

};
