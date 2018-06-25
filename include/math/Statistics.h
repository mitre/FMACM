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

/*
When you use this class you must specify both the type of the data you will be putting into it and the type you get when you multiply to objects of that type. 
For instance you could instantiate it with double, double 
Or Distance, Area 
*/

#include <vector>

using std::vector;

class Statistics
{
	public:
		Statistics(void);

		~Statistics(void);

		virtual void insert(double value);
		double get_std()const;
		inline double get_mean()const
		{
			return s1 / samples.size();
		}

		inline long get_number_of_samples()const
		{
			return samples.size();
		}

		inline double get_max()const
		{
			return max;
		}

		inline double get_min()const
		{
			return min;
		}
		//gwang 2013-11-08
		//input: pct is the percent (0 <= pct <= 1)
		double get_percentile(double pct);

		double get_bound95();

	private:
		double s1;          // sum of all values seen so far 
		vector<double> samples;
		double max;		  // private data members
		double min;		  // private data members
};
