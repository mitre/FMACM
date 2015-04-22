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

/*
When you use this class you must specify both the type of the data you will be putting into it and the type you get when you multiply to objects of that type. 
For instance you could instantiate it with double, double 
Or Distance, Area 
*/

#include <math.h>
#include <vector>
#include "Utility_Functions.h"
#include <algorithm>

using std::vector;

#define SQR(A) ((A)*(A))

class Statistics
{
	public:
		Statistics(void)
		{
			s1 = 0;
		}

		~Statistics(void)
		{
		
		}

		virtual void insert(double value)
		{
			if(samples.size() == 0)
			{
				max = value;
				min = value;
			}
			else 
			{
				max = MAX(max, value);
				min = MIN(min, value);
			}

			samples.push_back(value);

			s1 += value;
		}

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

		double get_std()const
		{
			// Computes standard deviation over samples
			//
			// returns standard deviation or -1 if no samples to compute from.

			double sDev = -1.0;

			if (samples.size() > 0) {
			
				double variance_sum = 0.0;
				for(int loop = 0; loop < (int)samples.size(); loop++)
				{
					if( loop != 0 )
					{
						variance_sum += SQR(samples[loop] - get_mean());
					}
					else
					{
						variance_sum = SQR(samples[loop] - get_mean());
					}
				}
				sDev = sqrt(variance_sum / samples.size() );
			}
				
			return sDev;
		}

		//gwang 2013-11-08
		//input: pct is the percent (0 <= pct <= 1)
		double get_percentile(double pct)
		{
			int desired_sample_num;
			double result;
			vector<double> samples_sorted;

			//samples_sorted = samples;
			for(int i=0; i<samples.size(); i++)
			{
				double tmp = samples.at(i);
				samples_sorted.push_back(samples.at(i));
			}

			stable_sort(samples_sorted.begin(), samples_sorted.end());
			
			desired_sample_num = (int) (pct * samples.size() );
			
			if(desired_sample_num == 0) //handling exceptional case
			{
				result = samples_sorted.at(0); //We use this, although this is not true strictly (because 0-percentile is not defined)
			}
			else
			{
				result = samples_sorted.at(desired_sample_num-1);
			}

			return(result);			

		} // get_percentile
		//end gwang

		double get_bound95()
		{
			// Computes 95th bounds of the vector samples.  This
			// based on Lesley's matlab code.
			//
			// Returns:95th bound calculation.

			double bound95 = -1.0;

			double n = (double) samples.size();
			double prob = 0.0;
			double delta = 0.1;

			while (prob < 0.95) {
				double size = 0.0;

				for (int ix = 0; ix < samples.size(); ix ++) {
					if (fabs(samples[ix]) < delta) {
						size = size + 1.0;
					}
				}

				prob = size/n;

				bound95 = delta;
				delta = delta + 0.1;
			}

			return bound95;
		}


	private:
		double s1;          // sum of all values seen so far 
		vector<double> samples;
		double max;		  // private data members
		double min;		  // private data members
};
