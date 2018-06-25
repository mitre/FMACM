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

/*
When you use this class you must specify both the type of the data you will be putting into it and the type you get when you multiply to objects of that type. 
For instance you could instantiate it with double, double 
Or Distance, Area 
 */

#include "math/Statistics.h"
#include <math.h>
#include <algorithm>


Statistics::Statistics(void)
{
	s1 = 0;
	max = 0.0;
	min = 0.0;
}

Statistics::~Statistics(void)
{

}

void Statistics::insert(double value)
{
	if(samples.size() == 0)
	{
		max = value;
		min = value;
	}
	else
	{
		max = std::max(max, value);
		min = std::min(min, value);
	}

	samples.push_back(value);

	s1 += value;
}

double Statistics::get_std()const
{
	// Computes standard deviation over samples
	//
	// returns standard deviation or -1 if no samples to compute from.

	// TODO see if this is the correct formula
	// wrong formula
	// return sqrt(s2 * number_of_samples - s1 * s1)/number_of_samples;

	//double dof = 1/(number_of_samples - 1);
	//return sqrt( dof*s2 - SQR( get_mean() ) );

	//return sqrt( fabs((s2 / (number_of_samples - 1.0)) - (number_of_samples)*(get_mean()*get_mean())/(number_of_samples - 1.0)) );

	// Pierre C. May 2013, formula is still wrong, (Sum^2 - N*Mean()^2)/N != ( SUM((s - mean)^2)/N

	double sDev = -1.0;

	if (samples.size() > 0) {

		double variance_sum = 0.0;
		for(int loop = 0; loop < (int)samples.size(); loop++)
		{
			if( loop != 0 )
			{
				variance_sum += pow(samples[loop] - get_mean(), 2);
			}
			else
			{
				variance_sum = pow(samples[loop] - get_mean(), 2);
			}
		}
		sDev = sqrt(variance_sum / samples.size() );
	}

	return sDev;
}

//gwang 2013-11-08
//input: pct is the percent (0 <= pct <= 1)
double Statistics::get_percentile(double pct)
{
	int desired_sample_num;
	double result;
	vector<double> samples_sorted;

	//samples_sorted = samples;
	for(unsigned int i=0; i<samples.size(); i++)
	{
		double tmp = samples.at(i);
		samples_sorted.push_back(tmp);
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

double Statistics::get_bound95()
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

		for (unsigned int ix = 0; ix < samples.size(); ix ++) {
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
