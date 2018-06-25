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

#include "utility/Logging.h"

/**
 * Random sample generator class.
 */

class RandomGenerator
{
  public:

    RandomGenerator();
	RandomGenerator(const double seed);
    ~RandomGenerator();

    void setSeed(const double seed);
    const double getSeed(void);

    const double uniformSample();
    const double gaussianSample();
    const double truncatedGaussianSample(const double maxstddev);
    const double rayleighSample();
    const double laplaceSample();

    /**
     * Returns a random sample from Gaussian distribution
     * with average = mean and standard deviation = sigma.
     */
    template <typename T>
    const T gaussianSample(const T mean, const T sigma) {
    	T v1 = mean + sigma * gaussianSample();
    	return v1;
    }

    /**
     * Returns a random sample from Gaussian distribution
     * with average = mean and standard deviation = sigma,
     * with the deviation not exceeding maxstddev * sigma.
     */
    template <typename T>
    const T truncatedGaussianSample(const T mean, const T sigma,
					 const double maxstddev) {
/*      Logging only works if T is a SpecificUnit or other streamable.
    	LOG4CPLUS_TRACE(logger, "Params:  mean=" << mean
    			<< ", sigma=" << sigma << ", maxstddev=" << maxstddev);*/
    	// check for zero standard deviation
    	if (sigma * 0 == sigma) {
    		return mean;
    	}
    	T v1 = mean + sigma * truncatedGaussianSample(maxstddev);
    	return v1;
    }

    /**
     * Returns a random sample from a Raleigh distribution sample with
     * the given mean and standard deviation.
     */
    template <typename T>
    const T rayleighSample(const T mean, const T sigma) {
    	T v1 = mean + sigma * rayleighSample();
    	return v1;
    }

    /**
     * Returns a random sample from a LaPlace distribution with
     * the given value of lambda.
     */
    template <typename T>
    const T laplaceSample(const T lambda) {
    	T v1 = lambda * laplaceSample();
    	return v1;
    }


  private:
    static log4cplus::Logger logger;

    static const double IA;
    static const double IM;
    static const double AM;
    static const double IQ;
    static const double IR;

    double mSeed;

};
