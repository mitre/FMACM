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

#include "math/RandomGenerator.h"
#include <math.h>
#include <time.h>
#include <assert.h>
#include "utility/constants.h"


const double RandomGenerator::IA = 16807;
const double RandomGenerator::IM = 2147483647;
const double RandomGenerator::AM = 1.0 / IM;
const double RandomGenerator::IQ = 127773.0;
const double RandomGenerator::IR = 2836.0;

log4cplus::Logger RandomGenerator::logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("RandomGenerator"));


RandomGenerator::RandomGenerator()
{
  // should never initialize to zero
  double seed = time(NULL);
  long k = seed / IM;
  mSeed = seed - (k * IM);
}

RandomGenerator::RandomGenerator(const double seed)
{
	assert(seed != 0.0);
	long k = seed / IM;
	mSeed = seed - (k * IM);
}

RandomGenerator::~RandomGenerator()
{
}

//generate a uniform random number between 0 and 1
//From "Numerical Recipe"
const double RandomGenerator::uniformSample()
{
  long k = mSeed / IQ;

  mSeed = IA * (mSeed - k * IQ) - IR * k;

  if (mSeed < 0.0)
  {
    mSeed += IM;
  }

  double sample = AM * mSeed;

  LOG4CPLUS_TRACE(logger, mSeed << "," << sample);

  return sample;
}

// returns gaussian distributed random variable with mean 0 and standard
// deviation 1.
// usage example:  x = gauss(0., 32.);
const double RandomGenerator::gaussianSample()
{
  double u1 = uniformSample();
  double u2 = uniformSample();

  double eln = -2.0 * log(u1);     // ALOG(U1)
  double ang =  2.0 * M_PI * u2;

  double v1  = sqrt(eln) * cos(ang);

  LOG4CPLUS_TRACE(logger, mSeed);

  return v1;
}

// returns truncated gaussian random varialbe with mean 0, standard
// deviation 1, and maximum standard deviation max_std_dev
const double RandomGenerator::truncatedGaussianSample(
                        const double maxstddev)
{
  double val = maxstddev + 1.0;

  while (val > (maxstddev)  ||  val < ( - maxstddev))
  {
    val = gaussianSample();
  }
  LOG4CPLUS_TRACE(logger, mSeed);

  return val;
}

// returns Rayleigh distributed random variable with mean 0 and standard
// deviation 1.
// usage example:  x = Rayleigh(0., 32.);
const double RandomGenerator::rayleighSample()
{
  double u1 = uniformSample();
  double v1 = (sqrt(-2.0*log(u1))-1.253)/sqrt(0.429);

  return v1;
}

// returns laplacian r.v. with lambda=1.
const double RandomGenerator::laplaceSample()
{
  double uni = uniformSample();
  double err = -log(uni);
  uni = uniformSample();

  if (uni < 0.5)
  { 
    err = -err;
  }

  return err ;
}

void RandomGenerator::setSeed(const double seed)
{
  // not allowed to set seed to zero
  assert(seed != 0.0);
  mSeed = seed;
}

const double RandomGenerator::getSeed(void)
{
  return mSeed;
}
