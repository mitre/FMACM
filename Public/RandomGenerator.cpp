// ****************************************************************************
// NOTICE
//
// This work was produced for the U.S. Government under Contract 693KA8-22-C-00001
// and is subject to Federal Aviation Administration Acquisition Management System
// Clause 3.5-13, Rights In Data-General, Alt. III and Alt. IV (Oct. 1996).
//
// The contents of this document reflect the views of the author and The MITRE
// Corporation and do not necessarily reflect the views of the Federal Aviation
// Administration (FAA) or the Department of Transportation (DOT). Neither the FAA
// nor the DOT makes any warranty or guarantee, expressed or implied, concerning
// the content or accuracy of these views.
//
// For further information, please contact The MITRE Corporation, Contracts Management
// Office, 7515 Colshire Drive, McLean, VA 22102-7539, (703) 983-6000.
//
// 2023 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#include "public/RandomGenerator.h"
#include <math.h>
#include <time.h>
#include <assert.h>
#include "utility/UtilityConstants.h"

const double RandomGenerator::m_IA = 16807;
const double RandomGenerator::m_IM = 2147483647;
const double RandomGenerator::m_AM = 1.0 / m_IM;
const double RandomGenerator::m_IQ = 127773.0;
const double RandomGenerator::m_IR = 2836.0;

log4cplus::Logger RandomGenerator::m_logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("RandomGenerator"));

RandomGenerator::RandomGenerator() {
   double seed = time(NULL);
   long k = seed / m_IM;
   m_seed = seed - (k * m_IM);
}

RandomGenerator::RandomGenerator(const double seed) {
   assert(seed != 0.0);
   long k = seed / m_IM;
   m_seed = seed - (k * m_IM);
}

const double RandomGenerator::UniformSample() {
   long k = m_seed / m_IQ;

   m_seed = m_IA * (m_seed - k * m_IQ) - m_IR * k;

   if (m_seed < 0.0) {
      m_seed += m_IM;
   }

   double sample = m_AM * m_seed;

   LOG4CPLUS_TRACE(m_logger, m_seed << "," << sample);

   return sample;
}

const double RandomGenerator::GaussianSample() {
   double u1 = UniformSample();
   double u2 = UniformSample();

   double eln = -2.0 * log(u1);
   double ang = 2.0 * M_PI * u2;

   double v1 = sqrt(eln) * cos(ang);

   LOG4CPLUS_TRACE(m_logger, m_seed);

   return v1;
}

const double RandomGenerator::TruncatedGaussianSample(const double max_standard_deviation) {
   double val = max_standard_deviation + 1.0;

   while (val > (max_standard_deviation) || val < (-max_standard_deviation)) {
      val = GaussianSample();
   }
   LOG4CPLUS_TRACE(m_logger, m_seed);

   return val;
}

const double RandomGenerator::RayleighSample() {
   double u1 = UniformSample();
   double v1 = (sqrt(-2.0 * log(u1)) - 1.253) / sqrt(0.429);

   return v1;
}

const double RandomGenerator::LaplaceSample() {
   double uni = UniformSample();
   double err = -log(uni);
   uni = UniformSample();

   if (uni < 0.5) {
      err = -err;
   }

   return err;
}

void RandomGenerator::SetSeed(const double seed) {
   assert(seed != 0.0);
   m_seed = seed;
}

const double RandomGenerator::GetSeed(void) { return m_seed; }
