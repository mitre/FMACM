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

#pragma once

#include "public/Logging.h"

class RandomGenerator final {
  public:
   RandomGenerator();

   RandomGenerator(const double seed);

   ~RandomGenerator() = default;

   void SetSeed(const double seed);

   const double GetSeed();

   const double UniformSample();

   const double GaussianSample();

   const double TruncatedGaussianSample(const double max_standard_deviation);

   const double RayleighSample();

   const double LaplaceSample();

   /**
    * Returns a random sample from Gaussian distribution
    * with average = mean and standard deviation = sigma.
    */
   template <typename T>
   const T GaussianSample(const T mean, const T sigma) {
      T v1 = mean + sigma * GaussianSample();
      return v1;
   }

   /**
    * Returns a random sample from Gaussian distribution
    * with average = mean and standard deviation = sigma,
    * with the deviation not exceeding maxstddev * sigma.
    */
   template <typename T>
   const T TruncatedGaussianSample(const T mean, const T sigma, const double max_standard_deviation) {
      // check for zero standard deviation
      if (sigma * 0 == sigma) {
         return mean;
      }
      T v1 = mean + sigma * TruncatedGaussianSample(max_standard_deviation);
      return v1;
   }

   /**
    * Returns a random sample from a Raleigh distribution sample with
    * the given mean and standard deviation.
    */
   template <typename T>
   const T RayleighSample(const T mean, const T sigma) {
      T v1 = mean + sigma * RayleighSample();
      return v1;
   }

   /**
    * Returns a random sample from a LaPlace distribution with
    * the given value of lambda.
    */
   template <typename T>
   const T LaplaceSample(const T lambda) {
      T v1 = lambda * LaplaceSample();
      return v1;
   }

  private:
   static log4cplus::Logger m_logger;

   static const double m_IA;
   static const double m_IM;
   static const double m_AM;
   static const double m_IQ;
   static const double m_IR;

   double m_seed;
};
